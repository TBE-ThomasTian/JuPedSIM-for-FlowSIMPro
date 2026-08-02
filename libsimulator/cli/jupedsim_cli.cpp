// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModel.hpp"
#include "CollisionFreeSpeedModelData.hpp"
#include "CollisionGeometry.hpp"
#include "FdsVisibilityField.hpp"
#include "LineSegment.hpp"
#include "GenericAgent.hpp"
#include "GeometryBuilder.hpp"
#include "Journey.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "Simulation.hpp"
#include "StageDescription.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <fmt/format.h>
#include <libdeflate.h>

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cctype>
#include <cstring>
#include <cmath>
#include <numbers>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
namespace fs = std::filesystem;
namespace pt = boost::property_tree;

constexpr uint32_t JSP_VERSION = 2;
constexpr uint32_t JSP_FLAG_DEFLATE = 1;
constexpr uint32_t JSP_RECORD_SIZE = 28; // u64 agent_id + 4x f32 + u32 floor_id
constexpr char JSP_META_MAGIC[4] = {'J', 'S', 'P', 'M'};
constexpr uint32_t JSP_META_VERSION = 1;
constexpr uint32_t JSP_META_RECORD_SIZE = 24; // u64 id + u8 age + u8 avatar + u16 + 3x f32
// Optional trailer describing where the storey sits in the source model, so a
// .jsp can be placed correctly without its scenario .xml next to it.
constexpr char JSP_FSP_MAGIC[4] = {'J', 'S', 'P', 'F'};
constexpr uint32_t JSP_FSP_VERSION = 1;
constexpr uint32_t JSP_FSP_RECORD_SIZE = 16; // 4x f32: elevation + centering xyz
// Optional FDS Smoke3D coupling metadata. The payload size makes this trailer
// extensible without changing the trajectory stream version.
constexpr char JSP_HAZARD_MAGIC[4] = {'J', 'S', 'P', 'H'};
constexpr uint32_t JSP_HAZARD_VERSION = 1;
constexpr uint32_t JSP_HAZARD_FIXED_PAYLOAD_SIZE = 56;
constexpr uint32_t JSP_HAZARD_FLAG_FDS_SMOKE3D = 1;
constexpr uint32_t JSP_HAZARD_FLAG_RELATIVE_PATH = 2;
constexpr int JSP_MIN_COMPRESSION_LEVEL = 1;
constexpr int JSP_MAX_COMPRESSION_LEVEL = 12;

constexpr std::streamoff HEADER_FRAME_COUNT_OFFSET = 28;
constexpr std::streamoff HEADER_INDEX_OFFSET = 36;

constexpr uint8_t AGE_GROUP_UNKNOWN = 0;
constexpr uint8_t AGE_GROUP_YOUNG = 1;
constexpr uint8_t AGE_GROUP_ADULT = 2;
constexpr uint8_t AGE_GROUP_ELDERLY = 3;

constexpr uint8_t AVATAR_HINT_UNKNOWN = 0;
constexpr uint8_t AVATAR_HINT_YOUNG = 1;
constexpr uint8_t AVATAR_HINT_ADULT = 2;
constexpr uint8_t AVATAR_HINT_GRANDPA = 3;
constexpr uint8_t AVATAR_HINT_GRANDMA = 4;
// Sex-specific variants. The trajectory file stores the hint as one byte, so
// these values extend the range without touching the format.
constexpr uint8_t AVATAR_HINT_YOUNG_MALE = 5;
constexpr uint8_t AVATAR_HINT_YOUNG_FEMALE = 6;
constexpr uint8_t AVATAR_HINT_ADULT_MALE = 7;
constexpr uint8_t AVATAR_HINT_ADULT_FEMALE = 8;

struct AgentConfig {
    Point position{};
    double radius{0.2};
    double timeGap{1.0};
    double desiredSpeed{1.2};
    std::string ageGroup{};
    std::string avatarHint{};
    // 1-based index into <exits>, 0 == not assigned. An assigned agent walks
    // straight to that exit and is never offered the choice, which is what
    // RiMEA Test 10 (Zuweisung von Rettungswegen) asks for.
    size_t escapeRoute{0};
    // Seconds the agent stands still before it starts walking (v0 is held at 0).
    // 0.0 == the previous behaviour: everybody moves from t = 0.
    // Must stay last: the designated initializer below requires declaration order.
    double preMovementTime{0.0};
};

struct AgentSpawnProfile {
    double radius{0.2};
    double timeGap{1.0};
    double desiredSpeed{1.2};
    std::string ageGroup{};
    std::string avatarHint{};
    double weight{1.0};
    // 1-based index into <exits>, 0 == not assigned. See AgentConfig.
    size_t escapeRoute{0};
    // Handed to every agent generated from this profile.
    double preMovementTime{0.0};
};

enum class DistributionMode {
    ByNumber,
    ByDensity,
    InCirclesByNumber,
    InCirclesByDensity,
    InRectanglesByNumber,
    UntilFilled,
    ByPercentage
};

enum class ExitTransitionMode {
    Fixed,
    RoundRobin,
    LeastTargeted,
    Adaptive
};

struct CircleSegmentConfig {
    double minRadius{0.0};
    double maxRadius{0.0};
    std::optional<uint64_t> numberOfAgents{};
    std::optional<double> density{};
};

/// Walking speed over age, as RiMEA 4.1.1 Abb. 3 / DIN 18009-2 Bild F.2 show it
/// after Weidmann. Linearly interpolated between knots, clamped outside.
struct SpeedOverAgeKnot {
    double age{};
    double speed{};
};

/// The population a <distribution> draws from when speeds are to follow age
/// rather than a handful of fixed profiles.
///
/// Both standards define the same default when nothing is known about the
/// occupants (RiMEA 3.2.4, DIN Bild F.1): half men, half women, age normally
/// distributed with mean 50 and standard deviation 20, truncated to 10..85, and
/// the speed read off the age curve.
///
/// The knots are deliberately part of the scenario rather than compiled in.
/// Abb. 3 is a diagram, not a table, so any numbers taken from it carry reading
/// error; a Nachweis has to be able to state which values were used and to
/// replace them with the primary source.
struct PopulationConfig {
    double ageMean{50.0};
    double ageSigma{20.0};
    double ageMin{10.0};
    double ageMax{85.0};
    double femaleShare{0.5};
    /// Men are on average this much faster than women. Both standards state
    /// 10.9 % (RiMEA 3.2.2.2, DIN Bild F.2 note 2).
    double maleSpeedBonus{0.109};
    /// Spread around the age curve. DIN Bild F.2 note 1 quotes Weidmann with
    /// sigma = 0.26 m/s, i.e. 19.3 % of the mean.
    double speedSigmaPercent{19.3};
    std::vector<SpeedOverAgeKnot> speedOverAge{};
};

/// One <zone> of an in_rectangles_by_number distribution: a named room gets its
/// own number of persons, placed at run time rather than at export time, so a
/// repeated run with a different seed gives a different arrangement.
struct RectangleZoneConfig {
    double minX{0.0};
    double minY{0.0};
    double maxX{0.0};
    double maxY{0.0};
    uint64_t numberOfAgents{0};
};

struct AgentDistributionConfig {
    DistributionMode mode{DistributionMode::ByNumber};
    double distanceToAgents{0.4};
    double distanceToPolygon{0.0};
    std::optional<uint64_t> numberOfAgents{};
    std::optional<double> density{};
    std::optional<double> percent{};
    std::optional<Point> centerPoint{};
    std::vector<CircleSegmentConfig> circleSegments{};
    std::vector<RectangleZoneConfig> rectangleZones{};
    std::optional<uint64_t> seed{};
    uint64_t maxIterations{10000};
    uint32_t k{30};
    AgentSpawnProfile defaultProfile{};
    std::vector<AgentSpawnProfile> profiles{};
    /// Set by a <population> child. Takes precedence over <profile> children for
    /// speed and age group; radius, time gap, pre-movement and escape route
    /// still come from the default profile.
    std::optional<PopulationConfig> population{};
};

struct ScenarioConfig {
    double dt{0.01};
    uint64_t maxIterations{10000};
    std::vector<Point> walkable{};
    std::vector<std::vector<Point>> obstacles{};
    std::optional<std::vector<Point>> exitPolygon{};
    struct DecisionConfig {
        Point position{};
        double distance{0.8};
    };
    struct MultiExitConfig {
        std::vector<std::vector<Point>> polygons{};
        ExitTransitionMode transitionMode{ExitTransitionMode::Adaptive};
        size_t fixedExitIndex{0};
        std::vector<uint64_t> roundRobinWeights{};
        double expectedTimeWeight{1.0};
        double densityWeight{1.0};
        double queueWeight{0.0};
        double switchPenalty{0.0};
        uint64_t decisionInterval{1};
        double reconsiderationThreshold{0.0};
    };
    std::optional<DecisionConfig> decision{};
    std::optional<MultiExitConfig> multiExit{};
    struct StairConfig {
        Point position{};
        double distance{0.6};
        double length{5.0};
        bool ascending{true};
        // DIN 18009-2 F.4.2 is explicit that stair speed is "nicht um einen
        // bestimmten Faktor reduziert": Tab. F.2 lists absolute speeds per age
        // group and direction. So an absolute speed can be given, and it wins
        // over the factor. 0 means "not set" and keeps the factor, which is the
        // behaviour of every scenario written before this existed. The standard
        // also allows a simplified approach - half the plane speed in both
        // directions - which is what the factors express.
        double upSpeedFactor{0.6};
        double downSpeedFactor{0.6};
        double upSpeed{0.0};
        double downSpeed{0.0};
        double waitingTime{0.0};
    };
    struct RampConfig {
        Point position{};
        double distance{0.6};
        double length{5.0};
        bool ascending{true};
        double upSpeedFactor{0.6};
        double downSpeedFactor{1.0};
        double waitingTime{0.0};
    };
    std::optional<StairConfig> stair{};
    std::optional<RampConfig> ramp{};
    std::vector<AgentConfig> agents{};
    std::optional<AgentDistributionConfig> distribution{};
    double strengthNeighborRepulsion{8.0};
    double rangeNeighborRepulsion{0.1};
    double strengthGeometryRepulsion{5.0};
    double rangeGeometryRepulsion{0.02};

    struct FdsHazardConfig {
        std::optional<fs::path> smvFile{};
        double eyeHeight{1.6};
        std::optional<double> sliceZ{};
        double zTolerance{0.25};
        double offsetX{0.0};
        double offsetY{0.0};
        double updateInterval{0.5};
        double awarenessBelow{15.0};
        double severeBelow{5.0};
        double minimumSpeedFactor{0.25};
        double visibilityFactor{3.0};
        double maximumVisibility{100.0};
        // Smoke as a start trigger for people who are still waiting: once the
        // visibility at their own position drops to this, they notice the fire
        // and leave after smokeReaction seconds, even if their pre_movement time
        // has not come yet. 0 disables it, which is the previous behaviour.
        double smokeAlertsBelow{0.0};
        double smokeReaction{0.0};
        // Height at which a waiting person LOOKS for smoke. People do not stare
        // at a plane in front of their nose: they see the layer building up
        // overhead long before it reaches them. Sampled on its own grid plane,
        // separate from eyeHeight, which stays what slows them down once walking.
        double smokeWatchHeight{2.5};
        // How far a person can look. Smoke is noticed anywhere within this
        // radius, not only where the person stands - which is how one actually
        // spots a fire. 0 means "own position only".
        // Walls are NOT taken into account: somebody may notice smoke that is
        // in fact behind a wall.
        double smokeSightRange{0.0};
        // Aperture of the view cone in degrees, centred on where the person
        // faces. 360 = looks around, which is the default and the sane choice
        // for somebody who is waiting: the orientation of a standing agent
        // points at its exit (CollisionFreeSpeedModel.cpp: direction is derived
        // from destination - pos every step, whatever the speed), so a narrow
        // cone would mean staring at the door for minutes and never noticing a
        // fire behind one's back. Narrow it only if that is what you intend.
        double smokeSightAngle{360.0};
        // Fraction of light the smoke has to swallow along a sight line before
        // the person calls it smoke. 0.4 means "the view is 40 % dimmer than it
        // should be" - roughly where a room visibly turns grey. This is what the
        // eye reacts to; smokeAlertsBelow covers the other case, smoke that has
        // arrived at the person's own position.
        double smokeViewDimmed{0.4};
    };
    std::optional<FdsHazardConfig> fdsHazard{};

    // Passed through from the exporting application, written to the .jsp so the
    // trajectory can be placed without the scenario file.
    struct SourceModelInfo {
        double storeyElevation{0.0};
        double centeringX{0.0};
        double centeringY{0.0};
        double centeringZ{0.0};
    };
    std::optional<SourceModelInfo> sourceModel{};
};

struct CliArgs {
    std::string scenarioPath{};
    std::optional<uint64_t> maxIterationsOverride{};
    std::optional<std::string> outputPath{};
    std::optional<std::string> fdsSmvPath{};
    uint32_t everyNthFrame{1};
    int compressionLevel{6};
};

struct FdsJspMetadata {
    std::string smvPathUtf8{};
    bool pathIsRelative{false};
    double sampleZ{};
    double zTolerance{};
    double offsetX{};
    double offsetY{};
    double timeOffsetSeconds{};
    double updateInterval{};
    double awarenessBelow{};
    double severeBelow{};
    double minimumSpeedFactor{};
    double visibilityFactor{};
    double maximumVisibility{};
};

struct FrameIndexEntry {
    uint64_t iteration{};
    double timeSeconds{};
    uint32_t agentCount{};
    uint64_t dataOffset{};
    uint64_t compressedSize{};
    uint64_t uncompressedSize{};
};

struct AgentProfileEntry {
    uint64_t agentId{};
    uint8_t ageGroupCode{AGE_GROUP_UNKNOWN};
    uint8_t avatarHintCode{AVATAR_HINT_UNKNOWN};
    float desiredSpeed{0.0f};
    float timeGap{0.0f};
    float radius{0.0f};
};

template <typename T>
T RequiredValue(const pt::ptree& node, const std::string& path, const std::string& name)
{
    const auto value = node.get_optional<T>(path);
    if(!value) {
        throw std::runtime_error("Missing required value: " + name);
    }
    return *value;
}

uint64_t ParseUint64(const std::string& value, const std::string& name)
{
    if(value.empty() ||
       !std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return std::isdigit(c) != 0;
       })) {
        throw std::runtime_error("Invalid integer for " + name + ": '" + value + "'");
    }

    std::size_t pos = 0;
    unsigned long long parsed = 0;
    try {
        parsed = std::stoull(value, &pos, 10);
    } catch(const std::exception&) {
        throw std::runtime_error("Invalid integer for " + name + ": '" + value + "'");
    }
    if(pos != value.size()) {
        throw std::runtime_error("Invalid integer for " + name + ": '" + value + "'");
    }
    if(parsed == 0) {
        throw std::runtime_error(name + " must be > 0");
    }
    return static_cast<uint64_t>(parsed);
}

void PrintUsage(const char* program)
{
    fmt::print(
        "Usage: {} <scenario.xml> [options]\n"
        "\n"
        "Options:\n"
        "  --out-jsp <file.jsp>       Output trajectory file path\n"
        "                             (default: <scenario>.jsp)\n"
        "  --fds-smv <file.smv>       Override the FDS Smokeview file configured in XML\n"
        "  --max-iterations <N>       Stop simulation after at most N iterations\n"
        "  --every-nth-frame <N>      Write every Nth simulation frame to .jsp\n"
        "  --compression-level <1-12> libdeflate level (default: 6)\n"
        "  -h, --help                 Show this help\n"
        "\n"
        "XML schema (minimal):\n"
        "  <scenario dt=\"0.01\" max_iterations=\"10000\">\n"
        "    <geometry>\n"
        "      <walkable><vertex x=\"0\" y=\"0\"/>...</walkable>\n"
        "      <obstacle>...</obstacle>  <!-- optional, repeatable -->\n"
        "    </geometry>\n"
        "    <fds_hazard file=\"case.smv\" eye_height=\"1.60\" z_tolerance=\"0.25\"\n"
        "                offset_x=\"0.0\" offset_y=\"0.0\" update_interval=\"0.5\"\n"
        "                smoke_alerts_below=\"0.0\" smoke_reaction=\"0.0\">\n"
        "      <!-- smoke_alerts_below: visibility at which a person who is still\n"
        "           waiting notices the fire and leaves smoke_reaction seconds\n"
        "           later, whether or not its pre_movement time has come.\n"
        "           0 = off; the person then only starts on its own clock. -->\n"
        "      <visibility awareness_below=\"15.0\" severe_below=\"5.0\"\n"
        "                  minimum_speed_factor=\"0.25\" visibility_factor=\"3.0\"\n"
        "                  maximum_visibility=\"100.0\"/>\n"
        "    </fds_hazard> <!-- optional; --fds-smv overrides only file -->\n"
        "    <exit><vertex x=\"...\" y=\"...\"/>...</exit>  <!-- single exit mode -->\n"
        "    <!-- OR multi-exit decision mode: -->\n"
        "    <decision x=\"...\" y=\"...\" distance=\"0.8\"/>\n"
        "    <exits mode=\"adaptive|least_targeted|round_robin|fixed\" fixed_index=\"0\"\n"
        "           expected_time_weight=\"1.0\" density_weight=\"1.0\" queue_weight=\"0.0\"\n"
        "           switch_penalty=\"0.0\" decision_interval=\"1\"\n"
        "           reconsideration_threshold=\"0.0\">\n"
        "      <exit weight=\"1\"><vertex x=\"...\" y=\"...\"/>...</exit>\n"
        "      <exit weight=\"2\"><vertex x=\"...\" y=\"...\"/>...</exit>\n"
        "    </exits>\n"
        "    <stair x=\"...\" y=\"...\" length=\"8.0\" distance=\"0.6\"\n"
        "           ascending=\"true\" speed_factor=\"0.6\" waiting_time=\"0.0\"\n"
        "           up_speed_factor=\"0.5\" down_speed_factor=\"0.65\"\n"
        "           up_speed=\"0.50\" down_speed=\"0.65\"/>\n"
        "           <!-- optional. speed_factor sets both directions; up_/down_ override\n"
        "                it. up_speed/down_speed are absolute m/s and win over the\n"
        "                factors: per DIN 18009-2 F.4.2 the speed on stairs is not a\n"
        "                fixed fraction of the speed on the level (Tab. F.2, after\n"
        "                Fruin: 30-50 years, inner stair, 0.50 up / 0.65 down). -->\n"
        "    <ramp x=\"...\" y=\"...\" length=\"10.0\" distance=\"0.6\" ascending=\"true\"\n"
        "          up_speed_factor=\"0.6\" down_speed_factor=\"1.0\" waiting_time=\"0.0\"/>\n"
        "          <!-- optional, use either <stair> or <ramp> -->\n"
        "    <agents>\n"
        "      <agent x=\"1\" y=\"1\" radius=\"0.2\" time_gap=\"1.0\" desired_speed=\"1.2\"\n"
        "             pre_movement=\"0.0\"  <!-- seconds standing still before walking -->\n"
        "             age_group=\"young|adult|elderly\" avatar_hint=\"young|adult|grandpa|grandma\"\n"
        "             escape_route=\"1\"  <!-- 1-based index into <exits>; omit to let\n"
        "                                   the person choose per <exits mode=...> -->/>\n"
        "      <!-- Or generate agents automatically -->\n"
        "      <distribution mode=\"by_number\" number_of_agents=\"200\"\n"
        "                    distance_to_agents=\"0.45\" distance_to_polygon=\"0.20\" seed=\"42\" />\n"
        "      <distribution mode=\"in_circles_by_density\" center_x=\"40\" center_y=\"15\"\n"
        "                    distance_to_agents=\"0.45\" distance_to_polygon=\"0.20\" seed=\"42\">\n"
        "        <segment min_radius=\"0\" max_radius=\"8\" density=\"2.0\"/>\n"
        "        <segment min_radius=\"8\" max_radius=\"16\" density=\"1.0\"/>\n"
        "      </distribution>\n"
        "      <!-- One headcount per room; positions are drawn per seed, so a\n"
        "           repeated run gives another arrangement -->\n"
        "      <distribution mode=\"in_rectangles_by_number\"\n"
        "                    distance_to_agents=\"0.45\" distance_to_polygon=\"0.20\" seed=\"42\">\n"
        "        <zone min_x=\"1\" min_y=\"1\" max_x=\"8\" max_y=\"8\" number_of_agents=\"40\"/>\n"
        "        <zone min_x=\"11\" min_y=\"1\" max_x=\"18\" max_y=\"8\" number_of_agents=\"25\"/>\n"
        "      </distribution>\n"
        "      <!-- Speeds following age instead of a few fixed profiles. The knots\n"
        "           are NOT built in: RiMEA Abb. 3 / DIN 18009-2 Bild F.2 is a\n"
        "           diagram, so the values used must be stated here and be citable.\n"
        "           Defaults below are the standard population of RiMEA 3.2.4 /\n"
        "           DIN Bild F.1. speed_sigma_percent is the spread AROUND the curve\n"
        "           at a given age (DIN Bild F.2 note 1: sigma = 0.26 m/s = 19.3 %);\n"
        "           the spread over the whole population comes out larger because\n"
        "           age and sex vary on top of it. -->\n"
        "      <distribution mode=\"by_number\" number_of_agents=\"500\" seed=\"42\">\n"
        "        <population age_mean=\"50\" age_sigma=\"20\" age_min=\"10\" age_max=\"85\"\n"
        "                    female_share=\"0.5\" male_speed_bonus=\"0.109\"\n"
        "                    speed_sigma_percent=\"19.3\">\n"
        "          <speed age=\"20\" v=\"1.61\"/>\n"
        "          <speed age=\"50\" v=\"1.41\"/>\n"
        "          <speed age=\"85\" v=\"0.68\"/>\n"
        "        </population>\n"
        "      </distribution>\n"
        "      <profile desired_speed=\"1.55\" radius=\"0.19\" time_gap=\"0.75\"\n"
        "               age_group=\"young\" avatar_hint=\"young\" weight=\"1.0\"\n"
        "               escape_route=\"1\"/>  <!-- applies to every agent drawn from it -->\n"
        "    </agents>\n"
        "    <model type=\"collision_free_speed\" .../>\n"
        "  </scenario>\n",
        program);
}

Point ParsePoint(const pt::ptree& node, const std::string& context)
{
    const auto x = RequiredValue<double>(node, "<xmlattr>.x", context + ".x");
    const auto y = RequiredValue<double>(node, "<xmlattr>.y", context + ".y");
    return {x, y};
}

std::vector<Point> ParsePolygon(const pt::ptree& node, const std::string& context)
{
    std::vector<Point> points{};
    for(const auto& [tag, child] : node) {
        if(tag == "vertex") {
            points.push_back(ParsePoint(child, context + ".vertex"));
        }
    }
    if(points.size() < 3) {
        throw std::runtime_error(
            context + " must contain at least 3 <vertex x=\"...\" y=\"...\"/> elements");
    }
    return points;
}

std::string ToLowerAscii(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool ParseBool(const std::string& value, const std::string& name)
{
    const auto v = ToLowerAscii(value);
    if(v == "true" || v == "1" || v == "yes" || v == "on") {
        return true;
    }
    if(v == "false" || v == "0" || v == "no" || v == "off") {
        return false;
    }
    throw std::runtime_error(
        "Invalid boolean for " + name + ": '" + value + "' (expected true/false)");
}

ExitTransitionMode ParseExitTransitionMode(const std::string& value)
{
    const auto mode = ToLowerAscii(value);
    if(mode == "fixed") {
        return ExitTransitionMode::Fixed;
    }
    if(mode == "round_robin") {
        return ExitTransitionMode::RoundRobin;
    }
    if(mode == "least_targeted") {
        return ExitTransitionMode::LeastTargeted;
    }
    if(mode == "adaptive") {
        return ExitTransitionMode::Adaptive;
    }
    throw std::runtime_error(
        "Unknown exits transition mode '" + value +
        "'. Supported: fixed, round_robin, least_targeted, adaptive");
}

uint8_t AgeGroupCodeFromString(const std::string& value)
{
    if(value == "young" || value == "youth" || value == "junior") {
        return AGE_GROUP_YOUNG;
    }
    if(value == "adult") {
        return AGE_GROUP_ADULT;
    }
    if(value == "elderly" || value == "old" || value == "senior") {
        return AGE_GROUP_ELDERLY;
    }
    return AGE_GROUP_UNKNOWN;
}

uint8_t AvatarHintCodeFromString(const std::string& value)
{
    if(value == "young") {
        return AVATAR_HINT_YOUNG;
    }
    if(value == "adult") {
        return AVATAR_HINT_ADULT;
    }
    if(value == "grandpa" || value == "elderly_male") {
        return AVATAR_HINT_GRANDPA;
    }
    if(value == "grandma" || value == "elderly_female") {
        return AVATAR_HINT_GRANDMA;
    }
    if(value == "young_male") {
        return AVATAR_HINT_YOUNG_MALE;
    }
    if(value == "young_female") {
        return AVATAR_HINT_YOUNG_FEMALE;
    }
    if(value == "adult_male") {
        return AVATAR_HINT_ADULT_MALE;
    }
    if(value == "adult_female") {
        return AVATAR_HINT_ADULT_FEMALE;
    }
    return AVATAR_HINT_UNKNOWN;
}

uint8_t ClassifyAgeGroupCode(double desiredSpeed)
{
    if(desiredSpeed >= 1.50) {
        return AGE_GROUP_YOUNG;
    }
    if(desiredSpeed <= 1.05) {
        return AGE_GROUP_ELDERLY;
    }
    return AGE_GROUP_ADULT;
}

uint8_t DeriveAvatarHintCode(uint8_t ageGroupCode, uint64_t agentId)
{
    if(ageGroupCode == AGE_GROUP_YOUNG) {
        return AVATAR_HINT_YOUNG;
    }
    if(ageGroupCode == AGE_GROUP_ADULT) {
        return AVATAR_HINT_ADULT;
    }
    if(ageGroupCode == AGE_GROUP_ELDERLY) {
        return (agentId % 2 == 0) ? AVATAR_HINT_GRANDMA : AVATAR_HINT_GRANDPA;
    }
    return AVATAR_HINT_UNKNOWN;
}

DistributionMode ParseDistributionMode(const std::string& value)
{
    const auto mode = ToLowerAscii(value);
    if(mode == "by_number") {
        return DistributionMode::ByNumber;
    }
    if(mode == "by_density") {
        return DistributionMode::ByDensity;
    }
    if(mode == "in_circles_by_number") {
        return DistributionMode::InCirclesByNumber;
    }
    if(mode == "in_circles_by_density") {
        return DistributionMode::InCirclesByDensity;
    }
    if(mode == "in_rectangles_by_number") {
        return DistributionMode::InRectanglesByNumber;
    }
    if(mode == "until_filled") {
        return DistributionMode::UntilFilled;
    }
    if(mode == "by_percentage") {
        return DistributionMode::ByPercentage;
    }
    throw std::runtime_error(
        "Unknown distribution mode '" + value +
        "'. Supported: by_number, by_density, in_circles_by_number, "
        "in_circles_by_density, in_rectangles_by_number, until_filled, by_percentage");
}

std::optional<uint64_t> ParseOptionalSeed(
    const pt::ptree& node,
    const std::string& path,
    const std::string& name)
{
    const auto seedValue = node.get_optional<std::string>(path);
    if(!seedValue) {
        return std::nullopt;
    }
    std::size_t pos = 0;
    unsigned long long parsed = 0;
    try {
        parsed = std::stoull(*seedValue, &pos, 10);
    } catch(const std::exception&) {
        throw std::runtime_error("Invalid integer for " + name + ": '" + *seedValue + "'");
    }
    if(pos != seedValue->size()) {
        throw std::runtime_error("Invalid integer for " + name + ": '" + *seedValue + "'");
    }
    return static_cast<uint64_t>(parsed);
}

AgentSpawnProfile ParseSpawnProfile(
    const pt::ptree& node,
    const std::string& context,
    const AgentSpawnProfile& defaults)
{
    AgentSpawnProfile profile = defaults;
    profile.radius = node.get<double>("<xmlattr>.radius", profile.radius);
    profile.timeGap = node.get<double>("<xmlattr>.time_gap", profile.timeGap);
    profile.desiredSpeed = node.get<double>("<xmlattr>.desired_speed", profile.desiredSpeed);
    profile.weight = node.get<double>("<xmlattr>.weight", profile.weight);
    profile.preMovementTime =
        node.get<double>("<xmlattr>.pre_movement", profile.preMovementTime);
    profile.ageGroup =
        ToLowerAscii(node.get<std::string>("<xmlattr>.age_group", profile.ageGroup));
    profile.avatarHint =
        ToLowerAscii(node.get<std::string>("<xmlattr>.avatar_hint", profile.avatarHint));
    {
        // Signed on purpose: a negative value has to be rejected rather than
        // wrap around into a huge index.
        const auto route = node.get<int64_t>("<xmlattr>.escape_route", 0);
        if(route < 0) {
            throw std::runtime_error(context + ".escape_route must be >= 1");
        }
        profile.escapeRoute = static_cast<size_t>(route);
    }

    if(profile.radius <= 0.0) {
        throw std::runtime_error(context + ".radius must be > 0");
    }
    if(profile.timeGap <= 0.0) {
        throw std::runtime_error(context + ".time_gap must be > 0");
    }
    if(profile.desiredSpeed <= 0.0) {
        throw std::runtime_error(context + ".desired_speed must be > 0");
    }
    if(profile.weight <= 0.0) {
        throw std::runtime_error(context + ".weight must be > 0");
    }
    // ">= 0", not "> 0": 0.0 is the default and means "starts immediately".
    if(!std::isfinite(profile.preMovementTime) || profile.preMovementTime < 0.0) {
        throw std::runtime_error(context + ".pre_movement must be finite and >= 0");
    }
    return profile;
}

void ValidateCircleSegments(const std::vector<CircleSegmentConfig>& segments, const std::string& context)
{
    for(size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if(seg.minRadius < 0.0 || seg.maxRadius < 0.0) {
            throw std::runtime_error(
                context + ".segment[" + std::to_string(i) + "] radii must be >= 0");
        }
        if(seg.minRadius >= seg.maxRadius) {
            throw std::runtime_error(
                context + ".segment[" + std::to_string(i) + "] requires min_radius < max_radius");
        }
        for(size_t j = 0; j < i; ++j) {
            const auto& other = segments[j];
            const bool disjoint =
                seg.maxRadius <= other.minRadius || other.maxRadius <= seg.minRadius;
            if(!disjoint) {
                throw std::runtime_error(
                    context + ".segment radii overlap: [" + std::to_string(seg.minRadius) +
                    ", " + std::to_string(seg.maxRadius) + "] vs [" +
                    std::to_string(other.minRadius) + ", " + std::to_string(other.maxRadius) + "]");
            }
        }
    }
}

AgentDistributionConfig ParseDistributionConfig(const pt::ptree& node, const std::string& context)
{
    AgentDistributionConfig config{};
    config.mode = ParseDistributionMode(
        RequiredValue<std::string>(node, "<xmlattr>.mode", context + ".mode"));
    config.distanceToAgents =
        node.get<double>("<xmlattr>.distance_to_agents", config.distanceToAgents);
    config.distanceToPolygon =
        node.get<double>("<xmlattr>.distance_to_polygon", config.distanceToPolygon);
    config.maxIterations = node.get<uint64_t>("<xmlattr>.max_iterations", config.maxIterations);
    config.k = node.get<uint32_t>("<xmlattr>.k", config.k);
    config.seed = ParseOptionalSeed(node, "<xmlattr>.seed", context + ".seed");

    if(config.distanceToAgents <= 0.0) {
        throw std::runtime_error(context + ".distance_to_agents must be > 0");
    }
    if(config.distanceToPolygon < 0.0) {
        throw std::runtime_error(context + ".distance_to_polygon must be >= 0");
    }
    if(config.maxIterations == 0) {
        throw std::runtime_error(context + ".max_iterations must be > 0");
    }
    if(config.k == 0) {
        throw std::runtime_error(context + ".k must be > 0");
    }

    config.defaultProfile = ParseSpawnProfile(node, context, config.defaultProfile);

    for(const auto& [tag, child] : node) {
        if(tag == "profile") {
            config.profiles.push_back(
                ParseSpawnProfile(child, context + ".profile", config.defaultProfile));
            continue;
        }
        if(tag == "population") {
            PopulationConfig population{};
            population.ageMean = child.get<double>("<xmlattr>.age_mean", population.ageMean);
            population.ageSigma = child.get<double>("<xmlattr>.age_sigma", population.ageSigma);
            population.ageMin = child.get<double>("<xmlattr>.age_min", population.ageMin);
            population.ageMax = child.get<double>("<xmlattr>.age_max", population.ageMax);
            population.femaleShare =
                child.get<double>("<xmlattr>.female_share", population.femaleShare);
            population.maleSpeedBonus =
                child.get<double>("<xmlattr>.male_speed_bonus", population.maleSpeedBonus);
            population.speedSigmaPercent =
                child.get<double>("<xmlattr>.speed_sigma_percent", population.speedSigmaPercent);
            for(const auto& [speedTag, speedNode] : child) {
                if(speedTag != "speed") {
                    continue;
                }
                SpeedOverAgeKnot knot{};
                knot.age = RequiredValue<double>(
                    speedNode, "<xmlattr>.age", context + ".population.speed.age");
                knot.speed = RequiredValue<double>(
                    speedNode, "<xmlattr>.v", context + ".population.speed.v");
                if(knot.speed <= 0.0) {
                    throw std::runtime_error(context + ".population.speed.v must be > 0");
                }
                population.speedOverAge.push_back(knot);
            }
            if(population.speedOverAge.size() < 2) {
                throw std::runtime_error(
                    context + ".population requires at least two <speed age=\"..\" v=\"..\"/> "
                              "knots. They are not built in on purpose: RiMEA Abb. 3 / DIN "
                              "Bild F.2 is a diagram, so the values used have to be stated in "
                              "the scenario and be citable in the report.");
            }
            std::sort(
                std::begin(population.speedOverAge),
                std::end(population.speedOverAge),
                [](const auto& a, const auto& b) { return a.age < b.age; });
            if(population.ageMax <= population.ageMin) {
                throw std::runtime_error(context + ".population.age_max must be > age_min");
            }
            if(population.ageSigma < 0.0) {
                throw std::runtime_error(context + ".population.age_sigma must be >= 0");
            }
            if(population.femaleShare < 0.0 || population.femaleShare > 1.0) {
                throw std::runtime_error(context + ".population.female_share must be in [0, 1]");
            }
            if(population.speedSigmaPercent < 0.0) {
                throw std::runtime_error(
                    context + ".population.speed_sigma_percent must be >= 0");
            }
            config.population = population;
            continue;
        }
        if(tag == "zone") {
            RectangleZoneConfig zone{};
            zone.minX = RequiredValue<double>(child, "<xmlattr>.min_x", context + ".zone.min_x");
            zone.minY = RequiredValue<double>(child, "<xmlattr>.min_y", context + ".zone.min_y");
            zone.maxX = RequiredValue<double>(child, "<xmlattr>.max_x", context + ".zone.max_x");
            zone.maxY = RequiredValue<double>(child, "<xmlattr>.max_y", context + ".zone.max_y");
            zone.numberOfAgents = RequiredValue<uint64_t>(
                child, "<xmlattr>.number_of_agents", context + ".zone.number_of_agents");
            if(zone.maxX <= zone.minX || zone.maxY <= zone.minY) {
                throw std::runtime_error(context + ".zone must have max_x > min_x and max_y > min_y");
            }
            if(zone.numberOfAgents == 0) {
                throw std::runtime_error(context + ".zone.number_of_agents must be > 0");
            }
            config.rectangleZones.push_back(zone);
            continue;
        }
        if(tag == "segment" || tag == "circle") {
            CircleSegmentConfig segment{};
            segment.minRadius =
                RequiredValue<double>(child, "<xmlattr>.min_radius", context + ".segment.min_radius");
            segment.maxRadius =
                RequiredValue<double>(child, "<xmlattr>.max_radius", context + ".segment.max_radius");
            if(config.mode == DistributionMode::InCirclesByNumber) {
                segment.numberOfAgents = RequiredValue<uint64_t>(
                    child,
                    "<xmlattr>.number_of_agents",
                    context + ".segment.number_of_agents");
                if(*segment.numberOfAgents == 0) {
                    throw std::runtime_error(
                        context + ".segment.number_of_agents must be > 0");
                }
            } else if(config.mode == DistributionMode::InCirclesByDensity) {
                segment.density = RequiredValue<double>(
                    child,
                    "<xmlattr>.density",
                    context + ".segment.density");
                if(*segment.density <= 0.0) {
                    throw std::runtime_error(context + ".segment.density must be > 0");
                }
            } else {
                throw std::runtime_error(
                    context + " contains circle segments but mode is not in_circles_*");
            }
            config.circleSegments.push_back(segment);
        }
    }

    switch(config.mode) {
        case DistributionMode::ByNumber: {
            config.numberOfAgents = RequiredValue<uint64_t>(
                node,
                "<xmlattr>.number_of_agents",
                context + ".number_of_agents");
            if(*config.numberOfAgents == 0) {
                throw std::runtime_error(context + ".number_of_agents must be > 0");
            }
            break;
        }
        case DistributionMode::ByDensity: {
            config.density =
                RequiredValue<double>(node, "<xmlattr>.density", context + ".density");
            if(*config.density <= 0.0) {
                throw std::runtime_error(context + ".density must be > 0");
            }
            break;
        }
        case DistributionMode::InCirclesByNumber:
        case DistributionMode::InCirclesByDensity: {
            const double cx =
                RequiredValue<double>(node, "<xmlattr>.center_x", context + ".center_x");
            const double cy =
                RequiredValue<double>(node, "<xmlattr>.center_y", context + ".center_y");
            config.centerPoint = Point{cx, cy};
            if(config.circleSegments.empty()) {
                throw std::runtime_error(context + " requires at least one <segment/>");
            }
            ValidateCircleSegments(config.circleSegments, context);
            break;
        }
        case DistributionMode::InRectanglesByNumber: {
            if(config.rectangleZones.empty()) {
                throw std::runtime_error(context + " requires at least one <zone/>");
            }
            break;
        }
        case DistributionMode::UntilFilled: {
            break;
        }
        case DistributionMode::ByPercentage: {
            config.percent =
                RequiredValue<double>(node, "<xmlattr>.percent", context + ".percent");
            if(*config.percent <= 0.0 || *config.percent > 100.0) {
                throw std::runtime_error(context + ".percent must be in (0, 100]");
            }
            break;
        }
    }

    return config;
}

struct BoundingBox {
    double minX{0.0};
    double minY{0.0};
    double maxX{0.0};
    double maxY{0.0};
};

BoundingBox BoundingBoxFromPoints(const std::vector<Point>& points)
{
    if(points.empty()) {
        throw std::runtime_error("Cannot compute bounding box from empty point list");
    }
    BoundingBox box{};
    box.minX = box.maxX = points.front().x;
    box.minY = box.maxY = points.front().y;
    for(const auto& p : points) {
        box.minX = std::min(box.minX, p.x);
        box.minY = std::min(box.minY, p.y);
        box.maxX = std::max(box.maxX, p.x);
        box.maxY = std::max(box.maxY, p.y);
    }
    return box;
}

BoundingBox IntersectBoxes(const BoundingBox& a, const BoundingBox& b)
{
    return BoundingBox{
        .minX = std::max(a.minX, b.minX),
        .minY = std::max(a.minY, b.minY),
        .maxX = std::min(a.maxX, b.maxX),
        .maxY = std::min(a.maxY, b.maxY),
    };
}

bool IsValidBox(const BoundingBox& box)
{
    return box.maxX > box.minX && box.maxY > box.minY;
}

double BoxArea(const BoundingBox& box)
{
    if(!IsValidBox(box)) {
        return 0.0;
    }
    return (box.maxX - box.minX) * (box.maxY - box.minY);
}

class SamplingGrid
{
public:
    SamplingGrid(BoundingBox box, double distanceToAgents)
        : _box(box)
        , _distance(distanceToAgents)
        , _distanceSq(distanceToAgents * distanceToAgents)
        , _cellSize(distanceToAgents / std::sqrt(2.0))
    {
        if(_cellSize <= 0.0) {
            throw std::runtime_error("distance_to_agents must be > 0");
        }
    }

    void Insert(Point p)
    {
        const auto [ix, iy] = CellCoords(p);
        _cells[CellKey(ix, iy)].push_back(p);
    }

    bool HasNeighborInDistance(Point p) const
    {
        const auto [ix, iy] = CellCoords(p);
        for(int dx = -2; dx <= 2; ++dx) {
            for(int dy = -2; dy <= 2; ++dy) {
                const auto key = CellKey(ix + dx, iy + dy);
                const auto it = _cells.find(key);
                if(it == _cells.end()) {
                    continue;
                }
                for(const auto& other : it->second) {
                    const double ddx = other.x - p.x;
                    const double ddy = other.y - p.y;
                    if(ddx * ddx + ddy * ddy < _distanceSq) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

private:
    static uint64_t CellKey(int ix, int iy)
    {
        return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32u) |
               static_cast<uint32_t>(iy);
    }

    std::pair<int, int> CellCoords(Point p) const
    {
        return {
            static_cast<int>(std::floor((p.x - _box.minX) / _cellSize)),
            static_cast<int>(std::floor((p.y - _box.minY) / _cellSize)),
        };
    }

private:
    BoundingBox _box{};
    double _distance{0.0};
    double _distanceSq{0.0};
    double _cellSize{0.0};
    std::unordered_map<uint64_t, std::vector<Point>> _cells{};
};

bool IsInsideRing(Point p, Point center, double innerRadius, double outerRadius)
{
    const double dx = p.x - center.x;
    const double dy = p.y - center.y;
    const double distSq = dx * dx + dy * dy;
    return distSq >= innerRadius * innerRadius && distSq <= outerRadius * outerRadius;
}

bool MeetsPlacementConstraints(
    const CollisionGeometry& geometry,
    Point candidate,
    double distanceToPolygon,
    const SamplingGrid& grid)
{
    if(!geometry.InsideGeometry(candidate)) {
        return false;
    }

    if(distanceToPolygon > 0.0) {
        const auto nearbySegments = geometry.LineSegmentsInDistanceTo(distanceToPolygon, candidate);
        if(nearbySegments.begin() != nearbySegments.end()) {
            return false;
        }
    }

    return !grid.HasNeighborInDistance(candidate);
}

Point RandomPointInBox(const BoundingBox& box, std::mt19937_64& rng)
{
    std::uniform_real_distribution<double> distX(box.minX, box.maxX);
    std::uniform_real_distribution<double> distY(box.minY, box.maxY);
    return Point{distX(rng), distY(rng)};
}

Point RandomPointInRing(
    Point center,
    double innerRadius,
    double outerRadius,
    std::mt19937_64& rng)
{
    std::uniform_real_distribution<double> distRho(innerRadius * innerRadius, outerRadius * outerRadius);
    std::uniform_real_distribution<double> distTheta(0.0, 2.0 * std::numbers::pi_v<double>);
    const double rho = std::sqrt(distRho(rng));
    const double theta = distTheta(rng);
    return Point{center.x + rho * std::cos(theta), center.y + rho * std::sin(theta)};
}

double AccessibleAreaSquareMeters(const CollisionGeometry& geometry)
{
    const auto& polyWithHoles = geometry.Polygon();
    double area = std::abs(polyWithHoles.outer_boundary().area());
    for(auto hole = polyWithHoles.holes_begin(); hole != polyWithHoles.holes_end(); ++hole) {
        area -= std::abs(hole->area());
    }
    return std::max(0.0, area);
}

double EstimateRingAreaInGeometry(
    const CollisionGeometry& geometry,
    const BoundingBox& globalBox,
    Point center,
    double innerRadius,
    double outerRadius,
    std::optional<uint64_t> seed,
    size_t ringIndex)
{
    const BoundingBox ringBox{
        .minX = center.x - outerRadius,
        .minY = center.y - outerRadius,
        .maxX = center.x + outerRadius,
        .maxY = center.y + outerRadius,
    };
    const BoundingBox sampleBox = IntersectBoxes(globalBox, ringBox);
    if(!IsValidBox(sampleBox)) {
        return 0.0;
    }

    std::mt19937_64 rng(seed.has_value() ? (*seed + 0x9E3779B97F4A7C15ull + ringIndex * 4099ull)
                                         : (0xA57F123456ull + ringIndex * 97ull));
    constexpr uint64_t kSamples = 12000;
    uint64_t accepted = 0;
    for(uint64_t i = 0; i < kSamples; ++i) {
        const Point p = RandomPointInBox(sampleBox, rng);
        if(!geometry.InsideGeometry(p)) {
            continue;
        }
        if(IsInsideRing(p, center, innerRadius, outerRadius)) {
            ++accepted;
        }
    }
    return BoxArea(sampleBox) * (static_cast<double>(accepted) / static_cast<double>(kSamples));
}

std::vector<Point> DistributeUntilFilled(
    const AgentDistributionConfig& dist,
    const CollisionGeometry& geometry,
    const BoundingBox& box,
    SamplingGrid& grid,
    std::mt19937_64& rng,
    double spacingDistance,
    double boundaryDistance)
{
    std::vector<Point> created{};
    created.reserve(256);

    std::vector<Point> active{};
    active.reserve(256);

    uint64_t attempts = 0;
    while(attempts < dist.maxIterations) {
        const Point first = RandomPointInBox(box, rng);
        if(MeetsPlacementConstraints(geometry, first, boundaryDistance, grid)) {
            grid.Insert(first);
            created.push_back(first);
            active.push_back(first);
            break;
        }
        ++attempts;
    }

    if(active.empty()) {
        throw std::runtime_error(
            "Distribution until_filled failed: first point could not be placed inside geometry");
    }

    while(!active.empty()) {
        std::uniform_int_distribution<size_t> pickActive(0, active.size() - 1);
        const size_t activeIdx = pickActive(rng);
        const Point ref = active[activeIdx];

        bool found = false;
        for(uint32_t i = 0; i < dist.k; ++i) {
            std::uniform_real_distribution<double> distRho(
                spacingDistance * spacingDistance,
                4.0 * spacingDistance * spacingDistance);
            std::uniform_real_distribution<double> distTheta(0.0, 2.0 * std::numbers::pi_v<double>);
            const double rho = std::sqrt(distRho(rng));
            const double theta = distTheta(rng);
            const Point candidate{
                ref.x + rho * std::cos(theta),
                ref.y + rho * std::sin(theta),
            };
            if(MeetsPlacementConstraints(geometry, candidate, boundaryDistance, grid)) {
                grid.Insert(candidate);
                created.push_back(candidate);
                active.push_back(candidate);
                found = true;
                break;
            }
        }

        if(!found) {
            active[activeIdx] = active.back();
            active.pop_back();
        }
    }

    return created;
}

/// Speed at `age`, linearly interpolated between the knots and clamped outside
/// their range. Clamping rather than extrapolating on purpose: beyond the ends
/// of Abb. 3 there is no data, and a straight line would run into negative
/// speeds within a few years.
double SpeedAtAge(const std::vector<SpeedOverAgeKnot>& knots, double age)
{
    if(age <= knots.front().age) {
        return knots.front().speed;
    }
    if(age >= knots.back().age) {
        return knots.back().speed;
    }
    for(size_t i = 0; i + 1 < knots.size(); ++i) {
        const auto& lo = knots[i];
        const auto& hi = knots[i + 1];
        if(age >= lo.age && age <= hi.age) {
            const double span = hi.age - lo.age;
            const double t = span > 0.0 ? (age - lo.age) / span : 0.0;
            return lo.speed + t * (hi.speed - lo.speed);
        }
    }
    return knots.back().speed;
}

/// Draws one person from the population: an age, a sex, and the speed that
/// follows from both plus the spread around the curve.
AgentSpawnProfile DrawFromPopulation(
    const PopulationConfig& population,
    const AgentSpawnProfile& base,
    std::mt19937_64& rng)
{
    AgentSpawnProfile profile = base;

    double age = population.ageMean;
    if(population.ageSigma > 0.0) {
        std::normal_distribution<double> ageDistribution(population.ageMean, population.ageSigma);
        // Redraw rather than clamp: clamping would pile people up on the two
        // limits, which is not what "normally distributed between minimum and
        // maximum" means.
        for(int attempt = 0; attempt < 1000; ++attempt) {
            age = ageDistribution(rng);
            if(age >= population.ageMin && age <= population.ageMax) {
                break;
            }
            age = std::clamp(age, population.ageMin, population.ageMax);
        }
    }

    std::uniform_real_distribution<double> sexDraw(0.0, 1.0);
    const bool female = sexDraw(rng) < population.femaleShare;

    // The curve is the population mean. Splitting it so the mean is preserved:
    // men get half the difference up, women half of it down.
    const double curveSpeed = SpeedAtAge(population.speedOverAge, age);
    const double half = population.maleSpeedBonus * 0.5;
    double speed = curveSpeed * (female ? (1.0 - half) : (1.0 + half));

    if(population.speedSigmaPercent > 0.0) {
        std::normal_distribution<double> spread(
            0.0, speed * population.speedSigmaPercent / 100.0);
        // Two sigma, as the tests in Annex 1 of RiMEA use it, and it keeps the
        // draw away from zero or negative speeds.
        const double limit = 2.0 * speed * population.speedSigmaPercent / 100.0;
        speed += std::clamp(spread(rng), -limit, limit);
    }
    profile.desiredSpeed = std::max(0.05, speed);

    // Only for display and for the age band written into the .jsp; the speed
    // above is what actually moves the person.
    if(age < 30.0) {
        profile.ageGroup = "young";
        profile.avatarHint = female ? "young_female" : "young_male";
    } else if(age < 60.0) {
        profile.ageGroup = "adult";
        profile.avatarHint = female ? "adult_female" : "adult_male";
    } else {
        profile.ageGroup = "elderly";
        profile.avatarHint = female ? "elderly_female" : "elderly_male";
    }
    return profile;
}

const AgentSpawnProfile& PickProfile(
    const std::vector<AgentSpawnProfile>& profiles,
    std::mt19937_64& rng)
{
    if(profiles.size() == 1) {
        return profiles.front();
    }

    double totalWeight = 0.0;
    for(const auto& profile : profiles) {
        totalWeight += profile.weight;
    }
    if(totalWeight <= 0.0) {
        return profiles.front();
    }

    std::uniform_real_distribution<double> pick(0.0, totalWeight);
    const double value = pick(rng);
    double cumulative = 0.0;
    for(const auto& profile : profiles) {
        cumulative += profile.weight;
        if(value <= cumulative) {
            return profile;
        }
    }
    return profiles.back();
}

std::vector<AgentConfig> GenerateDistributedAgents(
    const AgentDistributionConfig& dist,
    const CollisionGeometry& geometry,
    const std::vector<AgentConfig>& existingAgents)
{
    const auto& [walkableBoundary, _obstacles] = geometry.AccessibleArea();
    const BoundingBox globalBox = BoundingBoxFromPoints(walkableBoundary);

    std::vector<AgentSpawnProfile> effectiveProfiles = dist.profiles;
    if(effectiveProfiles.empty()) {
        effectiveProfiles.push_back(dist.defaultProfile);
    }
    double maxProfileRadius = 0.0;
    for(const auto& profile : effectiveProfiles) {
        maxProfileRadius = std::max(maxProfileRadius, profile.radius);
    }

    const double effectiveDistanceToPolygon =
        std::max(dist.distanceToPolygon, maxProfileRadius);
    const double effectiveDistanceToAgents =
        std::max(dist.distanceToAgents, 2.0 * maxProfileRadius);

    SamplingGrid grid(globalBox, effectiveDistanceToAgents);
    for(const auto& agent : existingAgents) {
        grid.Insert(agent.position);
    }

    std::mt19937_64 rng(dist.seed.has_value() ? *dist.seed : std::random_device{}());
    std::vector<Point> generatedPoints{};

    auto acceptPoint = [&](Point p) {
        grid.Insert(p);
        generatedPoints.push_back(p);
    };

    switch(dist.mode) {
        case DistributionMode::ByNumber: {
            uint64_t created = 0;
            uint64_t failedAttempts = 0;
            while(created < *dist.numberOfAgents) {
                if(failedAttempts > dist.maxIterations) {
                    throw std::runtime_error(
                        "Distribution by_number could not place all agents. Placed " +
                        std::to_string(created) + " of " + std::to_string(*dist.numberOfAgents));
                }
                const Point candidate = RandomPointInBox(globalBox, rng);
                if(MeetsPlacementConstraints(
                       geometry,
                       candidate,
                       effectiveDistanceToPolygon,
                       grid)) {
                    acceptPoint(candidate);
                    ++created;
                    failedAttempts = 0;
                } else {
                    ++failedAttempts;
                }
            }
            break;
        }
        case DistributionMode::InRectanglesByNumber: {
            // One zone per room, each with its own headcount. Unlike explicitly
            // exported <agent> elements the arrangement is drawn here, so a
            // repeated run with another seed gives another arrangement - which is
            // what a statistical evaluation over several runs needs.
            for(size_t z = 0; z < dist.rectangleZones.size(); ++z) {
                const auto& zoneConfig = dist.rectangleZones[z];
                const BoundingBox zoneBox{
                    zoneConfig.minX, zoneConfig.minY, zoneConfig.maxX, zoneConfig.maxY};
                uint64_t created = 0;
                uint64_t failedAttempts = 0;
                while(created < zoneConfig.numberOfAgents) {
                    if(failedAttempts > dist.maxIterations) {
                        throw std::runtime_error(
                            "Distribution in_rectangles_by_number could not place all agents "
                            "of zone " + std::to_string(z + 1) + ". Placed " +
                            std::to_string(created) + " of " +
                            std::to_string(zoneConfig.numberOfAgents) +
                            ". The zone may be too small, or overlap a wall or an exit.");
                    }
                    const Point candidate = RandomPointInBox(zoneBox, rng);
                    if(MeetsPlacementConstraints(
                           geometry,
                           candidate,
                           effectiveDistanceToPolygon,
                           grid)) {
                        acceptPoint(candidate);
                        ++created;
                        failedAttempts = 0;
                    } else {
                        ++failedAttempts;
                    }
                }
            }
            break;
        }
        case DistributionMode::ByDensity: {
            const uint64_t numberOfAgents = static_cast<uint64_t>(
                std::llround(*dist.density * AccessibleAreaSquareMeters(geometry)));
            uint64_t created = 0;
            uint64_t failedAttempts = 0;
            while(created < numberOfAgents) {
                if(failedAttempts > dist.maxIterations) {
                    throw std::runtime_error(
                        "Distribution by_density could not place all agents. Placed " +
                        std::to_string(created) + " of " + std::to_string(numberOfAgents));
                }
                const Point candidate = RandomPointInBox(globalBox, rng);
                if(MeetsPlacementConstraints(
                       geometry,
                       candidate,
                       effectiveDistanceToPolygon,
                       grid)) {
                    acceptPoint(candidate);
                    ++created;
                    failedAttempts = 0;
                } else {
                    ++failedAttempts;
                }
            }
            break;
        }
        case DistributionMode::InCirclesByNumber: {
            const Point center = *dist.centerPoint;
            for(size_t segIdx = 0; segIdx < dist.circleSegments.size(); ++segIdx) {
                const auto& segment = dist.circleSegments[segIdx];
                const double ringArea =
                    std::numbers::pi_v<double> *
                    (segment.maxRadius * segment.maxRadius - segment.minRadius * segment.minRadius);
                const BoundingBox ringBox{
                    .minX = center.x - segment.maxRadius,
                    .minY = center.y - segment.maxRadius,
                    .maxX = center.x + segment.maxRadius,
                    .maxY = center.y + segment.maxRadius,
                };
                const BoundingBox sectionBox = IntersectBoxes(globalBox, ringBox);
                const double sectionBoxArea = BoxArea(sectionBox);
                const double placeableArea = EstimateRingAreaInGeometry(
                    geometry,
                    globalBox,
                    center,
                    segment.minRadius,
                    segment.maxRadius,
                    dist.seed,
                    segIdx);
                const uint64_t target = *segment.numberOfAgents;

                if(ringArea < sectionBoxArea) {
                    uint64_t placed = 0;
                    for(uint64_t agentIdx = 0; agentIdx < target; ++agentIdx) {
                        bool found = false;
                        for(uint64_t attempt = 0; attempt < dist.maxIterations; ++attempt) {
                            const Point candidate =
                                RandomPointInRing(center, segment.minRadius, segment.maxRadius, rng);
                            if(MeetsPlacementConstraints(
                                   geometry,
                                   candidate,
                                   effectiveDistanceToPolygon,
                                   grid)) {
                                acceptPoint(candidate);
                                ++placed;
                                found = true;
                                break;
                            }
                        }
                        if(!found) {
                            throw std::runtime_error(
                                "Distribution in_circles_by_number could not place requested agents in "
                                "segment [" +
                                std::to_string(segment.minRadius) + ", " +
                                std::to_string(segment.maxRadius) + "]. Placed " +
                                std::to_string(placed) + " of " + std::to_string(target) +
                                ", density=" +
                                std::to_string(
                                    placeableArea > 0.0 ? placed / placeableArea : 0.0));
                        }
                    }
                } else {
                    uint64_t placed = 0;
                    uint64_t failedAttempts = 0;
                    while(placed < target) {
                        if(failedAttempts > dist.maxIterations) {
                            throw std::runtime_error(
                                "Distribution in_circles_by_number could not place requested agents in "
                                "segment [" +
                                std::to_string(segment.minRadius) + ", " +
                                std::to_string(segment.maxRadius) + "]. Placed " +
                                std::to_string(placed) + " of " + std::to_string(target) +
                                ", density=" +
                                std::to_string(
                                    placeableArea > 0.0 ? placed / placeableArea : 0.0));
                        }
                        const Point candidate = RandomPointInBox(sectionBox, rng);
                        if(IsInsideRing(candidate, center, segment.minRadius, segment.maxRadius) &&
                           MeetsPlacementConstraints(
                               geometry,
                               candidate,
                               effectiveDistanceToPolygon,
                               grid)) {
                            acceptPoint(candidate);
                            ++placed;
                            failedAttempts = 0;
                        } else {
                            ++failedAttempts;
                        }
                    }
                }
            }
            break;
        }
        case DistributionMode::InCirclesByDensity: {
            AgentDistributionConfig byNumberConfig = dist;
            byNumberConfig.mode = DistributionMode::InCirclesByNumber;
            byNumberConfig.circleSegments.clear();
            byNumberConfig.circleSegments.reserve(dist.circleSegments.size());
            for(size_t segIdx = 0; segIdx < dist.circleSegments.size(); ++segIdx) {
                const auto& segment = dist.circleSegments[segIdx];
                const double placeableArea = EstimateRingAreaInGeometry(
                    geometry,
                    globalBox,
                    *dist.centerPoint,
                    segment.minRadius,
                    segment.maxRadius,
                    dist.seed,
                    segIdx);
                const uint64_t count =
                    static_cast<uint64_t>(std::floor((*segment.density) * placeableArea));
                byNumberConfig.circleSegments.push_back(
                    CircleSegmentConfig{
                        .minRadius = segment.minRadius,
                        .maxRadius = segment.maxRadius,
                        .numberOfAgents = count,
                        .density = std::nullopt,
                    });
            }
            return GenerateDistributedAgents(byNumberConfig, geometry, existingAgents);
        }
        case DistributionMode::UntilFilled: {
            generatedPoints = DistributeUntilFilled(
                dist,
                geometry,
                globalBox,
                grid,
                rng,
                effectiveDistanceToAgents,
                effectiveDistanceToPolygon);
            break;
        }
        case DistributionMode::ByPercentage: {
            auto allSamples = DistributeUntilFilled(
                dist,
                geometry,
                globalBox,
                grid,
                rng,
                effectiveDistanceToAgents,
                effectiveDistanceToPolygon);
            const uint64_t needed = static_cast<uint64_t>(
                std::llround(static_cast<double>(allSamples.size()) * (*dist.percent / 100.0)));
            std::mt19937_64 shuffleRng(dist.seed.has_value() ? *dist.seed : std::random_device{}());
            std::shuffle(allSamples.begin(), allSamples.end(), shuffleRng);
            if(needed < allSamples.size()) {
                allSamples.resize(static_cast<size_t>(needed));
            }
            generatedPoints = std::move(allSamples);
            break;
        }
    }

    std::vector<AgentConfig> generatedAgents{};
    generatedAgents.reserve(generatedPoints.size());
    std::mt19937_64 profileRng(
        dist.seed.has_value() ? (*dist.seed + 0xD1B54A32D192ED03ull) : std::random_device{}());
    for(const auto& p : generatedPoints) {
        const auto& picked = PickProfile(effectiveProfiles, profileRng);
        // A <population> overrides speed and age group; everything else - radius,
        // time gap, pre-movement, escape route - still comes from the profile,
        // so the two can be combined.
        const AgentSpawnProfile drawn =
            dist.population.has_value()
                ? DrawFromPopulation(*dist.population, picked, profileRng)
                : picked;
        const AgentSpawnProfile& profile = drawn;
        generatedAgents.push_back(
            AgentConfig{
                .position = p,
                .radius = profile.radius,
                .timeGap = profile.timeGap,
                .desiredSpeed = profile.desiredSpeed,
                .ageGroup = profile.ageGroup,
                .avatarHint = profile.avatarHint,
                .escapeRoute = profile.escapeRoute,
                .preMovementTime = profile.preMovementTime,
            });
    }
    return generatedAgents;
}

/// True when a person standing at `position` would spot the smoke.
///
/// A person does not stare at the air in front of their nose: they look around,
/// and they see a layer building up overhead. So this samples the watch plane
/// not only under the person but on two rings around them, out to sightRange,
/// and takes the worst visibility found. Anything at or below alertsBelow counts
/// as "there is smoke over there, and I can see it".
///
/// Deliberately NOT modelled: walls. There is no occlusion test, so somebody can
/// notice smoke that is in fact in the room next door. Erring towards noticing
/// too early is the safer direction for the person, and the honest alternative -
/// ray casting against the geometry - is a different piece of work.
/// `position` and `facing` are in simulation coordinates, because that is what
/// the geometry is in; `offset` converts a point to the FDS grid for sampling.
/// Mixing the two would test the line of sight against walls that are somewhere
/// else entirely.
///
/// What is measured is how far the person can actually SEE, not how dense the
/// smoke happens to be where they stand. Along each sight line the extinction is
/// integrated step by step; the view ends where the accumulated optical depth
/// reaches the visibility factor - the same convention the field itself uses to
/// turn density into a visibility. Looking through twenty metres of thin haze
/// therefore blocks the view just as a few metres of thick smoke would, which is
/// what the eye does and what a single sample at one point cannot reproduce.
bool NoticesSmoke(
    FdsVisibilityField& field,
    const CollisionGeometry* geometry,
    double time,
    Point position,
    Point facing,
    Point offset,
    double sightRange,
    double sightAngleDegrees,
    double alertsBelow,
    double visibilityFactor,
    double maximumVisibility,
    double dimmedFraction)
{
    const auto here = field.Sample(time, Point{position.x + offset.x, position.y + offset.y});
    if(here.has_value() && *here <= alertsBelow) {
        return true;
    }
    if(sightRange <= 0.0) {
        return false;
    }

    constexpr double kPi = 3.141592653589793;
    constexpr double kTwoPi = 6.283185307179586;

    // Where the cone points. A standing agent still has an orientation - it is
    // recomputed every step from destination - pos - so this is "towards the
    // exit", not "wherever the head happens to be". That is precisely why the
    // full circle is the default.
    double centre = 0.0;
    const bool limited = sightAngleDegrees < 359.999;
    if(limited) {
        if(facing.x == 0.0 && facing.y == 0.0) {
            return false; // no direction to look in
        }
        centre = std::atan2(facing.y, facing.x);
    }
    const double half = kPi * sightAngleDegrees / 360.0;

    // Eight directions: enough to catch a smoke front from any side without
    // turning the trigger into the run's hot loop. With a narrowed cone the same
    // eight are spread across the aperture instead.
    constexpr int kDirections = 8;
    // One metre per step is well below the FDS cell size of this kind of case,
    // so the integral does not miss a pocket of smoke; the cap keeps a very
    // large sight range from becoming expensive.
    constexpr double kStep = 1.0;
    constexpr int kMaxSteps = 64;
    const int steps =
        std::min(kMaxSteps, std::max(1, static_cast<int>(std::ceil(sightRange / kStep))));
    const double ds = sightRange / steps;

    for(int direction = 0; direction < kDirections; ++direction) {
        const double angle =
            limited
                ? centre - half +
                      2.0 * half * (static_cast<double>(direction) + 0.5) / kDirections
                : kTwoPi * static_cast<double>(direction) / kDirections;
        const double dx = std::cos(angle);
        const double dy = std::sin(angle);

        double opticalDepth = 0.0;
        Point previous = position;
        for(int step = 1; step <= steps; ++step) {
            const double distance = ds * step;
            const Point probe{position.x + dx * distance, position.y + dy * distance};

            // A wall ends the line of sight here. Tested segment by segment so
            // the view stops AT the wall instead of being discarded entirely -
            // smoke on this side of it is still seen.
            if(geometry != nullptr && geometry->IntersectsAny(LineSegment{previous, probe})) {
                break;
            }
            previous = probe;

            const auto seen =
                field.Sample(time, Point{probe.x + offset.x, probe.y + offset.y});
            if(!seen.has_value()) {
                continue; // outside the FDS meshes: nothing to see, nothing to add
            }

            // Local extinction from the local visibility, the inverse of what
            // the field does - minus the floor that same cap implies. Clear air
            // is reported as maximumVisibility rather than as infinity, so the
            // naive inverse hands back visibilityFactor / maximumVisibility for
            // air that contains nothing at all. Over a long enough sight line
            // that alone accumulates into a "the view is dimmed" verdict and
            // everybody sets off in a spotless building. Subtracting the floor
            // makes clear air contribute exactly zero and stays continuous.
            const double extinction =
                std::max(0.0,
                         visibilityFactor / std::max(*seen, 1e-6) -
                             visibilityFactor / maximumVisibility);
            opticalDepth += extinction * ds;

            // How much of the view is lost: 1 - exp(-opticalDepth). This is what
            // the eye reacts to. The "visibility" convention (3 / extinction) is
            // far less sensitive - twenty-five metres of haze that swallows 70 %
            // of the light still counts as sixty metres of visibility under it,
            // which is why a point sample says "clear" for a view that plainly
            // is not.
            if(1.0 - std::exp(-opticalDepth) >= dimmedFraction) {
                return true;
            }
        }
    }
    return false;
}

CliArgs ParseCliArgs(int argc, char** argv)
{
    CliArgs args{};
    bool showHelp = false;

    for(int idx = 1; idx < argc; ++idx) {
        const std::string token = argv[idx];
        if(token == "--help" || token == "-h") {
            showHelp = true;
        } else if(token == "--max-iterations") {
            if(idx + 1 >= argc) {
                throw std::runtime_error("Missing value for --max-iterations");
            }
            args.maxIterationsOverride =
                ParseUint64(std::string(argv[++idx]), "--max-iterations");
        } else if(token == "--out-jsp") {
            if(idx + 1 >= argc) {
                throw std::runtime_error("Missing value for --out-jsp");
            }
            args.outputPath = std::string(argv[++idx]);
        } else if(token == "--fds-smv") {
            if(idx + 1 >= argc) {
                throw std::runtime_error("Missing value for --fds-smv");
            }
            args.fdsSmvPath = std::string(argv[++idx]);
        } else if(token == "--every-nth-frame") {
            if(idx + 1 >= argc) {
                throw std::runtime_error("Missing value for --every-nth-frame");
            }
            const auto parsed = ParseUint64(std::string(argv[++idx]), "--every-nth-frame");
            if(parsed > static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
                throw std::runtime_error("--every-nth-frame is out of supported range");
            }
            args.everyNthFrame = static_cast<uint32_t>(parsed);
        } else if(token == "--compression-level") {
            if(idx + 1 >= argc) {
                throw std::runtime_error("Missing value for --compression-level");
            }
            const auto parsed = ParseUint64(std::string(argv[++idx]), "--compression-level");
            if(parsed < static_cast<uint64_t>(JSP_MIN_COMPRESSION_LEVEL) ||
               parsed > static_cast<uint64_t>(JSP_MAX_COMPRESSION_LEVEL)) {
                throw std::runtime_error("--compression-level must be in [1, 12]");
            }
            args.compressionLevel = static_cast<int>(parsed);
        } else if(token.rfind("-", 0) == 0) {
            throw std::runtime_error("Unknown option: " + token);
        } else if(args.scenarioPath.empty()) {
            args.scenarioPath = token;
        } else {
            throw std::runtime_error(
                "Only one XML scenario file can be provided on the command line");
        }
    }

    if(showHelp) {
        PrintUsage(argv[0]);
        std::exit(0);
    }

    if(args.scenarioPath.empty()) {
        throw std::runtime_error("Missing required XML scenario file");
    }

    return args;
}

ScenarioConfig ParseScenarioConfig(const std::string& path)
{
    pt::ptree tree{};
    pt::read_xml(path, tree, pt::xml_parser::trim_whitespace);

    const auto& scenario = tree.get_child("scenario");
    ScenarioConfig config{};

    config.dt = scenario.get<double>("<xmlattr>.dt", config.dt);
    if(config.dt <= 0.0) {
        throw std::runtime_error("scenario.dt must be > 0");
    }

    config.maxIterations =
        scenario.get<uint64_t>("<xmlattr>.max_iterations", config.maxIterations);
    if(config.maxIterations == 0) {
        throw std::runtime_error("scenario.max_iterations must be > 0");
    }

    // flowsimpro_* attributes are optional metadata of the exporting application.
    {
        const auto elevation =
            scenario.get_optional<double>("<xmlattr>.flowsimpro_storey_elevation");
        const auto centeringX =
            scenario.get_optional<double>("<xmlattr>.flowsimpro_centering_x");
        const auto centeringY =
            scenario.get_optional<double>("<xmlattr>.flowsimpro_centering_y");
        const auto centeringZ =
            scenario.get_optional<double>("<xmlattr>.flowsimpro_centering_z");
        if(elevation || centeringX || centeringY || centeringZ) {
            ScenarioConfig::SourceModelInfo info{};
            info.storeyElevation = elevation.value_or(0.0);
            info.centeringX = centeringX.value_or(0.0);
            info.centeringY = centeringY.value_or(0.0);
            info.centeringZ = centeringZ.value_or(0.0);
            config.sourceModel = info;
        }
    }

    if(const auto fdsNodeOpt = scenario.get_child_optional("fds_hazard"); fdsNodeOpt) {
        const auto& fdsNode = *fdsNodeOpt;
        ScenarioConfig::FdsHazardConfig fds{};
        if(const auto configuredFile =
               fdsNode.get_optional<std::string>("<xmlattr>.file");
           configuredFile) {
            if(configuredFile->empty()) {
                throw std::runtime_error("scenario.fds_hazard.file must not be empty");
            }
            fs::path file(*configuredFile);
            if(file.is_relative()) {
                file = fs::absolute(fs::path(path)).parent_path() / file;
            }
            fds.smvFile = file.lexically_normal();
        }
        fds.eyeHeight =
            fdsNode.get<double>("<xmlattr>.eye_height", fds.eyeHeight);
        if(const auto sliceZ = fdsNode.get_optional<double>("<xmlattr>.slice_z"); sliceZ) {
            fds.sliceZ = *sliceZ;
        }
        fds.zTolerance =
            fdsNode.get<double>("<xmlattr>.z_tolerance", fds.zTolerance);
        fds.offsetX = fdsNode.get<double>("<xmlattr>.offset_x", fds.offsetX);
        fds.offsetY = fdsNode.get<double>("<xmlattr>.offset_y", fds.offsetY);
        fds.updateInterval =
            fdsNode.get<double>("<xmlattr>.update_interval", fds.updateInterval);

        if(const auto visibilityNode = fdsNode.get_child_optional("visibility"); visibilityNode) {
            fds.awarenessBelow = visibilityNode->get<double>(
                "<xmlattr>.awareness_below", fds.awarenessBelow);
            fds.severeBelow = visibilityNode->get<double>(
                "<xmlattr>.severe_below", fds.severeBelow);
            fds.minimumSpeedFactor = visibilityNode->get<double>(
                "<xmlattr>.minimum_speed_factor", fds.minimumSpeedFactor);
            fds.visibilityFactor = visibilityNode->get<double>(
                "<xmlattr>.visibility_factor", fds.visibilityFactor);
            fds.maximumVisibility = visibilityNode->get<double>(
                "<xmlattr>.maximum_visibility", fds.maximumVisibility);
        }

        fds.smokeAlertsBelow =
            fdsNode.get<double>("<xmlattr>.smoke_alerts_below", fds.smokeAlertsBelow);
        fds.smokeReaction =
            fdsNode.get<double>("<xmlattr>.smoke_reaction", fds.smokeReaction);
        if(!std::isfinite(fds.smokeAlertsBelow) || fds.smokeAlertsBelow < 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.smoke_alerts_below must be finite and >= 0");
        }
        if(!std::isfinite(fds.smokeReaction) || fds.smokeReaction < 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.smoke_reaction must be finite and >= 0");
        }
        fds.smokeWatchHeight =
            fdsNode.get<double>("<xmlattr>.smoke_watch_height", fds.smokeWatchHeight);
        if(!std::isfinite(fds.smokeWatchHeight) || fds.smokeWatchHeight <= 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.smoke_watch_height must be finite and > 0");
        }
        fds.smokeSightRange =
            fdsNode.get<double>("<xmlattr>.smoke_sight_range", fds.smokeSightRange);
        if(!std::isfinite(fds.smokeSightRange) || fds.smokeSightRange < 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.smoke_sight_range must be finite and >= 0");
        }
        fds.smokeViewDimmed =
            fdsNode.get<double>("<xmlattr>.smoke_view_dimmed", fds.smokeViewDimmed);
        if(!std::isfinite(fds.smokeViewDimmed) || fds.smokeViewDimmed <= 0.0 ||
           fds.smokeViewDimmed >= 1.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.smoke_view_dimmed must be finite and in (0, 1)");
        }
        fds.smokeSightAngle =
            fdsNode.get<double>("<xmlattr>.smoke_sight_angle", fds.smokeSightAngle);
        if(!std::isfinite(fds.smokeSightAngle) || fds.smokeSightAngle <= 0.0 ||
           fds.smokeSightAngle > 360.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.smoke_sight_angle must be finite and in (0, 360]");
        }

        if(!std::isfinite(fds.eyeHeight) || fds.eyeHeight <= 0.0) {
            throw std::runtime_error("scenario.fds_hazard.eye_height must be finite and > 0");
        }
        if(fds.sliceZ && !std::isfinite(*fds.sliceZ)) {
            throw std::runtime_error("scenario.fds_hazard.slice_z must be finite");
        }
        if(!std::isfinite(fds.zTolerance) || fds.zTolerance < 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.z_tolerance must be finite and >= 0");
        }
        if(!std::isfinite(fds.offsetX) || !std::isfinite(fds.offsetY)) {
            throw std::runtime_error("scenario.fds_hazard offsets must be finite");
        }
        if(!std::isfinite(fds.updateInterval) || fds.updateInterval <= 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.update_interval must be finite and > 0");
        }
        if(!std::isfinite(fds.awarenessBelow) || fds.awarenessBelow <= 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.visibility.awareness_below must be finite and > 0");
        }
        if(!std::isfinite(fds.severeBelow) || fds.severeBelow < 0.0 ||
           fds.severeBelow >= fds.awarenessBelow) {
            throw std::runtime_error(
                "scenario.fds_hazard.visibility.severe_below must be finite, >= 0, and less "
                "than awareness_below");
        }
        if(!std::isfinite(fds.minimumSpeedFactor) || fds.minimumSpeedFactor <= 0.0 ||
           fds.minimumSpeedFactor > 1.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.visibility.minimum_speed_factor must be in (0, 1]");
        }
        if(!std::isfinite(fds.visibilityFactor) || fds.visibilityFactor <= 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.visibility.visibility_factor must be finite and > 0");
        }
        if(!std::isfinite(fds.maximumVisibility) || fds.maximumVisibility <= 0.0) {
            throw std::runtime_error(
                "scenario.fds_hazard.visibility.maximum_visibility must be finite and > 0");
        }
        config.fdsHazard = std::move(fds);
    }

    const auto modelType =
        scenario.get<std::string>("model.<xmlattr>.type", "collision_free_speed");
    if(modelType != "collision_free_speed") {
        throw std::runtime_error(
            "Only model type 'collision_free_speed' is supported by jupedsim");
    }

    config.strengthNeighborRepulsion = scenario.get<double>(
        "model.<xmlattr>.strength_neighbor_repulsion",
        config.strengthNeighborRepulsion);
    config.rangeNeighborRepulsion = scenario.get<double>(
        "model.<xmlattr>.range_neighbor_repulsion",
        config.rangeNeighborRepulsion);
    config.strengthGeometryRepulsion = scenario.get<double>(
        "model.<xmlattr>.strength_geometry_repulsion",
        config.strengthGeometryRepulsion);
    config.rangeGeometryRepulsion = scenario.get<double>(
        "model.<xmlattr>.range_geometry_repulsion",
        config.rangeGeometryRepulsion);

    const auto geometryNodeOpt = scenario.get_child_optional("geometry");
    if(!geometryNodeOpt) {
        if(scenario.get_child_optional("floors")) {
            throw std::runtime_error(
                "Scenario contains <floors>: multi-floor scenarios are not supported. "
                "Export a single storey, which writes <geometry> directly under <scenario>.");
        }
        throw std::runtime_error("Missing node scenario.geometry");
    }
    const auto& geometryNode = *geometryNodeOpt;
    config.walkable =
        ParsePolygon(geometryNode.get_child("walkable"), "scenario.geometry.walkable");
    for(const auto& [tag, node] : geometryNode) {
        if(tag == "obstacle") {
            config.obstacles.push_back(ParsePolygon(node, "scenario.geometry.obstacle"));
        }
    }

    if(const auto singleExitNode = scenario.get_child_optional("exit"); singleExitNode) {
        config.exitPolygon = ParsePolygon(*singleExitNode, "scenario.exit");
    }

    if(const auto decisionNodeOpt = scenario.get_child_optional("decision"); decisionNodeOpt) {
        ScenarioConfig::DecisionConfig decision{};
        const auto& decisionNode = *decisionNodeOpt;
        decision.position = ParsePoint(decisionNode, "scenario.decision");
        decision.distance =
            decisionNode.get<double>("<xmlattr>.distance", decision.distance);
        if(decision.distance <= 0.0) {
            throw std::runtime_error("scenario.decision.distance must be > 0");
        }
        config.decision = decision;
    }

    if(const auto exitsNodeOpt = scenario.get_child_optional("exits"); exitsNodeOpt) {
        ScenarioConfig::MultiExitConfig multiExit{};
        const auto& exitsNode = *exitsNodeOpt;
        multiExit.transitionMode = ParseExitTransitionMode(
            exitsNode.get<std::string>("<xmlattr>.mode", "adaptive"));
        multiExit.fixedExitIndex = static_cast<size_t>(
            exitsNode.get<uint64_t>("<xmlattr>.fixed_index", multiExit.fixedExitIndex));
        multiExit.expectedTimeWeight = exitsNode.get<double>(
            "<xmlattr>.expected_time_weight",
            multiExit.expectedTimeWeight);
        multiExit.densityWeight =
            exitsNode.get<double>("<xmlattr>.density_weight", multiExit.densityWeight);
        multiExit.queueWeight =
            exitsNode.get<double>("<xmlattr>.queue_weight", multiExit.queueWeight);
        multiExit.switchPenalty =
            exitsNode.get<double>("<xmlattr>.switch_penalty", multiExit.switchPenalty);
        multiExit.decisionInterval =
            exitsNode.get<uint64_t>("<xmlattr>.decision_interval", multiExit.decisionInterval);
        multiExit.reconsiderationThreshold = exitsNode.get<double>(
            "<xmlattr>.reconsideration_threshold",
            multiExit.reconsiderationThreshold);

        if(multiExit.expectedTimeWeight < 0.0 || multiExit.densityWeight < 0.0 ||
           multiExit.queueWeight < 0.0 || multiExit.switchPenalty < 0.0) {
            throw std::runtime_error(
                "scenario.exits weights/penalty must be >= 0");
        }
        if(multiExit.decisionInterval == 0) {
            throw std::runtime_error("scenario.exits.decision_interval must be > 0");
        }
        if(multiExit.reconsiderationThreshold < 0.0) {
            throw std::runtime_error(
                "scenario.exits.reconsideration_threshold must be >= 0");
        }

        for(const auto& [tag, node] : exitsNode) {
            if(tag != "exit") {
                continue;
            }
            multiExit.polygons.push_back(ParsePolygon(node, "scenario.exits.exit"));
            const auto weight = node.get<uint64_t>("<xmlattr>.weight", 1);
            if(weight == 0) {
                throw std::runtime_error("scenario.exits.exit.weight must be > 0");
            }
            multiExit.roundRobinWeights.push_back(weight);
        }

        if(multiExit.polygons.size() < 2) {
            throw std::runtime_error("scenario.exits requires at least 2 <exit/> entries");
        }
        if(multiExit.fixedExitIndex >= multiExit.polygons.size()) {
            throw std::runtime_error(
                "scenario.exits.fixed_index out of range");
        }

        config.multiExit = multiExit;
    }

    if(config.exitPolygon.has_value() && config.multiExit.has_value()) {
        throw std::runtime_error("Use either <exit> or <exits>, not both");
    }
    if(!config.exitPolygon.has_value() && !config.multiExit.has_value()) {
        throw std::runtime_error("Missing <exit> or <exits> in scenario");
    }
    if(config.multiExit.has_value() && !config.decision.has_value()) {
        throw std::runtime_error("scenario.exits requires a <decision .../> element");
    }
    if(config.decision.has_value() && !config.multiExit.has_value()) {
        throw std::runtime_error("<decision> is only valid together with <exits>");
    }

    if(const auto stairNodeOpt = scenario.get_child_optional("stair"); stairNodeOpt) {
        ScenarioConfig::StairConfig stair{};
        const auto& stairNode = *stairNodeOpt;
        stair.position = ParsePoint(stairNode, "scenario.stair");
        stair.distance = stairNode.get<double>("<xmlattr>.distance", stair.distance);
        stair.length = stairNode.get<double>("<xmlattr>.length", stair.length);
        stair.ascending = stairNode.get<bool>("<xmlattr>.ascending", stair.ascending);
        // speed_factor stays as the value for both directions, so an existing
        // scenario keeps its meaning; up_/down_speed_factor override per direction.
        const double bothFactors =
            stairNode.get<double>("<xmlattr>.speed_factor", stair.upSpeedFactor);
        stair.upSpeedFactor =
            stairNode.get<double>("<xmlattr>.up_speed_factor", bothFactors);
        stair.downSpeedFactor =
            stairNode.get<double>("<xmlattr>.down_speed_factor", bothFactors);
        stair.upSpeed = stairNode.get<double>("<xmlattr>.up_speed", stair.upSpeed);
        stair.downSpeed = stairNode.get<double>("<xmlattr>.down_speed", stair.downSpeed);
        stair.waitingTime =
            stairNode.get<double>("<xmlattr>.waiting_time", stair.waitingTime);

        if(stair.distance <= 0.0) {
            throw std::runtime_error("scenario.stair.distance must be > 0");
        }
        if(stair.length < 0.0) {
            throw std::runtime_error("scenario.stair.length must be >= 0");
        }
        if(stair.upSpeedFactor <= 0.0 || stair.downSpeedFactor <= 0.0) {
            throw std::runtime_error(
                "scenario.stair speed factors must be > 0 "
                "(speed_factor, up_speed_factor, down_speed_factor)");
        }
        if(stair.upSpeed < 0.0 || stair.downSpeed < 0.0) {
            throw std::runtime_error(
                "scenario.stair.up_speed / down_speed must be >= 0 (0 = use the factor)");
        }
        if(stair.waitingTime < 0.0) {
            throw std::runtime_error("scenario.stair.waiting_time must be >= 0");
        }
        config.stair = stair;
    }
    if(const auto rampNodeOpt = scenario.get_child_optional("ramp"); rampNodeOpt) {
        ScenarioConfig::RampConfig ramp{};
        const auto& rampNode = *rampNodeOpt;
        ramp.position = ParsePoint(rampNode, "scenario.ramp");
        ramp.distance = rampNode.get<double>("<xmlattr>.distance", ramp.distance);
        ramp.length = rampNode.get<double>("<xmlattr>.length", ramp.length);
        ramp.upSpeedFactor =
            rampNode.get<double>("<xmlattr>.up_speed_factor", ramp.upSpeedFactor);
        ramp.downSpeedFactor =
            rampNode.get<double>("<xmlattr>.down_speed_factor", ramp.downSpeedFactor);
        ramp.waitingTime =
            rampNode.get<double>("<xmlattr>.waiting_time", ramp.waitingTime);
        if(const auto asc = rampNode.get_optional<std::string>("<xmlattr>.ascending"); asc) {
            ramp.ascending = ParseBool(*asc, "scenario.ramp.ascending");
        }

        if(ramp.distance <= 0.0) {
            throw std::runtime_error("scenario.ramp.distance must be > 0");
        }
        if(ramp.length < 0.0) {
            throw std::runtime_error("scenario.ramp.length must be >= 0");
        }
        if(ramp.upSpeedFactor <= 0.0) {
            throw std::runtime_error("scenario.ramp.up_speed_factor must be > 0");
        }
        if(ramp.downSpeedFactor <= 0.0) {
            throw std::runtime_error("scenario.ramp.down_speed_factor must be > 0");
        }
        if(ramp.waitingTime < 0.0) {
            throw std::runtime_error("scenario.ramp.waiting_time must be >= 0");
        }
        config.ramp = ramp;
    }

    if(config.stair.has_value() && config.ramp.has_value()) {
        throw std::runtime_error("Use either <stair> or <ramp>, not both in one scenario");
    }

    auto parseExplicitAgent = [&](const pt::ptree& node, const std::string& context) {
        AgentConfig agent{};
        agent.position = ParsePoint(node, context);
        agent.radius = node.get<double>("<xmlattr>.radius", agent.radius);
        agent.timeGap = node.get<double>("<xmlattr>.time_gap", agent.timeGap);
        agent.desiredSpeed = node.get<double>("<xmlattr>.desired_speed", agent.desiredSpeed);
        agent.preMovementTime =
            node.get<double>("<xmlattr>.pre_movement", agent.preMovementTime);
        agent.ageGroup =
            ToLowerAscii(node.get<std::string>("<xmlattr>.age_group", std::string{}));
        agent.avatarHint =
            ToLowerAscii(node.get<std::string>("<xmlattr>.avatar_hint", std::string{}));
        {
            // See ParseAgentProfile: signed so a negative index is rejected
            // instead of wrapping around.
            const auto route = node.get<int64_t>("<xmlattr>.escape_route", 0);
            if(route < 0) {
                throw std::runtime_error(context + ".escape_route must be >= 1");
            }
            agent.escapeRoute = static_cast<size_t>(route);
        }
        if(agent.radius <= 0.0) {
            throw std::runtime_error(context + ".radius must be > 0");
        }
        if(agent.timeGap <= 0.0) {
            throw std::runtime_error(context + ".time_gap must be > 0");
        }
        if(agent.desiredSpeed <= 0.0) {
            throw std::runtime_error(context + ".desired_speed must be > 0");
        }
        // ">= 0", not "> 0": 0.0 is the default and means "starts immediately".
        if(!std::isfinite(agent.preMovementTime) || agent.preMovementTime < 0.0) {
            throw std::runtime_error(context + ".pre_movement must be finite and >= 0");
        }
        config.agents.push_back(agent);
    };

    std::vector<pt::ptree> detachedProfiles{};
    if(const auto agentsNodeOpt = scenario.get_child_optional("agents"); agentsNodeOpt) {
        const auto& agentsNode = *agentsNodeOpt;
        for(const auto& [tag, node] : agentsNode) {
            if(tag == "agent") {
                parseExplicitAgent(node, "scenario.agents.agent");
            } else if(tag == "distribution") {
                if(config.distribution.has_value()) {
                    throw std::runtime_error(
                        "Distribution defined multiple times (scenario.agents.distribution)");
                }
                config.distribution = ParseDistributionConfig(node, "scenario.agents.distribution");
            } else if(tag == "profile") {
                detachedProfiles.push_back(node);
            }
        }
    }

    if(const auto topDistribution = scenario.get_child_optional("agent_distribution"); topDistribution) {
        if(config.distribution.has_value()) {
            throw std::runtime_error(
                "Distribution defined both in scenario.agents and scenario.agent_distribution");
        }
        config.distribution = ParseDistributionConfig(
            *topDistribution,
            "scenario.agent_distribution");
    }

    if(!detachedProfiles.empty()) {
        if(!config.distribution.has_value()) {
            throw std::runtime_error(
                "Found <agents><profile/> but no <distribution/> in scenario");
        }
        for(const auto& profileNode : detachedProfiles) {
            config.distribution->profiles.push_back(
                ParseSpawnProfile(
                    profileNode,
                    "scenario.agents.profile",
                    config.distribution->defaultProfile));
        }
    }

    if(config.agents.empty() && !config.distribution.has_value()) {
        throw std::runtime_error(
            "scenario.agents must contain at least one <agent/> or one <distribution/>");
    }

    return config;
}

void WriteAll(std::ostream& out, const void* data, std::size_t size)
{
    out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    if(!out) {
        throw std::runtime_error("Failed to write .jsp file");
    }
}

void WriteU32LE(std::ostream& out, uint32_t value)
{
    const uint8_t bytes[4] = {
        static_cast<uint8_t>(value & 0xFFu),
        static_cast<uint8_t>((value >> 8u) & 0xFFu),
        static_cast<uint8_t>((value >> 16u) & 0xFFu),
        static_cast<uint8_t>((value >> 24u) & 0xFFu),
    };
    WriteAll(out, bytes, sizeof(bytes));
}

void WriteU64LE(std::ostream& out, uint64_t value)
{
    const uint8_t bytes[8] = {
        static_cast<uint8_t>(value & 0xFFu),
        static_cast<uint8_t>((value >> 8u) & 0xFFu),
        static_cast<uint8_t>((value >> 16u) & 0xFFu),
        static_cast<uint8_t>((value >> 24u) & 0xFFu),
        static_cast<uint8_t>((value >> 32u) & 0xFFu),
        static_cast<uint8_t>((value >> 40u) & 0xFFu),
        static_cast<uint8_t>((value >> 48u) & 0xFFu),
        static_cast<uint8_t>((value >> 56u) & 0xFFu),
    };
    WriteAll(out, bytes, sizeof(bytes));
}

void WriteF64LE(std::ostream& out, double value)
{
    uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    WriteU64LE(out, bits);
}

void WriteF32LE(std::ostream& out, float value)
{
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    WriteU32LE(out, bits);
}

void AppendU32LE(std::vector<uint8_t>& out, uint32_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 16u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 24u) & 0xFFu));
}

void AppendU64LE(std::vector<uint8_t>& out, uint64_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 16u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 24u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 32u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 40u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 48u) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 56u) & 0xFFu));
}

void AppendF32LE(std::vector<uint8_t>& out, float value)
{
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    AppendU32LE(out, bits);
}

uint64_t StreamPosToU64(std::streampos pos)
{
    const auto asOff = static_cast<std::streamoff>(pos);
    if(asOff < 0) {
        throw std::runtime_error("Invalid negative stream position");
    }
    return static_cast<uint64_t>(asOff);
}

std::string PathToUtf8(const fs::path& path)
{
    const auto encoded = path.generic_u8string();
    return std::string(
        reinterpret_cast<const char*>(encoded.data()),
        encoded.size());
}

FdsJspMetadata MakeFdsJspMetadata(
    const fs::path& jspPath,
    const ScenarioConfig::FdsHazardConfig& config,
    double sampleZ)
{
    if(!config.smvFile.has_value()) {
        throw std::runtime_error("Internal error: FDS JSP metadata has no SMV file");
    }

    const auto absoluteJsp = fs::absolute(jspPath).lexically_normal();
    const auto absoluteSmv = fs::absolute(*config.smvFile).lexically_normal();
    std::error_code error{};
    auto storedPath = fs::relative(absoluteSmv, absoluteJsp.parent_path(), error);
    const bool pathIsRelative = !error && !storedPath.empty();
    if(!pathIsRelative) {
        storedPath = absoluteSmv;
    }

    return FdsJspMetadata{
        .smvPathUtf8 = PathToUtf8(storedPath),
        .pathIsRelative = pathIsRelative,
        .sampleZ = sampleZ,
        .zTolerance = config.zTolerance,
        .offsetX = config.offsetX,
        .offsetY = config.offsetY,
        .timeOffsetSeconds = 0.0,
        .updateInterval = config.updateInterval,
        .awarenessBelow = config.awarenessBelow,
        .severeBelow = config.severeBelow,
        .minimumSpeedFactor = config.minimumSpeedFactor,
        .visibilityFactor = config.visibilityFactor,
        .maximumVisibility = config.maximumVisibility,
    };
}

class JspTrajectoryWriter
{
public:
    JspTrajectoryWriter(
        fs::path path,
        double dt,
        uint32_t everyNthFrame,
        int compressionLevel,
        std::vector<AgentProfileEntry> agentProfiles,
        std::optional<ScenarioConfig::SourceModelInfo> sourceModel,
        std::optional<FdsJspMetadata> fdsMetadata)
        : _path(std::move(path))
        , _everyNthFrame(everyNthFrame)
        , _compressionLevel(compressionLevel)
        , _agentProfiles(std::move(agentProfiles))
        , _sourceModel(sourceModel)
        , _fdsMetadata(std::move(fdsMetadata))
    {
        _out.open(
            _path,
            std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
        if(!_out) {
            throw std::runtime_error(
                "Could not create output file: " + _path.string());
        }

        _compressor = libdeflate_alloc_compressor(_compressionLevel);
        if(_compressor == nullptr) {
            throw std::runtime_error("Could not allocate libdeflate compressor");
        }

        std::sort(
            _agentProfiles.begin(),
            _agentProfiles.end(),
            [](const AgentProfileEntry& lhs, const AgentProfileEntry& rhs) {
                return lhs.agentId < rhs.agentId;
            });
        if(_agentProfiles.size() > static_cast<size_t>(std::numeric_limits<uint32_t>::max())) {
            throw std::runtime_error("Too many agents to encode metadata in .jsp");
        }
        if(_fdsMetadata.has_value()) {
            const auto pathSize = _fdsMetadata->smvPathUtf8.size();
            if(pathSize == 0 ||
               pathSize > static_cast<size_t>(std::numeric_limits<uint32_t>::max()) -
                              JSP_HAZARD_FIXED_PAYLOAD_SIZE) {
                throw std::runtime_error("FDS SMV path is too long to encode in .jsp");
            }
        }

        WriteHeader(dt);
    }

    ~JspTrajectoryWriter()
    {
        if(_compressor != nullptr) {
            libdeflate_free_compressor(_compressor);
            _compressor = nullptr;
        }
    }

    void WriteFrame(Simulation& simulation, bool force = false)
    {
        const auto iteration = simulation.Iteration();
        if(_lastWrittenIteration.has_value() && *_lastWrittenIteration == iteration) {
            return;
        }
        if(!force && (iteration % _everyNthFrame != 0)) {
            return;
        }

        std::vector<uint8_t> uncompressed{};
        const auto& agents = simulation.Agents();
        uncompressed.reserve(agents.size() * JSP_RECORD_SIZE);
        for(const auto& agent : agents) {
            AppendU64LE(uncompressed, agent.id.getID());
            AppendF32LE(uncompressed, static_cast<float>(agent.pos.x));
            AppendF32LE(uncompressed, static_cast<float>(agent.pos.y));
            AppendF32LE(uncompressed, static_cast<float>(agent.orientation.x));
            AppendF32LE(uncompressed, static_cast<float>(agent.orientation.y));
            // Single-floor scenarios: every agent stays on floor 0.
            AppendU32LE(uncompressed, 0u);
        }

        const auto bound =
            libdeflate_deflate_compress_bound(_compressor, uncompressed.size());
        std::vector<uint8_t> compressed(bound);
        const uint8_t dummy = 0;
        const void* inputPtr = uncompressed.empty() ? static_cast<const void*>(&dummy)
                                                    : static_cast<const void*>(uncompressed.data());
        const auto compressedSize = libdeflate_deflate_compress(
            _compressor,
            inputPtr,
            uncompressed.size(),
            compressed.data(),
            compressed.size());
        if(compressedSize == 0) {
            throw std::runtime_error("libdeflate compression failed");
        }

        const auto frameOffset = StreamPosToU64(_out.tellp());
        WriteAll(_out, compressed.data(), compressedSize);

        _index.push_back(
            FrameIndexEntry{
                .iteration = iteration,
                .timeSeconds = simulation.ElapsedTime(),
                .agentCount = static_cast<uint32_t>(agents.size()),
                .dataOffset = frameOffset,
                .compressedSize = static_cast<uint64_t>(compressedSize),
                .uncompressedSize = static_cast<uint64_t>(uncompressed.size()),
            });

        _lastWrittenIteration = iteration;
    }

    void Finalize()
    {
        if(_finalized) {
            return;
        }

        const auto indexOffset = StreamPosToU64(_out.tellp());
        for(const auto& entry : _index) {
            WriteU64LE(_out, entry.iteration);
            WriteF64LE(_out, entry.timeSeconds);
            WriteU32LE(_out, entry.agentCount);
            WriteU32LE(_out, 0);
            WriteU64LE(_out, entry.dataOffset);
            WriteU64LE(_out, entry.compressedSize);
            WriteU64LE(_out, entry.uncompressedSize);
        }

        _out.seekp(HEADER_FRAME_COUNT_OFFSET);
        WriteU64LE(_out, static_cast<uint64_t>(_index.size()));
        _out.seekp(HEADER_INDEX_OFFSET);
        WriteU64LE(_out, indexOffset);
        _out.seekp(0, std::ios::end);
        WriteMetadataSection();
        WriteSourceModelSection();
        WriteFdsHazardSection();

        _out.flush();
        if(!_out) {
            throw std::runtime_error("Failed to finalize .jsp file");
        }

        _finalized = true;
    }

    const fs::path& Path() const { return _path; }

private:
    void WriteMetadataSection()
    {
        if(_agentProfiles.empty()) {
            return;
        }

        WriteAll(_out, JSP_META_MAGIC, sizeof(JSP_META_MAGIC));
        WriteU32LE(_out, JSP_META_VERSION);
        WriteU32LE(_out, JSP_META_RECORD_SIZE);
        WriteU32LE(_out, static_cast<uint32_t>(_agentProfiles.size()));

        for(const auto& profile : _agentProfiles) {
            WriteU64LE(_out, profile.agentId);
            const uint8_t codes[4] = {profile.ageGroupCode, profile.avatarHintCode, 0, 0};
            WriteAll(_out, codes, sizeof(codes));
            WriteF32LE(_out, profile.desiredSpeed);
            WriteF32LE(_out, profile.timeGap);
            WriteF32LE(_out, profile.radius);
        }
    }

    void WriteSourceModelSection()
    {
        if(!_sourceModel.has_value()) {
            return;
        }

        WriteAll(_out, JSP_FSP_MAGIC, sizeof(JSP_FSP_MAGIC));
        WriteU32LE(_out, JSP_FSP_VERSION);
        WriteU32LE(_out, JSP_FSP_RECORD_SIZE);
        WriteF32LE(_out, static_cast<float>(_sourceModel->storeyElevation));
        WriteF32LE(_out, static_cast<float>(_sourceModel->centeringX));
        WriteF32LE(_out, static_cast<float>(_sourceModel->centeringY));
        WriteF32LE(_out, static_cast<float>(_sourceModel->centeringZ));
    }

    void WriteFdsHazardSection()
    {
        if(!_fdsMetadata.has_value()) {
            return;
        }

        const auto& metadata = *_fdsMetadata;
        const auto pathSize = static_cast<uint32_t>(metadata.smvPathUtf8.size());
        const uint32_t flags = JSP_HAZARD_FLAG_FDS_SMOKE3D |
                               (metadata.pathIsRelative ? JSP_HAZARD_FLAG_RELATIVE_PATH : 0U);
        WriteAll(_out, JSP_HAZARD_MAGIC, sizeof(JSP_HAZARD_MAGIC));
        WriteU32LE(_out, JSP_HAZARD_VERSION);
        WriteU32LE(_out, JSP_HAZARD_FIXED_PAYLOAD_SIZE + pathSize);
        WriteU32LE(_out, flags);
        WriteU32LE(_out, pathSize);
        WriteF32LE(_out, static_cast<float>(metadata.sampleZ));
        WriteF32LE(_out, static_cast<float>(metadata.zTolerance));
        WriteF32LE(_out, static_cast<float>(metadata.offsetX));
        WriteF32LE(_out, static_cast<float>(metadata.offsetY));
        WriteF64LE(_out, metadata.timeOffsetSeconds);
        WriteF32LE(_out, static_cast<float>(metadata.updateInterval));
        WriteF32LE(_out, static_cast<float>(metadata.awarenessBelow));
        WriteF32LE(_out, static_cast<float>(metadata.severeBelow));
        WriteF32LE(_out, static_cast<float>(metadata.minimumSpeedFactor));
        WriteF32LE(_out, static_cast<float>(metadata.visibilityFactor));
        WriteF32LE(_out, static_cast<float>(metadata.maximumVisibility));
        WriteAll(_out, metadata.smvPathUtf8.data(), metadata.smvPathUtf8.size());
    }

private:
    void WriteHeader(double dt)
    {
        const char magic[4] = {'J', 'S', 'P', '1'};
        WriteAll(_out, magic, sizeof(magic));
        WriteU32LE(_out, JSP_VERSION);
        WriteU32LE(_out, JSP_FLAG_DEFLATE);
        WriteF64LE(_out, dt);
        WriteU32LE(_out, JSP_RECORD_SIZE);
        WriteU32LE(_out, _everyNthFrame);
        WriteU64LE(_out, 0); // frame_count, patched in Finalize
        WriteU64LE(_out, 0); // index_offset, patched in Finalize
        WriteU32LE(_out, static_cast<uint32_t>(_compressionLevel));
        WriteU32LE(_out, 0); // reserved
    }

private:
    fs::path _path;
    uint32_t _everyNthFrame{};
    int _compressionLevel{};
    std::fstream _out{};
    libdeflate_compressor* _compressor{nullptr};
    std::vector<FrameIndexEntry> _index{};
    std::vector<AgentProfileEntry> _agentProfiles{};
    std::optional<ScenarioConfig::SourceModelInfo> _sourceModel{};
    std::optional<FdsJspMetadata> _fdsMetadata{};
    std::optional<uint64_t> _lastWrittenIteration{};
    bool _finalized{false};
};

} // namespace

int main(int argc, char** argv)
{
    try {
        const auto args = ParseCliArgs(argc, argv);
        auto config = ParseScenarioConfig(args.scenarioPath);
        if(args.maxIterationsOverride.has_value()) {
            config.maxIterations = *args.maxIterationsOverride;
        }
        if(args.fdsSmvPath.has_value()) {
            if(!config.fdsHazard.has_value()) {
                throw std::runtime_error(
                    "--fds-smv requires an <fds_hazard> section in the scenario XML so the "
                    "visibility behavior is explicit");
            }
            config.fdsHazard->smvFile =
                fs::absolute(fs::path(*args.fdsSmvPath)).lexically_normal();
        }

        std::optional<FdsVisibilityField> fdsVisibility{};
        std::optional<FdsVisibilityField> fdsWatchField{};
        std::optional<double> fdsSampleZ{};
        if(config.fdsHazard.has_value()) {
            auto& fds = *config.fdsHazard;
            if(!fds.smvFile.has_value()) {
                throw std::runtime_error(
                    "<fds_hazard> requires file=\"...smv\" or the --fds-smv override");
            }
            if(ToLowerAscii(fds.smvFile->extension().string()) != ".smv") {
                throw std::runtime_error("FDS hazard input must be a .smv file");
            }
            double targetZ = fds.eyeHeight;
            if(config.sourceModel.has_value()) {
                targetZ += config.sourceModel->storeyElevation;
            }
            if(fds.sliceZ.has_value()) {
                targetZ = *fds.sliceZ;
            }
            fdsSampleZ = targetZ;
            fdsVisibility = FdsVisibilityField::Load(
                *fds.smvFile,
                targetZ,
                fds.zTolerance,
                fds.visibilityFactor,
                fds.maximumVisibility);
            fmt::print(
                "fds_smoke={} meshes={} eye_z={:.3f}m time=[{:.3f},{:.3f}]s\n",
                fdsVisibility->SmvFile().string(),
                fdsVisibility->SliceCount(),
                fdsVisibility->SliceZ(),
                fdsVisibility->FirstTime(),
                fdsVisibility->LastTime());

            // Second plane, only for noticing the fire: people see the layer
            // building up overhead well before it sinks to their eyes. Loaded
            // only when the trigger is actually used, it costs another pass over
            // the Smoke3D data.
            if(fds.smokeAlertsBelow > 0.0) {
                double watchZ = fds.smokeWatchHeight;
                if(config.sourceModel.has_value()) {
                    watchZ += config.sourceModel->storeyElevation;
                }
                try {
                    fdsWatchField = FdsVisibilityField::Load(
                        *fds.smvFile,
                        watchZ,
                        fds.zTolerance,
                        fds.visibilityFactor,
                        fds.maximumVisibility);
                    fmt::print(
                        "fds_smoke_watch z={:.3f}m sight_range={:.2f}m\n",
                        fdsWatchField->SliceZ(),
                        fds.smokeSightRange);
                } catch(const std::exception& error) {
                    // A watch height above the mesh is a configuration mistake,
                    // not a reason to abandon the run: fall back to eye height
                    // and say so, rather than silently changing the meaning.
                    fmt::print(
                        stderr,
                        "warning: smoke_watch_height {:.2f} m unusable ({}); noticing falls "
                        "back to eye height\n",
                        fds.smokeWatchHeight,
                        error.what());
                    fdsWatchField.reset();
                }
            }
        }

        fs::path outputPath{};
        if(args.outputPath.has_value()) {
            outputPath = fs::path(*args.outputPath);
        } else {
            outputPath = fs::path(args.scenarioPath);
            outputPath.replace_extension(".jsp");
        }

        GeometryBuilder geometryBuilder{};
        geometryBuilder.AddAccessibleArea(config.walkable);
        for(const auto& obstacle : config.obstacles) {
            geometryBuilder.ExcludeFromAccessibleArea(obstacle);
        }

        auto model = std::make_unique<CollisionFreeSpeedModel>(
            config.strengthNeighborRepulsion,
            config.rangeNeighborRepulsion,
            config.strengthGeometryRepulsion,
            config.rangeGeometryRepulsion);
        auto geometry = std::make_unique<CollisionGeometry>(geometryBuilder.Build());

        std::vector<AgentConfig> allAgents = config.agents;
        if(config.distribution.has_value()) {
            auto generated =
                GenerateDistributedAgents(*config.distribution, *geometry, allAgents);
            allAgents.insert(
                allAgents.end(),
                std::make_move_iterator(generated.begin()),
                std::make_move_iterator(generated.end()));
        }
        if(allAgents.empty()) {
            throw std::runtime_error("No agents available after parsing/generation");
        }

        Simulation simulation(std::move(model), std::move(geometry), config.dt);

        std::map<BaseStage::ID, TransitionDescription> journeyStages{};
        std::vector<BaseStage::ID> exitStages{};
        if(config.exitPolygon.has_value()) {
            const auto exitStage =
                simulation.AddStage(ExitDescription{Polygon(*config.exitPolygon)});
            journeyStages.emplace(exitStage, NonTransitionDescription{});
            exitStages.push_back(exitStage);
        } else if(config.multiExit.has_value()) {
            for(const auto& polygon : config.multiExit->polygons) {
                const auto exitStage = simulation.AddStage(ExitDescription{Polygon(polygon)});
                journeyStages.emplace(exitStage, NonTransitionDescription{});
                exitStages.push_back(exitStage);
            }
        } else {
            throw std::runtime_error("Internal error: missing exit stage configuration");
        }

        BaseStage::ID downstreamStage = exitStages.front();
        if(config.multiExit.has_value()) {
            const auto& multiExit = *config.multiExit;
            const auto& decision = *config.decision;
            const auto decisionStage = simulation.AddStage(
                WaypointDescription{decision.position, decision.distance});

            TransitionDescription transition = NonTransitionDescription{};
            switch(multiExit.transitionMode) {
            case ExitTransitionMode::Fixed:
                transition = FixedTransitionDescription(
                    exitStages.at(multiExit.fixedExitIndex));
                break;
            case ExitTransitionMode::RoundRobin: {
                std::vector<std::tuple<BaseStage::ID, uint64_t>> weights{};
                weights.reserve(exitStages.size());
                for(size_t idx = 0; idx < exitStages.size(); ++idx) {
                    weights.emplace_back(exitStages[idx], multiExit.roundRobinWeights[idx]);
                }
                transition = RoundRobinTransitionDescription(weights);
                break;
            }
            case ExitTransitionMode::LeastTargeted:
                transition = LeastTargetedTransitionDescription(exitStages);
                break;
            case ExitTransitionMode::Adaptive:
                transition = AdaptiveTransitionDescription(
                    exitStages,
                    multiExit.expectedTimeWeight,
                    multiExit.densityWeight,
                    multiExit.queueWeight,
                    multiExit.switchPenalty,
                    multiExit.decisionInterval,
                    multiExit.reconsiderationThreshold);
                break;
            }

            journeyStages.emplace(decisionStage, transition);
            downstreamStage = decisionStage;
        }

        BaseStage::ID initialStage = downstreamStage;
        // Shared with the per-exit journeys below, so an assigned agent still
        // walks the stair or ramp before heading to its exit.
        std::optional<BaseStage::ID> transitStage{};
        if(config.stair.has_value()) {
            const auto& stair = *config.stair;
            const auto stairStage = simulation.AddStage(StairDescription{
                .position = stair.position,
                .distance = stair.distance,
                .length = stair.length,
                .ascending = stair.ascending,
                .upSpeedFactor = stair.upSpeedFactor,
                .downSpeedFactor = stair.downSpeedFactor,
                .upSpeed = stair.upSpeed,
                .downSpeed = stair.downSpeed,
                .waitingTime = stair.waitingTime,
                .timeStep = config.dt,
            });
            journeyStages.emplace(stairStage, FixedTransitionDescription(downstreamStage));
            initialStage = stairStage;
            transitStage = stairStage;
        } else if(config.ramp.has_value()) {
            const auto& ramp = *config.ramp;
            const auto rampStage = simulation.AddStage(RampDescription{
                .position = ramp.position,
                .distance = ramp.distance,
                .length = ramp.length,
                .ascending = ramp.ascending,
                .upSpeedFactor = ramp.upSpeedFactor,
                .downSpeedFactor = ramp.downSpeedFactor,
                .waitingTime = ramp.waitingTime,
                .timeStep = config.dt,
            });
            journeyStages.emplace(rampStage, FixedTransitionDescription(downstreamStage));
            initialStage = rampStage;
            transitStage = rampStage;
        }
        const auto journeyId = simulation.AddJourney(journeyStages);

        // Agents with escape_route get their own journey holding just that exit,
        // rather than starting on the shared journey with stageId set to the
        // exit. That matters because the shared journey carries the decision
        // point's AdaptiveTransition, which re-decides for anyone en route to one
        // of its candidates (see Journey::reevaluators) and would pull an
        // assigned agent off its route again. A journey without that transition
        // has nothing to reconsider, so the assignment holds.
        std::map<size_t, std::pair<Journey::ID, BaseStage::ID>> assignedJourneys{};
        const auto journeyForExit =
            [&](size_t exitIndex) -> std::pair<Journey::ID, BaseStage::ID> {
            if(const auto it = assignedJourneys.find(exitIndex);
               it != std::end(assignedJourneys)) {
                return it->second;
            }
            const auto target = exitStages.at(exitIndex);
            std::map<BaseStage::ID, TransitionDescription> stages{};
            stages.emplace(target, NonTransitionDescription{});
            BaseStage::ID start = target;
            if(transitStage.has_value()) {
                // Same stage object as the shared journey; only the transition
                // hanging off it differs, which is per journey anyway.
                stages.emplace(*transitStage, FixedTransitionDescription(target));
                start = *transitStage;
            }
            const auto entry = std::make_pair(simulation.AddJourney(stages), start);
            assignedJourneys.emplace(exitIndex, entry);
            return entry;
        };

        std::vector<AgentProfileEntry> agentProfiles{};
        agentProfiles.reserve(allAgents.size());
        std::unordered_map<uint64_t, double> baseDesiredSpeeds{};
        baseDesiredSpeeds.reserve(allAgents.size());
        // Release time in seconds per agent. Only agents with pre_movement > 0
        // get an entry, so an ordinary scenario leaves this map empty and every
        // addition below is skipped by an empty() check.
        std::unordered_map<uint64_t, double> preMovementTimes{};
        // Last speed factor <fds_hazard> assigned per agent. Only maintained
        // while somebody is still being held back, so a released agent picks up
        // the current smoke damping instead of full speed.
        std::unordered_map<uint64_t, double> fdsSpeedFactors{};

        for(const auto& agentConfig : allAgents) {
            CollisionFreeSpeedModelData modelData{};
            modelData.timeGap = agentConfig.timeGap;
            modelData.v0 = agentConfig.desiredSpeed;
            modelData.radius = agentConfig.radius;
            if(agentConfig.preMovementTime > 0.0) {
                // Born standing, so the agent cannot move even in the very first
                // step. v0 = 0 is explicitly allowed by the model
                // (CollisionFreeSpeedModel.cpp: "constexpr double v0Min = 0.;").
                modelData.v0 = 0.0;
            }

            auto agentJourney = journeyId;
            auto agentStage = initialStage;
            if(agentConfig.escapeRoute > 0) {
                if(agentConfig.escapeRoute > exitStages.size()) {
                    throw std::runtime_error(fmt::format(
                        "escape_route={} exceeds the number of exits ({})",
                        agentConfig.escapeRoute,
                        exitStages.size()));
                }
                std::tie(agentJourney, agentStage) =
                    journeyForExit(agentConfig.escapeRoute - 1);
            }

            GenericAgent agent{
                GenericAgent::ID::Invalid,
                agentJourney,
                agentStage,
                agentConfig.position,
                Point{1.0, 0.0},
                modelData};
            const auto agentId = simulation.AddAgent(std::move(agent)).getID();
            // Deliberately the configured speed, not the held-back v0: the FDS
            // block, the age classification and the .jsp profile record all read
            // this and must stay unaffected.
            baseDesiredSpeeds.emplace(agentId, agentConfig.desiredSpeed);
            if(agentConfig.preMovementTime > 0.0) {
                preMovementTimes.emplace(agentId, agentConfig.preMovementTime);
            }

            uint8_t ageGroupCode = AgeGroupCodeFromString(agentConfig.ageGroup);
            if(ageGroupCode == AGE_GROUP_UNKNOWN) {
                ageGroupCode = ClassifyAgeGroupCode(agentConfig.desiredSpeed);
            }

            uint8_t avatarHintCode = AvatarHintCodeFromString(agentConfig.avatarHint);
            if(avatarHintCode == AVATAR_HINT_UNKNOWN) {
                avatarHintCode = DeriveAvatarHintCode(ageGroupCode, agentId);
            }

            agentProfiles.push_back(
                AgentProfileEntry{
                    .agentId = agentId,
                    .ageGroupCode = ageGroupCode,
                    .avatarHintCode = avatarHintCode,
                    .desiredSpeed = static_cast<float>(agentConfig.desiredSpeed),
                    .timeGap = static_cast<float>(agentConfig.timeGap),
                    .radius = static_cast<float>(agentConfig.radius),
                });
        }

        if(!preMovementTimes.empty() &&
           (config.stair.has_value() || config.ramp.has_value())) {
            // Stair::IsCompleted freezes the traversal budget the first time an
            // agent is inside the stage radius, computing it from v0. An agent
            // held at 0 that already starts inside gets max(0.1, 0.0) and stays
            // slow for good. Starting outside the radius - the normal case - is
            // unaffected, because the budget is only computed on arrival.
            fmt::print(
                stderr,
                "warning: pre_movement together with <stair>/<ramp>: agents that start inside "
                "the stage radius get their traversal time computed from v0 = 0 and are slowed "
                "down permanently\n");
        }

        std::optional<FdsJspMetadata> fdsJspMetadata{};
        if(config.fdsHazard.has_value() && fdsSampleZ.has_value()) {
            fdsJspMetadata =
                MakeFdsJspMetadata(outputPath, *config.fdsHazard, *fdsSampleZ);
        }

        JspTrajectoryWriter writer(
            outputPath,
            config.dt,
            args.everyNthFrame,
            args.compressionLevel,
            std::move(agentProfiles),
            config.sourceModel,
            std::move(fdsJspMetadata));

        writer.WriteFrame(simulation, true);

        // Progress for callers that drive this as a subprocess. Printed rarely
        // and flushed, because stdout is fully buffered when it is a pipe.
        const uint64_t progressEvery =
            std::max<uint64_t>(1, config.maxIterations / 200);

        // Agents that stop making progress are the failure mode that is easy to
        // miss: the run looks finished, the evacuation time is simply wrong.
        // Sample positions every few seconds and report who never moved.
        const uint64_t stallCheckEvery =
            std::max<uint64_t>(1, static_cast<uint64_t>(5.0 / config.dt));
        constexpr double kStallDistance = 0.05;   // metres per sample
        constexpr int kStallSamples = 3;          // ~15 s without progress
        std::unordered_map<uint64_t, Point> lastSampled{};
        std::unordered_map<uint64_t, int> stalledSamples{};

        // Queueing in front of a door is a result, not a defect, so it is
        // counted separately from agents that hang somewhere in the open.
        constexpr double kExitVicinity = 2.0;   // metres
        struct ExitArea {
            double minX{}, minY{}, maxX{}, maxY{};
            size_t maxQueue{0};
            double maxQueueTime{0.0};
            size_t left{0};              // persons that went through this exit
            // Persons already standing inside the polygon when the simulation
            // started. They count towards left so the tally matches the
            // population, but they never traversed the opening, so they are
            // kept out of the flow window.
            size_t presentAtStart{0};
            double firstLeftTime{-1.0};
            double lastLeftTime{0.0};
        };
        std::vector<ExitArea> exitAreas{};
        {
            std::vector<std::vector<Point>> exitPolygons{};
            if(config.exitPolygon.has_value()) {
                exitPolygons.push_back(*config.exitPolygon);
            } else if(config.multiExit.has_value()) {
                exitPolygons = config.multiExit->polygons;
            }
            for(const auto& polygon : exitPolygons) {
                ExitArea area{};
                area.minX = area.minY = std::numeric_limits<double>::max();
                area.maxX = area.maxY = std::numeric_limits<double>::lowest();
                for(const auto& p : polygon) {
                    area.minX = std::min(area.minX, p.x);
                    area.maxX = std::max(area.maxX, p.x);
                    area.minY = std::min(area.minY, p.y);
                    area.maxY = std::max(area.maxY, p.y);
                }
                exitAreas.push_back(area);
            }
        }

        const auto distanceToExit = [](const ExitArea& area, const Point& p) {
            const double dx = std::max({area.minX - p.x, 0.0, p.x - area.maxX});
            const double dy = std::max({area.minY - p.y, 0.0, p.y - area.maxY});
            return std::hypot(dx, dy);
        };

        // Walls for the line-of-sight test. Simulation::Geo() returns by value,
        // so it is fetched once here rather than per agent and per tick, and
        // only when the smoke trigger is actually in use.
        std::optional<CollisionGeometry> sightGeometry{};
        if(config.fdsHazard.has_value() && config.fdsHazard->smokeAlertsBelow > 0.0 &&
           config.fdsHazard->smokeSightRange > 0.0) {
            sightGeometry = simulation.Geo();
        }

        // An agent placed inside an exit polygon is queued for removal already by
        // Simulation::AddAgent, which runs the strategical decision system on the
        // new agent (Simulation.cpp) and lets Exit::IsCompleted push it onto the
        // removal list. The first Iterate() drains and clears that list before the
        // loop below can attribute anyone, so without this pass those persons
        // would silently disappear from the exit tally - the very number a
        // Nachweis quotes. Reachable e.g. when a <distribution> spawn area
        // overlaps an exit; it depends on the seed, which is why it went unnoticed.
        if(!exitAreas.empty() && !simulation.RemovedAgents().empty()) {
            std::unordered_map<uint64_t, Point> spawnPositions{};
            for(const auto& agent : simulation.Agents()) {
                spawnPositions[agent.id.getID()] = agent.pos;
            }
            size_t presentAtStartTotal = 0;
            for(const auto& removed : simulation.RemovedAgents()) {
                const auto it = spawnPositions.find(removed.getID());
                if(it == spawnPositions.end()) {
                    continue;
                }
                size_t nearest = 0;
                double nearestDistance = std::numeric_limits<double>::max();
                for(size_t e = 0; e < exitAreas.size(); ++e) {
                    const double d = distanceToExit(exitAreas[e], it->second);
                    if(d < nearestDistance) {
                        nearestDistance = d;
                        nearest = e;
                    }
                }
                // Counted so the tally matches the population, but deliberately
                // left out of first/lastLeftTime: these persons never crossed the
                // opening, and letting them stretch the window to t=0 would
                // understate the specific flow.
                ++exitAreas[nearest].left;
                ++exitAreas[nearest].presentAtStart;
                ++presentAtStartTotal;
            }
            if(presentAtStartTotal > 0) {
                fmt::print(
                    stderr,
                    "warning: {} agents were already inside an exit when the simulation "
                    "started; they are counted in left= but excluded from the flow window\n",
                    presentAtStartTotal);
            }
        }

        double nextFdsUpdate = 0.0;
        bool warnedOutsideFdsMeshes = false;
        bool warnedAfterFdsData = false;
        while(simulation.AgentCount() > 0 && simulation.Iteration() < config.maxIterations) {
            if(fdsVisibility.has_value() &&
               simulation.ElapsedTime() + std::numeric_limits<double>::epsilon() >=
                   nextFdsUpdate) {
                const auto& fds = *config.fdsHazard;
                // Only worth recording while somebody is still held back.
                // Without pre_movement this stays a single bool test per tick.
                const bool trackFdsFactors = !preMovementTimes.empty();
                std::size_t agentsOutsideMeshes = 0;
                for(auto& agent : simulation.Agents()) {
                    auto& modelData = std::get<CollisionFreeSpeedModelData>(agent.model);
                    const auto baseSpeed = baseDesiredSpeeds.at(agent.id.getID());
                    const auto visibility = fdsVisibility->Sample(
                        simulation.ElapsedTime(),
                        Point{agent.pos.x + fds.offsetX, agent.pos.y + fds.offsetY});
                    if(!visibility.has_value()) {
                        modelData.v0 = baseSpeed;
                        if(trackFdsFactors) {
                            fdsSpeedFactors[agent.id.getID()] = 1.0;
                        }
                        ++agentsOutsideMeshes;
                        continue;
                    }

                    double speedFactor = 1.0;
                    if(*visibility <= fds.severeBelow) {
                        speedFactor = fds.minimumSpeedFactor;
                    } else if(*visibility < fds.awarenessBelow) {
                        const double interpolation =
                            (*visibility - fds.severeBelow) /
                            (fds.awarenessBelow - fds.severeBelow);
                        speedFactor = fds.minimumSpeedFactor +
                                      (1.0 - fds.minimumSpeedFactor) * interpolation;
                    }
                    modelData.v0 = baseSpeed * speedFactor;
                    if(trackFdsFactors) {
                        fdsSpeedFactors[agent.id.getID()] = speedFactor;
                    }

                    // Smoke as a start trigger: somebody who is still waiting
                    // and spots the fire leaves regardless of the clock. Only
                    // ever brings the release forward, never delays it, so the
                    // person goes at whichever comes first - alarm or smoke.
                    if(fds.smokeAlertsBelow > 0.0 && !preMovementTimes.empty()) {
                        const auto waitIt = preMovementTimes.find(agent.id.getID());
                        if(waitIt != preMovementTimes.end() &&
                           NoticesSmoke(
                               fdsWatchField ? *fdsWatchField : *fdsVisibility,
                               sightGeometry ? &*sightGeometry : nullptr,
                               simulation.ElapsedTime(),
                               agent.pos,
                               agent.orientation,
                               Point{fds.offsetX, fds.offsetY},
                               fds.smokeSightRange,
                               fds.smokeSightAngle,
                               fds.smokeAlertsBelow,
                               fds.visibilityFactor,
                               fds.maximumVisibility,
                               fds.smokeViewDimmed)) {
                            const double noticed =
                                simulation.ElapsedTime() + fds.smokeReaction;
                            waitIt->second = std::min(waitIt->second, noticed);
                        }
                    }
                }
                if(agentsOutsideMeshes > 0 && !warnedOutsideFdsMeshes) {
                    fmt::print(
                        stderr,
                        "warning: {} agents are outside the Smoke3D meshes; their base speed is "
                        "used\n",
                        agentsOutsideMeshes);
                    warnedOutsideFdsMeshes = true;
                }
                if(simulation.ElapsedTime() > fdsVisibility->LastTime() &&
                   !warnedAfterFdsData) {
                    fmt::print(
                        stderr,
                        "warning: JuPedSIM time exceeds the last Smoke3D frame; the final smoke "
                        "state is held constant\n");
                    warnedAfterFdsData = true;
                }
                do {
                    nextFdsUpdate += fds.updateInterval;
                } while(nextFdsUpdate <= simulation.ElapsedTime());
            }

            // Pre-movement time. Must run AFTER the FDS block, whose v0 writes
            // would otherwise send a still-waiting agent off for this step, and
            // BEFORE simulation.Iterate(), which reads v0. Nothing in between
            // reads v0; only positions are saved there.
            if(!preMovementTimes.empty()) {
                const double now = simulation.ElapsedTime();
                for(auto& agent : simulation.Agents()) {
                    const auto agentId = agent.id.getID();
                    const auto it = preMovementTimes.find(agentId);
                    if(it == preMovementTimes.end()) {
                        continue; // already released, v0 belongs to the FDS block again
                    }
                    auto& modelData = std::get<CollisionFreeSpeedModelData>(agent.model);
                    // ElapsedTime() is dt * iteration without accumulation drift,
                    // the epsilon only covers the binary representation of dt.
                    if(now + 1e-9 >= it->second) {
                        const auto factorIt = fdsSpeedFactors.find(agentId);
                        const double speedFactor =
                            (factorIt != fdsSpeedFactors.end()) ? factorIt->second : 1.0;
                        modelData.v0 = baseDesiredSpeeds.at(agentId) * speedFactor;
                        // Erase on release: from here on v0 is the FDS block's
                        // business, otherwise this pass would overwrite the smoke
                        // damping between two FDS ticks.
                        preMovementTimes.erase(it);
                    } else {
                        modelData.v0 = 0.0;
                    }
                }
            }

            // Positions of this iteration's agents, so an agent that leaves can
            // be attributed to the exit it was standing at.
            std::unordered_map<uint64_t, Point> positionBeforeStep{};
            if(!exitAreas.empty()) {
                for(const auto& agent : simulation.Agents()) {
                    positionBeforeStep[agent.id.getID()] = agent.pos;
                }
            }

            simulation.Iterate();

            for(const auto& removed : simulation.RemovedAgents()) {
                const auto it = positionBeforeStep.find(removed.getID());
                if(it == positionBeforeStep.end() || exitAreas.empty()) {
                    continue;
                }
                size_t nearest = 0;
                double nearestDistance = std::numeric_limits<double>::max();
                for(size_t e = 0; e < exitAreas.size(); ++e) {
                    const double d = distanceToExit(exitAreas[e], it->second);
                    if(d < nearestDistance) {
                        nearestDistance = d;
                        nearest = e;
                    }
                }
                ExitArea& area = exitAreas[nearest];
                ++area.left;
                if(area.firstLeftTime < 0.0) {
                    area.firstLeftTime = simulation.ElapsedTime();
                }
                area.lastLeftTime = simulation.ElapsedTime();
            }

            writer.WriteFrame(
                simulation,
                simulation.AgentCount() == 0 ||
                    simulation.Iteration() >= config.maxIterations);

            if(simulation.Iteration() % progressEvery == 0) {
                fmt::print(
                    "progress={} of {} agents={}\n",
                    simulation.Iteration(),
                    config.maxIterations,
                    simulation.AgentCount());
                std::fflush(stdout);
            }

            if(simulation.Iteration() % stallCheckEvery == 0) {
                std::vector<size_t> queueSize(exitAreas.size(), 0);

                for(const auto& agent : simulation.Agents()) {
                    const uint64_t id = agent.id.getID();
                    const auto previous = lastSampled.find(id);
                    if(previous != lastSampled.end() &&
                       std::hypot(agent.pos.x - previous->second.x,
                                  agent.pos.y - previous->second.y) < kStallDistance) {
                        ++stalledSamples[id];
                    } else {
                        stalledSamples[id] = 0;
                    }
                    lastSampled[id] = agent.pos;

                    for(size_t e = 0; e < exitAreas.size(); ++e) {
                        if(distanceToExit(exitAreas[e], agent.pos) <= kExitVicinity) {
                            ++queueSize[e];
                        }
                    }
                }

                for(size_t e = 0; e < exitAreas.size(); ++e) {
                    if(queueSize[e] > exitAreas[e].maxQueue) {
                        exitAreas[e].maxQueue = queueSize[e];
                        exitAreas[e].maxQueueTime = simulation.ElapsedTime();
                    }
                }
            }
        }

        for(size_t e = 0; e < exitAreas.size(); ++e) {
            const ExitArea& area = exitAreas[e];
            const double width = std::min(area.maxX - area.minX, area.maxY - area.minY);
            const double span = area.lastLeftTime - std::max(0.0, area.firstLeftTime);
            // Only persons that actually crossed the opening carry information
            // about its capacity; those already standing inside at t=0 are in
            // left= for the population count but must not inflate the flow.
            const size_t traversed = area.left - area.presentAtStart;
            const double flow = (traversed > 1 && span > 1e-9)
                                    ? static_cast<double>(traversed) / span
                                    : 0.0;
            fmt::print(
                "exit={} width={:.2f} m left={} first={:.1f} s last={:.1f} s flow={:.2f} persons/s\n",
                e + 1,
                width,
                area.left,
                std::max(0.0, area.firstLeftTime),
                area.lastLeftTime,
                flow);
            fmt::print(
                "congestion exit={} max_persons_within_{:.0f}m={} at t={:.1f} s\n",
                e + 1,
                kExitVicinity,
                area.maxQueue,
                area.maxQueueTime);
        }

        std::vector<std::pair<uint64_t, Point>> stuck{};
        size_t queueing = 0;
        for(const auto& agent : simulation.Agents()) {
            const auto it = stalledSamples.find(agent.id.getID());
            if(it == stalledSamples.end() || it->second < kStallSamples) {
                continue;
            }
            const bool atExit = std::any_of(
                exitAreas.begin(), exitAreas.end(), [&](const ExitArea& area) {
                    return distanceToExit(area, agent.pos) <= kExitVicinity;
                });
            if(atExit) {
                ++queueing;
            } else {
                stuck.emplace_back(agent.id.getID(), agent.pos);
            }
        }
        if(queueing > 0) {
            fmt::print("queueing_agents={} (waiting within {:.0f} m of an exit)\n",
                       queueing, kExitVicinity);
        }
        if(!stuck.empty()) {
            fmt::print(
                "stuck_agents={} (no progress for {:.0f} s, not at an exit)\n",
                stuck.size(),
                kStallSamples * stallCheckEvery * config.dt);
            const size_t shown = std::min<size_t>(stuck.size(), 10);
            for(size_t i = 0; i < shown; ++i) {
                fmt::print(
                    "stuck id={} at ({:.3f}, {:.3f})\n",
                    stuck[i].first,
                    stuck[i].second.x,
                    stuck[i].second.y);
            }
            if(stuck.size() > shown) {
                fmt::print("stuck ... and {} more\n", stuck.size() - shown);
            }
        }
        writer.Finalize();

        const bool completed = simulation.AgentCount() == 0;
        if(completed) {
            fmt::print("Simulation completed.\n");
        } else {
            fmt::print("Simulation stopped at max_iterations.\n");
        }
        fmt::print(
            "iterations={} elapsed_time={} remaining_agents={}\n",
            simulation.Iteration(),
            simulation.ElapsedTime(),
            simulation.AgentCount());
        fmt::print("jsp_output={}\n", writer.Path().string());

        return completed ? 0 : 2;
    } catch(const std::exception& ex) {
        fmt::print(stderr, "Error: {}\n", ex.what());
        return 1;
    }
}
