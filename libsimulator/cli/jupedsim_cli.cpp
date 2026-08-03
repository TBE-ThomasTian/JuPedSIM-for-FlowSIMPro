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
#include <numeric>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
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
// Optional storey table of a multi-floor run. Indexed by the per-agent floor
// column, which is the storey's rank by elevation - not the <floor id=".."> of
// the scenario, which the exporter allocates freely. The id is carried along so
// a trajectory can still be traced back to the storey it came from.
constexpr char JSP_FLOORS_MAGIC[4] = {'J', 'S', 'P', 'L'};
constexpr uint32_t JSP_FLOORS_VERSION = 1;
constexpr uint32_t JSP_FLOORS_FIXED_PAYLOAD_SIZE = 16; // u32 count + 3x f32 centering
constexpr uint32_t JSP_FLOORS_RECORD_SIZE = 8;         // u32 floor id + f32 elevation
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
    /// still come from the profile the person was drawn from, which is the
    /// default profile only when the distribution has no <profile> children.
    std::optional<PopulationConfig> population{};
};

struct ScenarioConfig {
    double dt{0.01};
    uint64_t maxIterations{10000};
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
    /// One storey of the building, with everything that is geometrically bound
    /// to it. A single-storey scenario - <geometry> directly under <scenario> -
    /// parses into exactly one of these, so both kinds of file run through the
    /// same code below and every scenario written before <floors> existed keeps
    /// its behaviour unchanged.
    struct FloorConfig {
        /// The id as written by the exporter. Deliberately not an index: the
        /// exporter allocates it from the storey band and may skip values, and
        /// <stair from_floor=".."> refers to this number, not to a position.
        int64_t id{0};
        std::string name{};
        /// Height of this storey's floor surface in the source model. The
        /// simulation itself stays two-dimensional; this only tells an upward
        /// stair from a downward one and places the storey in the trajectory.
        double elevation{0.0};
        std::vector<Point> walkable{};
        std::vector<std::vector<Point>> obstacles{};
        std::optional<std::vector<Point>> exitPolygon{};
        std::optional<DecisionConfig> decision{};
        std::optional<MultiExitConfig> multiExit{};
        std::optional<StairConfig> stair{};
        std::optional<RampConfig> ramp{};
        std::vector<AgentConfig> agents{};
        std::optional<AgentDistributionConfig> distribution{};
    };

    /// A stairway between two storeys, from <connections><stair>. The person
    /// walks to `entrance` on `fromFloor` and leaves that storey there; for as
    /// long as the descent takes they belong to no storey at all, and then they
    /// continue at `arrival` on `toFloor`. They are NOT held standing at the
    /// entrance - that would pile every arrival onto one point and let a
    /// stairway discharge one person per descent.
    ///
    /// `width` and `distance` are the clear width and the landing depth of the
    /// way onto the stairway, not the storey height. Together they are the area
    /// in which a person counts as being on the stairway.
    ///
    /// That the landing may be left out of the walked route at all is
    /// DIN 18009-2 Annex E.3: "Die Weglaengen von Treppenpodesten koennen
    /// vernachlaessigt werden, wenn die Podestlaenge in der Laufl inie die
    /// zweifache Breite der Treppe nicht ueberschreitet." Hence the check
    /// distance <= 2 * width - past that the clause no longer covers it, and
    /// the metres saved would be metres the escape route really has.
    ///
    /// Note this is NOT the meaning `distance` has on a single-floor <stair>,
    /// where it is a plain trigger radius around the marker.
    ///
    /// `length` is the way the person actually covers, and the descent takes
    /// length / speed. Which length that is depends on where the speed comes
    /// from, and the two standards differ:
    ///
    ///   - RiMEA Tests 2 and 3 measure the staircase "entlang der Schraege" and
    ///     require the person to cover it in length / speed. Speed is then the
    ///     speed along the slope.
    ///   - DIN 18009-2 F.4.2 says of Tab. F.2, and RiMEA 3.2.2.3 of its Tab. 3,
    ///     that Fruin's figures are Horizontalgeschwindigkeiten. Taking a speed
    ///     from either table therefore means `length` has to be the horizontal
    ///     run, not the slope.
    ///
    /// Left out, `length` follows DIN 18009-2 Annex E, which counts a stairway
    /// in an escape route "mit dem dreifachen der zu ueberwindenden Hoehe".
    struct FloorConnection {
        int64_t fromFloor{0};
        int64_t toFloor{0};
        Point entrance{};
        Point arrival{};
        /// Where the flight ends on the storey it starts from. Given, a person
        /// walking the stair is drawn moving along it instead of vanishing for
        /// the length of the descent - which is what a 3D view needs, and what
        /// stops a third of the population from disappearing at once.
        std::optional<Point> flightEnd{};
        /// Clear width of the way onto the stairway, in metres. Together with
        /// `distance` it is what the opening at `entrance` is made of, so people
        /// step onto the stair where it begins instead of from anywhere along
        /// the shaft.
        ///
        /// 1.20 m is a common clear width for a stair in a building and is only
        /// a stand-in for a scenario that leaves it out; a Nachweis has to state
        /// the real one.
        double width{1.2};
        /// Depth of the landing at `entrance`, in metres - see the note above on
        /// DIN 18009-2 Annex E.3. Defaults to the width, which is the smallest
        /// landing the stair itself can have, and is capped at twice it.
        double distance{1.2};
        std::optional<double> length{};
        double upSpeedFactor{0.6};
        double downSpeedFactor{0.6};
        double upSpeed{0.0};
        double downSpeed{0.0};
        double waitingTime{0.0};
    };

    std::vector<FloorConfig> floors{};
    std::vector<FloorConnection> connections{};
    /// True when the scenario file used <floors>. A one-storey building written
    /// the old way is not the same thing, and the two differ in what is written
    /// to the .jsp: see WriteSourceModelSection.
    bool multiFloor{false};
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
        // Walls are taken into account: a sight line stops at the first one it
        // meets, so smoke behind a wall is not noticed.
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
        "    <!-- A building: one <floor> per storey, each holding exactly what a\n"
        "         one-storey <scenario> holds (geometry, exit(s), agents, an\n"
        "         in-storey stair/ramp). Without <floors> the elements below sit\n"
        "         directly under <scenario>, which is the one-storey form. -->\n"
        "    <floors>\n"
        "      <floor id=\"0\" flowsimpro_name=\"Ground floor\"\n"
        "             flowsimpro_elevation_m=\"0.0\">\n"
        "        <geometry>...</geometry>\n"
        "        <exit>...</exit>\n"
        "        <agents>...</agents>\n"
        "      </floor>\n"
        "      <floor id=\"1\" flowsimpro_elevation_m=\"3.0\">...</floor>\n"
        "    </floors>\n"
        "    <connections>\n"
        "      <!-- A stairway between two storeys. The person walks to\n"
        "           (x, y) on from_floor, is inside the stairway for\n"
        "           length / speed + waiting_time seconds - blocking neither\n"
        "           storey - and continues at (exit_x, exit_y) on to_floor.\n"
        "           An <exit> of from_floor that contains (x, y) is that\n"
        "           stairway's head, not a way out of the building.\n"
        "           width is the clear width of the way onto the stairway and\n"
        "           distance the depth of its landing, both measured at (x, y).\n"
        "           Together they are the area a person is on the stairway in.\n"
        "           DIN 18009-2 Annex E.3 lets a landing be left out of the\n"
        "           escape route length while it is no longer than twice the\n"
        "           stair width, so distance may not exceed 2 * width.\n"
        "           end_x/end_y are the far end of the flight. Given, somebody\n"
        "           walking the stairway is drawn moving along it instead of\n"
        "           being absent from every frame for the whole descent.\n"
        "           length is the way actually walked. RiMEA Tests 2/3 measure\n"
        "           it along the slope; the Fruin speeds of DIN 18009-2\n"
        "           Tab. F.2 and RiMEA Tab. 3 are horizontal speeds, so with\n"
        "           those it is the horizontal run. Left out, it follows\n"
        "           DIN 18009-2 Annex E: three times the height overcome.\n"
        "           speed_factor multiplies the person's own speed and sets\n"
        "           both directions; up_/down_ override per direction, and\n"
        "           up_speed/down_speed are absolute m/s and win over factors.\n"
        "           Up or down is read from flowsimpro_elevation_m, so both\n"
        "           directions of a shaft may be listed without sending anyone\n"
        "           the wrong way: people walk towards a storey that has an\n"
        "           <exit>, over as few stairways as possible.\n"
        "           Density on the stairway itself is not modelled - RiMEA\n"
        "           Test 13 needs the staircase drawn as walkable geometry. -->\n"
        "      <stair from_floor=\"1\" to_floor=\"0\" x=\"24\" y=\"7\"\n"
        "             exit_x=\"24\" exit_y=\"7\" width=\"1.20\" distance=\"2.40\"\n"
        "             end_x=\"24\" end_y=\"11\" length=\"9.0\"\n"
        "             speed_factor=\"0.6\" waiting_time=\"0.0\"/>\n"
        "    </connections>\n"
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
        "             escape_route=\"1\"  <!-- 1-based index into the ways OUT of\n"
        "                                   the building on this storey, in the\n"
        "                                   order <exits> lists them. An <exit>\n"
        "                                   that holds a stairway entrance is a\n"
        "                                   stair head and is not counted. Omit\n"
        "                                   to let the person choose per\n"
        "                                   <exits mode=...> -->/>\n"
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
/// and they see a layer building up overhead. So this casts eight sight lines
/// across the view cone and marches each out to sightRange, accumulating the
/// optical depth along it. A line counts as smoke once it has swallowed
/// `dimmedFraction` of the light. The person's own position is a separate test,
/// compared against alertsBelow directly.
///
/// Walls ARE modelled: each line is tested segment by segment against the
/// geometry and stops at the first one it meets, so smoke in the room next door
/// is not noticed. Only a call that is handed no geometry sees through walls.
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

/// Reads everything that belongs to one storey: its geometry, its way out, an
/// optional stair or ramp inside the storey, and its occupants.
///
/// The same node layout is accepted under <scenario> and under
/// <floors><floor>, so `ctx` carries the path for error messages - it is the
/// only thing that differs between a one-storey file and one storey of a
/// building.
void ParseFloorNode(
    const pt::ptree& floorNode,
    const std::string& ctx,
    ScenarioConfig::FloorConfig& floor)
{
    const auto geometryNodeOpt = floorNode.get_child_optional("geometry");
    if(!geometryNodeOpt) {
        throw std::runtime_error("Missing node " + ctx + ".geometry");
    }
    const auto& geometryNode = *geometryNodeOpt;
    floor.walkable =
        ParsePolygon(geometryNode.get_child("walkable"), ctx + ".geometry.walkable");
    for(const auto& [tag, node] : geometryNode) {
        if(tag == "obstacle") {
            floor.obstacles.push_back(ParsePolygon(node, ctx + ".geometry.obstacle"));
        }
    }

    if(const auto singleExitNode = floorNode.get_child_optional("exit"); singleExitNode) {
        floor.exitPolygon = ParsePolygon(*singleExitNode, ctx + ".exit");
    }

    if(const auto decisionNodeOpt = floorNode.get_child_optional("decision"); decisionNodeOpt) {
        ScenarioConfig::DecisionConfig decision{};
        const auto& decisionNode = *decisionNodeOpt;
        decision.position = ParsePoint(decisionNode, ctx + ".decision");
        decision.distance =
            decisionNode.get<double>("<xmlattr>.distance", decision.distance);
        if(decision.distance <= 0.0) {
            throw std::runtime_error(ctx + ".decision.distance must be > 0");
        }
        floor.decision = decision;
    }

    if(const auto exitsNodeOpt = floorNode.get_child_optional("exits"); exitsNodeOpt) {
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
                ctx + ".exits weights/penalty must be >= 0");
        }
        if(multiExit.decisionInterval == 0) {
            throw std::runtime_error(ctx + ".exits.decision_interval must be > 0");
        }
        if(multiExit.reconsiderationThreshold < 0.0) {
            throw std::runtime_error(
                ctx + ".exits.reconsideration_threshold must be >= 0");
        }

        for(const auto& [tag, node] : exitsNode) {
            if(tag != "exit") {
                continue;
            }
            multiExit.polygons.push_back(ParsePolygon(node, ctx + ".exits.exit"));
            const auto weight = node.get<uint64_t>("<xmlattr>.weight", 1);
            if(weight == 0) {
                throw std::runtime_error(ctx + ".exits.exit.weight must be > 0");
            }
            multiExit.roundRobinWeights.push_back(weight);
        }

        if(multiExit.polygons.size() < 2) {
            throw std::runtime_error(ctx + ".exits requires at least 2 <exit/> entries");
        }
        if(multiExit.fixedExitIndex >= multiExit.polygons.size()) {
            throw std::runtime_error(
                ctx + ".exits.fixed_index out of range");
        }

        floor.multiExit = multiExit;
    }

    if(floor.exitPolygon.has_value() && floor.multiExit.has_value()) {
        throw std::runtime_error("Use either <exit> or <exits>, not both");
    }
    // Whether a storey needs a way out of its own is decided by the caller: in
    // a building it may well have none, because its way out is a stairway to
    // the storey below. That the stairway actually leads somewhere is checked
    // once the connections are known.
    if(floor.multiExit.has_value() && !floor.decision.has_value()) {
        throw std::runtime_error(ctx + ".exits requires a <decision .../> element");
    }
    if(floor.decision.has_value() && !floor.multiExit.has_value()) {
        throw std::runtime_error("<decision> is only valid together with <exits>");
    }

    if(const auto stairNodeOpt = floorNode.get_child_optional("stair"); stairNodeOpt) {
        ScenarioConfig::StairConfig stair{};
        const auto& stairNode = *stairNodeOpt;
        stair.position = ParsePoint(stairNode, ctx + ".stair");
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
            throw std::runtime_error(ctx + ".stair.distance must be > 0");
        }
        if(stair.length < 0.0) {
            throw std::runtime_error(ctx + ".stair.length must be >= 0");
        }
        if(stair.upSpeedFactor <= 0.0 || stair.downSpeedFactor <= 0.0) {
            throw std::runtime_error(
                ctx + ".stair speed factors must be > 0 "
                      "(speed_factor, up_speed_factor, down_speed_factor)");
        }
        if(stair.upSpeed < 0.0 || stair.downSpeed < 0.0) {
            throw std::runtime_error(
                ctx + ".stair.up_speed / down_speed must be >= 0 (0 = use the factor)");
        }
        if(stair.waitingTime < 0.0) {
            throw std::runtime_error(ctx + ".stair.waiting_time must be >= 0");
        }
        floor.stair = stair;
    }
    if(const auto rampNodeOpt = floorNode.get_child_optional("ramp"); rampNodeOpt) {
        ScenarioConfig::RampConfig ramp{};
        const auto& rampNode = *rampNodeOpt;
        ramp.position = ParsePoint(rampNode, ctx + ".ramp");
        ramp.distance = rampNode.get<double>("<xmlattr>.distance", ramp.distance);
        ramp.length = rampNode.get<double>("<xmlattr>.length", ramp.length);
        ramp.upSpeedFactor =
            rampNode.get<double>("<xmlattr>.up_speed_factor", ramp.upSpeedFactor);
        ramp.downSpeedFactor =
            rampNode.get<double>("<xmlattr>.down_speed_factor", ramp.downSpeedFactor);
        ramp.waitingTime =
            rampNode.get<double>("<xmlattr>.waiting_time", ramp.waitingTime);
        if(const auto asc = rampNode.get_optional<std::string>("<xmlattr>.ascending"); asc) {
            ramp.ascending = ParseBool(*asc, ctx + ".ramp.ascending");
        }

        if(ramp.distance <= 0.0) {
            throw std::runtime_error(ctx + ".ramp.distance must be > 0");
        }
        if(ramp.length < 0.0) {
            throw std::runtime_error(ctx + ".ramp.length must be >= 0");
        }
        if(ramp.upSpeedFactor <= 0.0) {
            throw std::runtime_error(ctx + ".ramp.up_speed_factor must be > 0");
        }
        if(ramp.downSpeedFactor <= 0.0) {
            throw std::runtime_error(ctx + ".ramp.down_speed_factor must be > 0");
        }
        if(ramp.waitingTime < 0.0) {
            throw std::runtime_error(ctx + ".ramp.waiting_time must be >= 0");
        }
        floor.ramp = ramp;
    }

    if(floor.stair.has_value() && floor.ramp.has_value()) {
        throw std::runtime_error(
            "Use either <stair> or <ramp>, not both, in " + ctx);
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
            // See ParseSpawnProfile: signed so a negative index is rejected
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
        floor.agents.push_back(agent);
    };

    std::vector<pt::ptree> detachedProfiles{};
    if(const auto agentsNodeOpt = floorNode.get_child_optional("agents"); agentsNodeOpt) {
        const auto& agentsNode = *agentsNodeOpt;
        for(const auto& [tag, node] : agentsNode) {
            if(tag == "agent") {
                parseExplicitAgent(node, ctx + ".agents.agent");
            } else if(tag == "distribution") {
                if(floor.distribution.has_value()) {
                    throw std::runtime_error(
                        "Distribution defined multiple times in " + ctx + ".agents");
                }
                floor.distribution = ParseDistributionConfig(node, ctx + ".agents.distribution");
            } else if(tag == "profile") {
                detachedProfiles.push_back(node);
            }
        }
    }

    if(const auto topDistribution = floorNode.get_child_optional("agent_distribution"); topDistribution) {
        if(floor.distribution.has_value()) {
            throw std::runtime_error(
                ctx + ": distribution defined both in .agents and .agent_distribution");
        }
        floor.distribution = ParseDistributionConfig(
            *topDistribution,
            ctx + ".agent_distribution");
    }

    if(!detachedProfiles.empty()) {
        if(!floor.distribution.has_value()) {
            throw std::runtime_error(
                "Found <agents><profile/> but no <distribution/> in " + ctx);
        }
        for(const auto& profileNode : detachedProfiles) {
            floor.distribution->profiles.push_back(
                ParseSpawnProfile(
                    profileNode,
                    ctx + ".agents.profile",
                    floor.distribution->defaultProfile));
        }
    }

    // Whether an empty storey is an error is decided by the caller: a building
    // may well have one that nobody is on, and the exporter writes an empty
    // <agents> for every storey without manually placed persons as soon as one
    // storey has them.
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

    const auto floorsNodeOpt = scenario.get_child_optional("floors");
    if(!floorsNodeOpt) {
        // One storey, written the way every scenario was written before
        // <floors> existed. Parsed into a single floor so that everything
        // below - stages, journeys, the trajectory writer - has exactly one
        // code path to maintain.
        ScenarioConfig::FloorConfig floor{};
        if(config.sourceModel.has_value()) {
            floor.elevation = config.sourceModel->storeyElevation;
        }
        ParseFloorNode(scenario, "scenario", floor);
        if(floor.agents.empty() && !floor.distribution.has_value()) {
            throw std::runtime_error(
                "scenario.agents must contain at least one <agent/> or one <distribution/>");
        }
        if(!floor.exitPolygon.has_value() && !floor.multiExit.has_value()) {
            throw std::runtime_error("Missing <exit> or <exits> in scenario");
        }
        // A stairway joins two storeys, and this file has one. Everything below
        // reads <connections> only after <floors>, so accepting it here would
        // mean running as if the stairways written down did not exist and
        // reporting an evacuation time for a building that was never simulated.
        if(scenario.get_child_optional("connections")) {
            throw std::runtime_error(
                "scenario.connections needs <floors>: a <stair> joins two storeys, "
                "and a scenario without <floors> has one");
        }
        config.floors.push_back(std::move(floor));
        return config;
    }

    config.multiFloor = true;
    std::set<int64_t> seenFloorIds{};
    for(const auto& [tag, floorNode] : *floorsNodeOpt) {
        // Same reasoning as in the <connections> loop below: a comment or the
        // attribute pseudo-node is not an element, but anything else that is not
        // a <floor> would be a whole storey dropped without a word.
        if(tag == "<xmlcomment>" || tag == "<xmlattr>") {
            continue;
        }
        if(tag != "floor") {
            throw std::runtime_error(
                "scenario.floors accepts only <floor> elements, found <" + tag + ">");
        }
        ScenarioConfig::FloorConfig floor{};
        floor.id = RequiredValue<int64_t>(
            floorNode, "<xmlattr>.id", "scenario.floors.floor.id");
        if(!seenFloorIds.insert(floor.id).second) {
            throw std::runtime_error(fmt::format(
                "scenario.floors: floor id {} appears twice", floor.id));
        }
        floor.name = floorNode.get<std::string>("<xmlattr>.flowsimpro_name", std::string{});
        floor.elevation =
            floorNode.get<double>("<xmlattr>.flowsimpro_elevation_m", 0.0);
        if(!std::isfinite(floor.elevation)) {
            throw std::runtime_error(fmt::format(
                "scenario.floors.floor[id={}].flowsimpro_elevation_m must be finite",
                floor.id));
        }
        ParseFloorNode(
            floorNode,
            fmt::format("scenario.floors.floor[id={}]", floor.id),
            floor);
        config.floors.push_back(std::move(floor));
    }
    if(config.floors.empty()) {
        throw std::runtime_error("scenario.floors contains no <floor> element");
    }
    if(std::none_of(
           config.floors.begin(), config.floors.end(), [](const auto& floor) {
               return !floor.agents.empty() || floor.distribution.has_value();
           })) {
        throw std::runtime_error(
            "scenario.floors: no storey holds a single <agent/> or <distribution/>");
    }

    if(const auto connectionsOpt = scenario.get_child_optional("connections");
       connectionsOpt) {
        for(const auto& [tag, stairNode] : *connectionsOpt) {
            // <xmlcomment> and <xmlattr> are how Boost surfaces comments and
            // attributes as children; only a real unknown element is an error,
            // because silently ignoring one would drop a storey connection.
            if(tag == "<xmlcomment>" || tag == "<xmlattr>") {
                continue;
            }
            if(tag != "stair") {
                throw std::runtime_error(
                    "scenario.connections accepts only <stair/> elements, found <" +
                    tag + ">");
            }
            ScenarioConfig::FloorConnection link{};
            link.fromFloor = RequiredValue<int64_t>(
                stairNode, "<xmlattr>.from_floor", "scenario.connections.stair.from_floor");
            link.toFloor = RequiredValue<int64_t>(
                stairNode, "<xmlattr>.to_floor", "scenario.connections.stair.to_floor");
            const std::string what = fmt::format(
                "scenario.connections.stair[{} -> {}]", link.fromFloor, link.toFloor);
            if(link.fromFloor == link.toFloor) {
                throw std::runtime_error(what + " connects a floor to itself");
            }
            if(seenFloorIds.count(link.fromFloor) == 0) {
                throw std::runtime_error(
                    what + ": from_floor names no <floor id=\"...\">");
            }
            if(seenFloorIds.count(link.toFloor) == 0) {
                throw std::runtime_error(
                    what + ": to_floor names no <floor id=\"...\">");
            }
            link.entrance = ParsePoint(stairNode, what);
            link.arrival = Point{
                RequiredValue<double>(stairNode, "<xmlattr>.exit_x", what + ".exit_x"),
                RequiredValue<double>(stairNode, "<xmlattr>.exit_y", what + ".exit_y")};
            link.width = stairNode.get<double>("<xmlattr>.width", link.width);
            {
                const auto endX = stairNode.get_optional<double>("<xmlattr>.end_x");
                const auto endY = stairNode.get_optional<double>("<xmlattr>.end_y");
                if(endX.has_value() != endY.has_value()) {
                    throw std::runtime_error(
                        what + ": end_x and end_y have to be given together");
                }
                if(endX) {
                    link.flightEnd = Point{*endX, *endY};
                }
            }
            // Left out, the landing is as deep as the stair is wide - the
            // smallest one the flight itself can have. Taking the struct default
            // instead would make a narrow stair fail its own 2 * width check.
            link.distance =
                stairNode.get<double>("<xmlattr>.distance", link.width);
            if(const auto length = stairNode.get_optional<double>("<xmlattr>.length");
               length) {
                link.length = *length;
            }
            // speed_factor covers both directions, exactly as on a single-floor
            // <stair>; the per-direction attributes exist so a hand-written
            // scenario can follow DIN 18009-2 Tab. F.2, which distinguishes
            // them. The exporter writes speed_factor and, where the case states
            // them, the absolute up_speed/down_speed; only the per-direction
            // factors stay hand-written.
            const double bothFactors =
                stairNode.get<double>("<xmlattr>.speed_factor", link.upSpeedFactor);
            link.upSpeedFactor =
                stairNode.get<double>("<xmlattr>.up_speed_factor", bothFactors);
            link.downSpeedFactor =
                stairNode.get<double>("<xmlattr>.down_speed_factor", bothFactors);
            link.upSpeed = stairNode.get<double>("<xmlattr>.up_speed", link.upSpeed);
            link.downSpeed = stairNode.get<double>("<xmlattr>.down_speed", link.downSpeed);
            link.waitingTime =
                stairNode.get<double>("<xmlattr>.waiting_time", link.waitingTime);

            if(link.width <= 0.0) {
                throw std::runtime_error(what + ".width must be > 0");
            }
            if(link.distance <= 0.0) {
                throw std::runtime_error(what + ".distance must be > 0");
            }
            if(link.distance > 2.0 * link.width) {
                throw std::runtime_error(fmt::format(
                    "{}: distance={:.2f} m is the depth of the landing and may not "
                    "exceed twice the stair width ({:.2f} m at width={:.2f} m). "
                    "DIN 18009-2 Annex E.3 allows a landing to be left out of the "
                    "escape route length only up to that; beyond it the metres are "
                    "part of the route and have to be walked.",
                    what,
                    link.distance,
                    2.0 * link.width,
                    link.width));
            }
            if(link.length.has_value() && *link.length < 0.0) {
                throw std::runtime_error(what + ".length must be >= 0");
            }
            if(link.upSpeedFactor <= 0.0 || link.downSpeedFactor <= 0.0) {
                throw std::runtime_error(what + " speed factors must be > 0");
            }
            if(link.upSpeed < 0.0 || link.downSpeed < 0.0) {
                throw std::runtime_error(
                    what + ".up_speed / down_speed must be >= 0 (0 = use the factor)");
            }
            if(link.waitingTime < 0.0) {
                throw std::runtime_error(what + ".waiting_time must be >= 0");
            }
            config.connections.push_back(link);
        }
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
        std::optional<FdsJspMetadata> fdsMetadata,
        std::vector<std::pair<int64_t, double>> floorElevations)
        : _path(std::move(path))
        , _everyNthFrame(everyNthFrame)
        , _compressionLevel(compressionLevel)
        , _agentProfiles(std::move(agentProfiles))
        , _sourceModel(sourceModel)
        , _fdsMetadata(std::move(fdsMetadata))
        , _floorElevations(std::move(floorElevations))
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

    /// One frame across the whole building. Every storey contributes the agents
    /// standing on it; the floor column tells them apart, which is what lets a
    /// reader place a person at the right height.
    ///
    /// The storeys are stepped in lockstep, so any of them reports the same
    /// iteration and the same elapsed time - SimulationClock::ElapsedTime() is
    /// dT * iteration, not an accumulated sum.
    /// A person on a stairway, placed along it. Handed in separately because
    /// they belong to no Simulation while they are on it.
    struct TransitRecord {
        uint64_t agentId{};
        Point position{};
        Point orientation{};
        uint32_t floorColumn{};
    };

    void WriteFrame(
        const std::vector<std::pair<Simulation*, uint32_t>>& floors,
        const std::vector<TransitRecord>& onStairways = {},
        bool force = false)
    {
        const auto iteration = floors.front().first->Iteration();
        if(_lastWrittenIteration.has_value() && *_lastWrittenIteration == iteration) {
            return;
        }
        if(!force && (iteration % _everyNthFrame != 0)) {
            return;
        }

        std::vector<uint8_t> uncompressed{};
        size_t agentCount = onStairways.size();
        for(const auto& [simulation, floorIndex] : floors) {
            agentCount += simulation->Agents().size();
        }
        uncompressed.reserve(agentCount * JSP_RECORD_SIZE);
        for(const auto& [simulation, floorIndex] : floors) {
            for(const auto& agent : simulation->Agents()) {
                AppendU64LE(uncompressed, agent.id.getID());
                AppendF32LE(uncompressed, static_cast<float>(agent.pos.x));
                AppendF32LE(uncompressed, static_cast<float>(agent.pos.y));
                AppendF32LE(uncompressed, static_cast<float>(agent.orientation.x));
                AppendF32LE(uncompressed, static_cast<float>(agent.orientation.y));
                AppendU32LE(uncompressed, floorIndex);
            }
        }
        for(const auto& transit : onStairways) {
            AppendU64LE(uncompressed, transit.agentId);
            AppendF32LE(uncompressed, static_cast<float>(transit.position.x));
            AppendF32LE(uncompressed, static_cast<float>(transit.position.y));
            AppendF32LE(uncompressed, static_cast<float>(transit.orientation.x));
            AppendF32LE(uncompressed, static_cast<float>(transit.orientation.y));
            AppendU32LE(uncompressed, transit.floorColumn);
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
                .timeSeconds = floors.front().first->ElapsedTime(),
                .agentCount = static_cast<uint32_t>(agentCount),
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
        // Last on purpose. A reader that does not know a trailer magic stops
        // scanning there rather than skipping the block, so anything new has to
        // sit behind everything the current readers do know.
        WriteFloorTableSection();

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

    /// Where the storey sits in the source model, for a reader that has only
    /// the .jsp and not the scenario .xml next to it.
    ///
    /// Deliberately not written for a building: this block carries exactly one
    /// elevation, and FlowSIMPro treats its mere presence as "the scenario
    /// knows the height", after which it places floor n at that one elevation
    /// plus n times a guessed storey spacing
    /// (EvacuationManager::floorZForId). For a building that guess would
    /// override the real storey heights it already has from the model, so the
    /// block is left out and the true elevations go into JSPL instead.
    void WriteSourceModelSection()
    {
        if(!_sourceModel.has_value() || !_floorElevations.empty()) {
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

    /// The building's storeys, in the order the floor column indexes them: the
    /// real elevation of each, plus the centering shift that JSPF carries for a
    /// single storey. Written only for a <floors> scenario.
    ///
    /// This exists because the floor column alone says which storey somebody is
    /// on, not how high that storey is, and the one elevation JSPF holds cannot
    /// describe a building whose storeys are not evenly spaced. It carries a
    /// payload size so a reader can skip it; that the readers of today stop at
    /// an unknown magic instead is why it is written last.
    void WriteFloorTableSection()
    {
        if(_floorElevations.empty()) {
            return;
        }

        const auto count = static_cast<uint32_t>(_floorElevations.size());
        WriteAll(_out, JSP_FLOORS_MAGIC, sizeof(JSP_FLOORS_MAGIC));
        WriteU32LE(_out, JSP_FLOORS_VERSION);
        WriteU32LE(_out, JSP_FLOORS_FIXED_PAYLOAD_SIZE + count * JSP_FLOORS_RECORD_SIZE);
        WriteU32LE(_out, count);
        WriteF32LE(_out, static_cast<float>(_sourceModel ? _sourceModel->centeringX : 0.0));
        WriteF32LE(_out, static_cast<float>(_sourceModel ? _sourceModel->centeringY : 0.0));
        WriteF32LE(_out, static_cast<float>(_sourceModel ? _sourceModel->centeringZ : 0.0));
        for(const auto& [floorId, elevation] : _floorElevations) {
            WriteU32LE(_out, static_cast<uint32_t>(floorId));
            WriteF32LE(_out, static_cast<float>(elevation));
        }
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
    /// {<floor id> from the scenario, elevation}, in floor-column order. Empty
    /// for a single-storey run, which is what tells the two apart here.
    std::vector<std::pair<int64_t, double>> _floorElevations{};
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

        // Smoke is sampled on one horizontal plane, derived from the storey
        // elevation. A building needs one plane per storey; a single field
        // would quietly apply the ground floor's smoke to every storey, which
        // is worse than saying so.
        if(config.multiFloor && config.fdsHazard.has_value()) {
            throw std::runtime_error(
                "<fds_hazard> together with <floors> is not supported: the visibility "
                "field is sampled on one plane and can only be right for one storey");
        }

        const size_t floorCount = config.floors.size();

        // The per-agent floor column of the .jsp is the storey's rank by
        // elevation, not its <floor id="..">: a reader turns that column into a
        // height, and only the ordering carries that meaning. The exporter
        // allocates ids from the storey band and may skip values.
        std::vector<size_t> byElevation(floorCount);
        std::iota(byElevation.begin(), byElevation.end(), size_t{0});
        std::stable_sort(
            byElevation.begin(),
            byElevation.end(),
            [&](size_t lhs, size_t rhs) {
                return config.floors[lhs].elevation < config.floors[rhs].elevation;
            });
        std::vector<uint32_t> floorColumn(floorCount, 0);
        for(size_t rank = 0; rank < floorCount; ++rank) {
            floorColumn[byElevation[rank]] = static_cast<uint32_t>(rank);
        }
        std::unordered_map<int64_t, size_t> floorIndexById{};
        for(size_t f = 0; f < floorCount; ++f) {
            floorIndexById.emplace(config.floors[f].id, f);
        }

        constexpr size_t kLeavesBuilding = std::numeric_limits<size_t>::max();

        // The way each stairway is actually walked. Only known here, because it
        // depends on the two storey elevations.
        //
        // Where the scenario does not say, DIN 18009-2 Annex E supplies it: a
        // stairway in an escape route counts "mit dem dreifachen der zu
        // ueberwindenden Hoehe". A given length is used as it stands - it is the
        // scenario author who knows whether it is the slope or, for a speed out
        // of DIN Tab. F.2 / RiMEA Tab. 3, the horizontal run.
        std::vector<double> connectionLength(config.connections.size(), 0.0);
        for(size_t c = 0; c < config.connections.size(); ++c) {
            const auto& link = config.connections[c];
            const double rise = std::abs(
                config.floors[floorIndexById.at(link.toFloor)].elevation -
                config.floors[floorIndexById.at(link.fromFloor)].elevation);
            connectionLength[c] = link.length.value_or(3.0 * rise);
            if(connectionLength[c] + 1e-9 < rise) {
                throw std::runtime_error(fmt::format(
                    "scenario.connections.stair[{} -> {}]: length {:.3f} m is shorter than "
                    "the {:.3f} m it has to descend; no stairway can be",
                    link.fromFloor,
                    link.toFloor,
                    connectionLength[c],
                    rise));
            }
            if(connectionLength[c] <= 0.0) {
                throw std::runtime_error(fmt::format(
                    "scenario.connections.stair[{} -> {}]: length is 0 m and the two storeys "
                    "are at the same elevation, so there is nothing to walk",
                    link.fromFloor,
                    link.toFloor));
            }
        }

        struct FloorRuntime {
            std::unique_ptr<Simulation> simulation{};
            std::vector<BaseStage::ID> exitStages{};
            std::vector<std::vector<Point>> exitPolygons{};
            /// Per entry of exitStages: the <connections><stair> an agent takes
            /// on reaching that opening, or kLeavesBuilding when the opening is
            /// a way out of the building.
            std::vector<size_t> connectionForExit{};
            /// Indices into exitStages that agents of this storey are routed to.
            std::vector<size_t> routedTargets{};
            std::optional<BaseStage::ID> transitStage{};
            std::optional<Journey::ID> sharedJourney{};
            std::optional<BaseStage::ID> sharedInitialStage{};
            /// Journey per routed target, for the storeys where each agent is
            /// sent to its nearest stairway rather than offered a choice.
            std::map<size_t, std::pair<Journey::ID, BaseStage::ID>> targetJourneys{};
        };
        std::vector<FloorRuntime> floors(floorCount);

        // Geometry first: GenerateDistributedAgents needs the built geometry,
        // and the Simulation takes ownership of it.
        std::vector<std::vector<AgentConfig>> floorAgents(floorCount);
        size_t populationTotal = 0;
        for(size_t f = 0; f < floorCount; ++f) {
            const auto& floorConfig = config.floors[f];
            const std::string what =
                config.multiFloor
                    ? fmt::format("floor id={} ({})", floorConfig.id,
                                  floorConfig.name.empty() ? "unnamed" : floorConfig.name)
                    : std::string("scenario");

            GeometryBuilder geometryBuilder{};
            geometryBuilder.AddAccessibleArea(floorConfig.walkable);
            for(const auto& obstacle : floorConfig.obstacles) {
                geometryBuilder.ExcludeFromAccessibleArea(obstacle);
            }
            std::unique_ptr<CollisionGeometry> geometry{};
            try {
                geometry = std::make_unique<CollisionGeometry>(geometryBuilder.Build());
            } catch(const std::exception& error) {
                throw std::runtime_error(
                    fmt::format("{}: {}", what, error.what()));
            }

            auto& agents = floorAgents[f];
            agents = floorConfig.agents;
            if(floorConfig.distribution.has_value()) {
                auto generated =
                    GenerateDistributedAgents(*floorConfig.distribution, *geometry, agents);
                agents.insert(
                    agents.end(),
                    std::make_move_iterator(generated.begin()),
                    std::make_move_iterator(generated.end()));
            }
            populationTotal += agents.size();

            auto model = std::make_unique<CollisionFreeSpeedModel>(
                config.strengthNeighborRepulsion,
                config.rangeNeighborRepulsion,
                config.strengthGeometryRepulsion,
                config.rangeGeometryRepulsion);
            floors[f].simulation = std::make_unique<Simulation>(
                std::move(model), std::move(geometry), config.dt);
        }
        if(populationTotal == 0) {
            throw std::runtime_error("No agents available after parsing/generation");
        }

        // Exit openings of every storey, before it is known which of them lead
        // out of the building.
        for(size_t f = 0; f < floorCount; ++f) {
            const auto& floorConfig = config.floors[f];
            auto& runtime = floors[f];
            if(floorConfig.exitPolygon.has_value()) {
                runtime.exitPolygons.push_back(*floorConfig.exitPolygon);
            } else if(floorConfig.multiExit.has_value()) {
                runtime.exitPolygons = floorConfig.multiExit->polygons;
            }
            runtime.connectionForExit.assign(runtime.exitPolygons.size(), kLeavesBuilding);
        }

        // An opening drawn no deeper than a person is wide cannot be walked
        // into. Exit::IsCompleted tests the CENTRE of the agent against the
        // polygon (Stage.cpp), while the geometry repulsion keeps that centre a
        // radius away from the wall behind it - so for a strip drawn flush with
        // the wall and exactly one radius deep, the removal line and the closest
        // reachable line are the same line. People then only get out while the
        // crowd behind them pushes hard enough, and the last of them never do.
        //
        // It is not a stalled-agent problem but a flow problem, and a silent
        // one: a 1.0 m door drawn 0.20 m deep carried 0.48 persons/(m s) where
        // 0.30 m deep carries 2.22 - the evacuation looks five times slower
        // without anything failing. Deepening it inwards is therefore a repair,
        // not a liberty, and it is done here rather than in the exporting
        // application because only the solver knows where the obstacles are.
        //
        // Deliberately conservative: only an axis-aligned rectangle is touched,
        // only along its short axis, only into space that is actually walkable,
        // and never beyond what is needed. Anything else is left alone and
        // reported, because a wrong exit is worse than a shallow one.
        const auto boundsOf = [](const std::vector<Point>& polygon) {
            BoundingBox box{
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest()};
            for(const auto& p : polygon) {
                box.minX = std::min(box.minX, p.x);
                box.maxX = std::max(box.maxX, p.x);
                box.minY = std::min(box.minY, p.y);
                box.maxY = std::max(box.maxY, p.y);
            }
            return box;
        };
        for(size_t f = 0; f < floorCount; ++f) {
            double maxRadius = 0.0;
            for(const auto& agent : floorAgents[f]) {
                maxRadius = std::max(maxRadius, agent.radius);
            }
            if(maxRadius <= 0.0 || floors[f].exitPolygons.empty()) {
                continue;
            }
            const double wanted = 2.0 * maxRadius;
            const auto geometry = floors[f].simulation->Geo();
            const std::string what = config.multiFloor
                                         ? fmt::format(" on floor id={}", config.floors[f].id)
                                         : std::string{};

            for(size_t e = 0; e < floors[f].exitPolygons.size(); ++e) {
                auto& polygon = floors[f].exitPolygons[e];
                if(polygon.size() != 4) {
                    continue;
                }
                const auto box = boundsOf(polygon);
                const double width = box.maxX - box.minX;
                const double height = box.maxY - box.minY;
                // Axis-aligned means every vertex sits on the bounding box.
                const bool axisAligned = std::all_of(
                    polygon.begin(), polygon.end(), [&](const Point& p) {
                        return (std::abs(p.x - box.minX) < 1e-9 ||
                                std::abs(p.x - box.maxX) < 1e-9) &&
                               (std::abs(p.y - box.minY) < 1e-9 ||
                                std::abs(p.y - box.maxY) < 1e-9);
                    });
                const double depth = std::min(width, height);
                if(depth >= wanted) {
                    continue;
                }
                if(!axisAligned) {
                    fmt::print(
                        stderr,
                        "warning: exit {}{} is only {:.2f} m deep, less than the {:.2f} m a "
                        "person of radius {:.2f} m needs to step into it, and is not an "
                        "axis-aligned rectangle, so it was left as it is; people will only "
                        "leave through it while they are being pushed from behind\n",
                        e + 1, what, depth, wanted, maxRadius);
                    continue;
                }

                const bool alongY = height < width;
                const double grow = wanted - depth;
                // Try both ways and keep the one that stays walkable. The far
                // side is the wall the opening sits in, so normally exactly one
                // of the two works.
                bool deepened = false;
                for(const int direction : {-1, 1}) {
                    auto candidate = polygon;
                    const double lo = alongY ? box.minY : box.minX;
                    const double hi = alongY ? box.maxY : box.maxX;
                    const double newLo = (direction < 0) ? lo - grow : lo;
                    const double newHi = (direction < 0) ? hi : hi + grow;
                    for(auto& p : candidate) {
                        double& coordinate = alongY ? p.y : p.x;
                        coordinate = (std::abs(coordinate - lo) < 1e-9) ? newLo : newHi;
                    }
                    // The whole added strip has to be walkable, not just its
                    // centre: AddStage only checks the centroid, so a half-buried
                    // exit would pass validation and silently narrow the opening.
                    const auto probe = boundsOf(candidate);
                    bool clear = true;
                    for(int i = 0; i <= 8 && clear; ++i) {
                        const double along = probe.minX +
                                             (probe.maxX - probe.minX) * i / 8.0;
                        for(int j = 0; j <= 8 && clear; ++j) {
                            const double across = probe.minY +
                                                  (probe.maxY - probe.minY) * j / 8.0;
                            clear = geometry.InsideGeometry(Point{along, across});
                        }
                    }
                    if(!clear) {
                        continue;
                    }
                    polygon = candidate;
                    deepened = true;
                    fmt::print(
                        stderr,
                        "note: exit {}{} was {:.2f} m deep, too shallow for a person of "
                        "radius {:.2f} m to step into; deepened inwards to {:.2f} m\n",
                        e + 1, what, depth, maxRadius, wanted);
                    break;
                }
                if(!deepened) {
                    fmt::print(
                        stderr,
                        "warning: exit {}{} is only {:.2f} m deep and there is no walkable "
                        "space to deepen it into; people will only leave through it while "
                        "they are being pushed from behind, and the last of them will not "
                        "get out at all\n",
                        e + 1, what, depth);
                }
            }
        }

        // A stairway starts somewhere on its lower storey, and the exporter has
        // no attribute saying whether the exit drawn on that storey is a way
        // out of the building or the head of that stairway. It writes at most
        // one exit per floor, and FlowSIMPro's own upper storeys carry the
        // stair head as a stand-in exit. So the openings are told apart by
        // where the stairway begins: an exit polygon that contains the entrance
        // of an outgoing stair is that stair's head, not a way out.
        //
        // Getting this wrong is not a subtle error - every person would "leave
        // the building" on their own storey within seconds and <connections>
        // would never be used.
        //
        // Such a polygon is then dropped rather than reused as the way into the
        // stairway. It is the outline of the whole shaft, several metres across,
        // and using it would let people step onto the stair from any side and
        // through walls that are not drawn. A stairway is entered where it
        // begins, so the way in is a `width` wide opening at the entrance point
        // the scenario names - the point the user clicked when placing it.
        std::vector<std::vector<size_t>> outgoing(floorCount);
        std::vector<std::vector<bool>> isStairHead(floorCount);
        for(size_t f = 0; f < floorCount; ++f) {
            isStairHead[f].assign(floors[f].exitPolygons.size(), false);
        }
        for(size_t c = 0; c < config.connections.size(); ++c) {
            const auto& link = config.connections[c];
            const size_t from = floorIndexById.at(link.fromFloor);
            outgoing[from].push_back(c);

            auto& runtime = floors[from];
            for(size_t e = 0; e < runtime.exitPolygons.size(); ++e) {
                if(Polygon(runtime.exitPolygons[e]).IsInside(link.entrance)) {
                    isStairHead[from][e] = true;
                    break;
                }
            }
        }

        for(size_t f = 0; f < floorCount; ++f) {
            auto& runtime = floors[f];
            std::vector<std::vector<Point>> keptPolygons{};
            std::vector<uint64_t> keptWeights{};
            const bool hasWeights =
                config.floors[f].multiExit.has_value() &&
                config.floors[f].multiExit->roundRobinWeights.size() ==
                    runtime.exitPolygons.size();
            // fixed_index counts the <exit> elements as the scenario wrote them,
            // so dropping a stair head from the middle of that list shifts every
            // index behind it. Translated here rather than at the point of use,
            // where the original numbering is no longer visible.
            const bool isFixed =
                config.floors[f].multiExit.has_value() &&
                config.floors[f].multiExit->transitionMode == ExitTransitionMode::Fixed;
            const size_t namedExit =
                isFixed ? config.floors[f].multiExit->fixedExitIndex : 0;
            std::optional<size_t> keptNamedExit{};
            size_t dropped = 0;
            for(size_t e = 0; e < runtime.exitPolygons.size(); ++e) {
                if(isStairHead[f][e]) {
                    if(isFixed && e == namedExit) {
                        throw std::runtime_error(fmt::format(
                            "floor id={}: fixed_index={} names an <exit> that holds a "
                            "stairway entrance, which is a way to the next storey and "
                            "not a way out of the building",
                            config.floors[f].id,
                            namedExit));
                    }
                    ++dropped;
                    continue;
                }
                if(isFixed && e == namedExit) {
                    keptNamedExit = keptPolygons.size();
                }
                keptPolygons.push_back(runtime.exitPolygons[e]);
                if(hasWeights) {
                    keptWeights.push_back(config.floors[f].multiExit->roundRobinWeights[e]);
                }
            }
            if(dropped > 0 && hasWeights) {
                config.floors[f].multiExit->roundRobinWeights = keptWeights;
            }
            if(isFixed && keptNamedExit.has_value()) {
                config.floors[f].multiExit->fixedExitIndex = *keptNamedExit;
            }
            runtime.exitPolygons = std::move(keptPolygons);
            runtime.connectionForExit.assign(runtime.exitPolygons.size(), kLeavesBuilding);

            // The way onto each stairway that starts here: the landing, centred
            // on the entrance. `width` across so it can be walked into from the
            // storey, `distance` deep so somebody standing on it is inside it -
            // Exit::IsCompleted tests the centre of the person.
            //
            // Where the flight is known the depth runs along it, which is the
            // "Lauflinie" DIN 18009-2 Annex E.3 measures the landing in. Without
            // end_x/end_y there is no direction to lay it out along, and the
            // landing degenerates to a square that is the larger of the two -
            // the only shape that cannot point the wrong way.
            for(const size_t c : outgoing[f]) {
                const auto& link = config.connections[c];
                const double halfWidth = 0.5 * link.width;
                const double halfDepth = 0.5 * link.distance;
                Point along{0.0, 0.0};
                if(link.flightEnd.has_value()) {
                    const Point flight = *link.flightEnd - link.entrance;
                    if(flight.Norm() > 1e-9) {
                        along = flight.Normalized();
                    }
                }
                if(along == Point{0.0, 0.0}) {
                    const double half = std::max(halfWidth, halfDepth);
                    runtime.exitPolygons.push_back(std::vector<Point>{
                        Point{link.entrance.x - half, link.entrance.y - half},
                        Point{link.entrance.x + half, link.entrance.y - half},
                        Point{link.entrance.x + half, link.entrance.y + half},
                        Point{link.entrance.x - half, link.entrance.y + half}});
                } else {
                    const Point across{-along.y, along.x};
                    const Point depth = along * halfDepth;
                    const Point side = across * halfWidth;
                    runtime.exitPolygons.push_back(std::vector<Point>{
                        link.entrance - depth - side,
                        link.entrance + depth - side,
                        link.entrance + depth + side,
                        link.entrance - depth + side});
                }
                runtime.connectionForExit.push_back(c);
            }
        }

        // Which storeys can be left, and by which stairway. The search starts
        // at every storey that has a way out and walks the stairways backwards,
        // so a person is never sent up a stair that leads away from an exit -
        // FlowSIMPro's automatic stair placement writes both directions of
        // every shaft, and taking the wrong one would be a trap.
        std::vector<bool> hasBuildingExit(floorCount, false);
        for(size_t f = 0; f < floorCount; ++f) {
            for(size_t e = 0; e < floors[f].connectionForExit.size(); ++e) {
                if(floors[f].connectionForExit[e] == kLeavesBuilding) {
                    hasBuildingExit[f] = true;
                }
            }
        }
        constexpr size_t kUnreachable = std::numeric_limits<size_t>::max();
        std::vector<size_t> stairsToExit(floorCount, kUnreachable);
        std::vector<size_t> queue{};
        for(size_t f = 0; f < floorCount; ++f) {
            if(hasBuildingExit[f]) {
                stairsToExit[f] = 0;
                queue.push_back(f);
            }
        }
        for(size_t head = 0; head < queue.size(); ++head) {
            const size_t to = queue[head];
            for(const auto& link : config.connections) {
                if(floorIndexById.at(link.toFloor) != to) {
                    continue;
                }
                const size_t from = floorIndexById.at(link.fromFloor);
                if(stairsToExit[from] != kUnreachable) {
                    continue;
                }
                stairsToExit[from] = stairsToExit[to] + 1;
                queue.push_back(from);
            }
        }
        for(size_t f = 0; f < floorCount; ++f) {
            if(stairsToExit[f] == kUnreachable && !floorAgents[f].empty()) {
                throw std::runtime_error(fmt::format(
                    "floor id={} holds {} persons but no <exit> and no <connections><stair> "
                    "leading to a storey that has one",
                    config.floors[f].id,
                    floorAgents[f].size()));
            }
        }

        // Where the people of each storey are sent. A storey with a way out
        // uses it, exactly as a single-storey scenario does. A storey without
        // one sends its people to the stairways that get closest to an exit;
        // where several qualify, each person takes the nearest.
        for(size_t f = 0; f < floorCount; ++f) {
            auto& runtime = floors[f];
            if(hasBuildingExit[f]) {
                for(size_t e = 0; e < runtime.connectionForExit.size(); ++e) {
                    if(runtime.connectionForExit[e] == kLeavesBuilding) {
                        runtime.routedTargets.push_back(e);
                    }
                }
                continue;
            }
            size_t best = kUnreachable;
            for(size_t e = 0; e < runtime.connectionForExit.size(); ++e) {
                const size_t c = runtime.connectionForExit[e];
                if(c == kLeavesBuilding) {
                    continue;
                }
                const size_t to = floorIndexById.at(config.connections[c].toFloor);
                best = std::min(best, stairsToExit[to]);
            }
            for(size_t e = 0; e < runtime.connectionForExit.size(); ++e) {
                const size_t c = runtime.connectionForExit[e];
                if(c == kLeavesBuilding) {
                    continue;
                }
                const size_t to = floorIndexById.at(config.connections[c].toFloor);
                if(stairsToExit[to] == best) {
                    runtime.routedTargets.push_back(e);
                }
            }
        }

        // How long somebody is inside a stairway, in iterations. The formula is
        // Stair::IsCompleted's (Stage.cpp): an absolute speed per DIN 18009-2
        // Tab. F.2 wins over the factor, and the factor is applied to the
        // person's own walking speed.
        //
        // Unlike an in-storey <stair>, this time is not spent standing at the
        // opening: a stairway between two storeys is a room of its own, and a
        // person walking down it neither blocks the storey above nor the one
        // below. Holding them at the entrance instead would pile every arrival
        // onto a single point - and where a shaft runs through several storeys
        // the arrival point IS the next entrance, so the stairway would
        // discharge one person per descent.
        const auto descentSpeed =
            [&](const ScenarioConfig::FloorConnection& link, bool ascending, double v0) {
                const double absoluteSpeed = ascending ? link.upSpeed : link.downSpeed;
                const double speedFactor =
                    ascending ? link.upSpeedFactor : link.downSpeedFactor;
                return absoluteSpeed > 0.0 ? absoluteSpeed : std::max(0.1, v0 * speedFactor);
            };
        const auto traversalIterations = [&](size_t c, bool ascending, double v0) {
            const auto& link = config.connections[c];
            const double traversalTime =
                (connectionLength[c] / descentSpeed(link, ascending, v0)) + link.waitingTime;
            if(traversalTime <= 0.0) {
                return uint64_t{0};
            }
            return static_cast<uint64_t>(std::ceil(traversalTime / config.dt));
        };

        // Stages and journeys, storey by storey.
        for(size_t f = 0; f < floorCount; ++f) {
            const auto& floorConfig = config.floors[f];
            auto& runtime = floors[f];
            auto& simulation = *runtime.simulation;
            const std::string what = fmt::format("floor id={}", floorConfig.id);

            for(const auto& polygon : runtime.exitPolygons) {
                runtime.exitStages.push_back(
                    simulation.AddStage(ExitDescription{Polygon(polygon)}));
            }

            // An in-storey <stair>/<ramp> stays what it was: a stage walked
            // before heading on, here in front of whatever the storey routes to.
            if(floorConfig.stair.has_value()) {
                const auto& stair = *floorConfig.stair;
                runtime.transitStage = simulation.AddStage(StairDescription{
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
            } else if(floorConfig.ramp.has_value()) {
                const auto& ramp = *floorConfig.ramp;
                runtime.transitStage = simulation.AddStage(RampDescription{
                    .position = ramp.position,
                    .distance = ramp.distance,
                    .length = ramp.length,
                    .ascending = ramp.ascending,
                    .upSpeedFactor = ramp.upSpeedFactor,
                    .downSpeedFactor = ramp.downSpeedFactor,
                    .waitingTime = ramp.waitingTime,
                    .timeStep = config.dt,
                });
            }

            // Reaching target `e` means reaching its opening - whether that
            // opening leads out of the building or into a stairway.
            const auto chainTo =
                [&](size_t e, std::map<BaseStage::ID, TransitionDescription>& stages) {
                    const auto exitStage = runtime.exitStages.at(e);
                    stages.emplace(exitStage, NonTransitionDescription{});
                    return exitStage;
                };

            std::map<BaseStage::ID, TransitionDescription> journeyStages{};
            // <exits> chooses between the ways out this storey lists. A storey
            // without one is not choosing between them at all - its targets are
            // the stairways appended above, which carry no weight and no index
            // because the scenario never named them. Sending those through the
            // <exits> machinery read roundRobinWeights past its end and made
            // fixed_index select a stairway the file never pointed at.
            if(runtime.routedTargets.size() == 1 || !floorConfig.multiExit.has_value() ||
               !hasBuildingExit[f]) {
                // One way on, or several stairways among which each person is
                // given the nearest rather than a choice: one journey per
                // target, assigned when the agent is created.
                for(const size_t e : runtime.routedTargets) {
                    std::map<BaseStage::ID, TransitionDescription> stages{};
                    BaseStage::ID start = chainTo(e, stages);
                    if(runtime.transitStage.has_value()) {
                        stages.emplace(
                            *runtime.transitStage, FixedTransitionDescription(start));
                        start = *runtime.transitStage;
                    }
                    runtime.targetJourneys.emplace(
                        e, std::make_pair(simulation.AddJourney(stages), start));
                }
                // A storey with nothing to walk to - no way out, no stairway -
                // gets no journey. That is allowed as long as nobody is on it
                // and nobody is sent to it; both are checked above.
                if(!runtime.targetJourneys.empty()) {
                    const auto first = runtime.targetJourneys.begin()->second;
                    runtime.sharedJourney = first.first;
                    runtime.sharedInitialStage = first.second;
                }
            } else {
                // Several ways out with a <decision> point: unchanged from a
                // single-storey scenario, including the adaptive choice.
                std::vector<BaseStage::ID> candidates{};
                for(const size_t e : runtime.routedTargets) {
                    candidates.push_back(chainTo(e, journeyStages));
                }
                const auto& multiExit = *floorConfig.multiExit;
                if(!floorConfig.decision.has_value()) {
                    throw std::runtime_error(
                        what + ": <exits> requires a <decision .../> element");
                }
                const auto& decision = *floorConfig.decision;
                const auto decisionStage = simulation.AddStage(
                    WaypointDescription{decision.position, decision.distance});

                TransitionDescription transition = NonTransitionDescription{};
                switch(multiExit.transitionMode) {
                case ExitTransitionMode::Fixed:
                    transition =
                        FixedTransitionDescription(candidates.at(multiExit.fixedExitIndex));
                    break;
                case ExitTransitionMode::RoundRobin: {
                    std::vector<std::tuple<BaseStage::ID, uint64_t>> weights{};
                    weights.reserve(candidates.size());
                    for(size_t idx = 0; idx < candidates.size(); ++idx) {
                        weights.emplace_back(
                            candidates[idx], multiExit.roundRobinWeights[idx]);
                    }
                    transition = RoundRobinTransitionDescription(weights);
                    break;
                }
                case ExitTransitionMode::LeastTargeted:
                    transition = LeastTargetedTransitionDescription(candidates);
                    break;
                case ExitTransitionMode::Adaptive:
                    transition = AdaptiveTransitionDescription(
                        candidates,
                        multiExit.expectedTimeWeight,
                        multiExit.densityWeight,
                        multiExit.queueWeight,
                        multiExit.switchPenalty,
                        multiExit.decisionInterval,
                        multiExit.reconsiderationThreshold);
                    break;
                }
                journeyStages.emplace(decisionStage, transition);
                BaseStage::ID start = decisionStage;
                if(runtime.transitStage.has_value()) {
                    journeyStages.emplace(
                        *runtime.transitStage, FixedTransitionDescription(decisionStage));
                    start = *runtime.transitStage;
                }
                runtime.sharedJourney = simulation.AddJourney(journeyStages);
                runtime.sharedInitialStage = start;

                // Agents with escape_route get a journey holding just that exit,
                // rather than starting on the shared journey with stageId set to
                // the exit. That matters because the shared journey carries the
                // decision point's AdaptiveTransition, which re-decides for
                // anyone en route to one of its candidates (see
                // Journey::reevaluators) and would pull an assigned agent off
                // its route again. A journey without that transition has nothing
                // to reconsider, so the assignment holds.
                for(const size_t e : runtime.routedTargets) {
                    std::map<BaseStage::ID, TransitionDescription> stages{};
                    BaseStage::ID start2 = chainTo(e, stages);
                    if(runtime.transitStage.has_value()) {
                        stages.emplace(
                            *runtime.transitStage, FixedTransitionDescription(start2));
                        start2 = *runtime.transitStage;
                    }
                    runtime.targetJourneys.emplace(
                        e, std::make_pair(simulation.AddJourney(stages), start2));
                }
            }
        }

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
        const auto boxOf = [](const std::vector<Point>& polygon) {
            ExitArea area{};
            area.minX = area.minY = std::numeric_limits<double>::max();
            area.maxX = area.maxY = std::numeric_limits<double>::lowest();
            for(const auto& p : polygon) {
                area.minX = std::min(area.minX, p.x);
                area.maxX = std::max(area.maxX, p.x);
                area.minY = std::min(area.minY, p.y);
                area.maxY = std::max(area.maxY, p.y);
            }
            return area;
        };
        const auto distanceToExit = [](const ExitArea& area, const Point& p) {
            const double dx = std::max({area.minX - p.x, 0.0, p.x - area.maxX});
            const double dy = std::max({area.minY - p.y, 0.0, p.y - area.maxY});
            return std::hypot(dx, dy);
        };
        std::vector<std::vector<ExitArea>> exitAreas(floorCount);
        for(size_t f = 0; f < floorCount; ++f) {
            for(const auto& polygon : floors[f].exitPolygons) {
                exitAreas[f].push_back(boxOf(polygon));
            }
        }

        // The journey towards whichever of a storey's targets lies nearest to a
        // given point, or the storey's shared journey when there is nothing to
        // choose between.
        //
        // Both the people created on a storey and the people arriving on it from
        // a stairway have to go through here. Handing the arrivals the shared
        // journey instead - which is targetJourneys.begin(), the lowest exit
        // index - sends everybody coming down any shaft across the storey into
        // the first shaft the scenario happens to list, empties the others, and
        // makes the evacuation time depend on the order the exporter wrote the
        // <stair> elements in.
        const auto journeyNearest =
            [&](size_t f, const Point& from) -> std::pair<Journey::ID, BaseStage::ID> {
            const auto& runtime = floors[f];
            if(runtime.routedTargets.size() > 1 && !config.floors[f].multiExit.has_value()) {
                size_t nearest = runtime.routedTargets.front();
                double nearestDistance = std::numeric_limits<double>::max();
                for(const size_t e : runtime.routedTargets) {
                    const double d = distanceToExit(exitAreas[f][e], from);
                    if(d < nearestDistance) {
                        nearestDistance = d;
                        nearest = e;
                    }
                }
                return runtime.targetJourneys.at(nearest);
            }
            return {*runtime.sharedJourney, *runtime.sharedInitialStage};
        };

        std::vector<AgentProfileEntry> agentProfiles{};
        agentProfiles.reserve(populationTotal);
        std::unordered_map<uint64_t, double> baseDesiredSpeeds{};
        baseDesiredSpeeds.reserve(populationTotal);
        // Release time in seconds per agent. Only agents with pre_movement > 0
        // get an entry, so an ordinary scenario leaves this map empty and every
        // addition below is skipped by an empty() check.
        std::unordered_map<uint64_t, double> preMovementTimes{};
        // Last speed factor <fds_hazard> assigned per agent. Only maintained
        // while somebody is still being held back, so a released agent picks up
        // the current smoke damping instead of full speed.
        std::unordered_map<uint64_t, double> fdsSpeedFactors{};
        bool anyTransitStage = false;

        for(size_t f = 0; f < floorCount; ++f) {
            auto& runtime = floors[f];
            auto& simulation = *runtime.simulation;
            anyTransitStage = anyTransitStage || runtime.transitStage.has_value();
            // escape_route counts the ways OUT of the building, in the order the
            // scenario lists them. A stair head is skipped: it is an <exit>
            // element to the parser, but assigning somebody to it would not be
            // assigning them an escape route.
            std::vector<size_t> buildingExitIndices{};
            for(size_t e = 0; e < runtime.connectionForExit.size(); ++e) {
                if(runtime.connectionForExit[e] == kLeavesBuilding) {
                    buildingExitIndices.push_back(e);
                }
            }

            for(const auto& agentConfig : floorAgents[f]) {
                CollisionFreeSpeedModelData modelData{};
                modelData.timeGap = agentConfig.timeGap;
                modelData.v0 = agentConfig.desiredSpeed;
                modelData.radius = agentConfig.radius;
                if(agentConfig.preMovementTime > 0.0) {
                    // Born standing, so the agent cannot move even in the very
                    // first step. v0 = 0 is explicitly allowed by the model
                    // (CollisionFreeSpeedModel.cpp: "constexpr double v0Min = 0.;").
                    modelData.v0 = 0.0;
                }

                auto agentJourney = *runtime.sharedJourney;
                auto agentStage = *runtime.sharedInitialStage;
                if(agentConfig.escapeRoute > 0) {
                    if(agentConfig.escapeRoute > buildingExitIndices.size()) {
                        throw std::runtime_error(fmt::format(
                            "escape_route={} exceeds the number of exits ({}) on floor id={}",
                            agentConfig.escapeRoute,
                            buildingExitIndices.size(),
                            config.floors[f].id));
                    }
                    std::tie(agentJourney, agentStage) = runtime.targetJourneys.at(
                        buildingExitIndices[agentConfig.escapeRoute - 1]);
                } else {
                    // Several stairways and no <decision> to choose between
                    // them: everybody takes the one nearest to where they are.
                    std::tie(agentJourney, agentStage) =
                        journeyNearest(f, agentConfig.position);
                }

                GenericAgent agent{
                    GenericAgent::ID::Invalid,
                    agentJourney,
                    agentStage,
                    agentConfig.position,
                    Point{1.0, 0.0},
                    modelData};
                const auto agentId = simulation.AddAgent(std::move(agent)).getID();
                // Deliberately the configured speed, not the held-back v0: the
                // FDS block, the age classification and the .jsp profile record
                // all read this and must stay unaffected.
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
        }

        if(!preMovementTimes.empty() && anyTransitStage) {
            // Stair::IsCompleted freezes the traversal budget the first time an
            // agent is inside the stage radius, computing it from v0. An agent
            // held at 0 that already starts inside gets max(0.1, 0.0) and stays
            // slow for good. Starting outside the radius - the normal case - is
            // unaffected, because the budget is only computed on arrival.
            //
            // Only an in-storey <stair>/<ramp> can do this, so <connections> is
            // deliberately not a trigger: a stairway between storeys is timed by
            // traversalIterations from the person's configured speed, not from
            // the held-back v0, and warning about it sent people looking for a
            // fault that a multi-storey scenario cannot have.
            fmt::print(
                stderr,
                "warning: pre_movement together with <stair>/<ramp>: agents that start inside "
                "the stage radius get their traversal time computed from v0 = 0 and are slowed "
                "down permanently\n");
        }

        if(config.multiFloor) {
            for(size_t rank = 0; rank < floorCount; ++rank) {
                const size_t f = byElevation[rank];
                const auto& floorConfig = config.floors[f];
                size_t heads = 0;
                for(const size_t assigned : floors[f].connectionForExit) {
                    heads += (assigned != kLeavesBuilding) ? 1 : 0;
                }
                fmt::print(
                    "floor column={} id={} name=\"{}\" elevation={:.3f} m persons={} "
                    "building_exits={} stair_heads={} stairs_to_exit={}\n",
                    rank,
                    floorConfig.id,
                    floorConfig.name,
                    floorConfig.elevation,
                    floorAgents[f].size(),
                    floors[f].connectionForExit.size() - heads,
                    heads,
                    stairsToExit[f]);
            }
            // What each stairway does, in the terms a Nachweis has to state:
            // the way walked, the speed it is walked at and the resulting
            // time. RiMEA 3.2.2.3 asks for the reduction on stairs to be
            // documented with its source, and RiMEA Tests 2 and 3 check exactly
            // this length-over-speed relation.
            for(size_t c = 0; c < config.connections.size(); ++c) {
                const auto& link = config.connections[c];
                const double fromZ =
                    config.floors[floorIndexById.at(link.fromFloor)].elevation;
                const double toZ = config.floors[floorIndexById.at(link.toFloor)].elevation;
                const bool ascending = toZ > fromZ;
                // Reported for a person walking 1.0 m/s on the level, so the
                // figure is comparable between scenarios; each person descends
                // at their own speed.
                const double speed = descentSpeed(link, ascending, 1.0);
                fmt::print(
                    "stair from={} to={} {} rise={:.2f} m entrance=({:.3f}, {:.3f}) "
                    "width={:.2f} m arrival=({:.3f}, {:.3f}) length={:.2f} m{} "
                    "speed={:.2f} m/s{} wait={:.2f} s time={:.1f} s\n",
                    link.fromFloor,
                    link.toFloor,
                    ascending ? "up" : "down",
                    std::abs(toZ - fromZ),
                    link.entrance.x,
                    link.entrance.y,
                    link.width,
                    link.arrival.x,
                    link.arrival.y,
                    connectionLength[c],
                    link.length.has_value() ? "" : " (DIN 18009-2 Annex E: 3x rise)",
                    speed,
                    (ascending ? link.upSpeed : link.downSpeed) > 0.0
                        ? " (absolute)"
                        : " (at v0 = 1.0 m/s)",
                    link.waitingTime,
                    connectionLength[c] / speed + link.waitingTime);
            }
            std::fflush(stdout);
        }

        std::optional<FdsJspMetadata> fdsJspMetadata{};
        if(config.fdsHazard.has_value() && fdsSampleZ.has_value()) {
            fdsJspMetadata =
                MakeFdsJspMetadata(outputPath, *config.fdsHazard, *fdsSampleZ);
        }

        std::vector<std::pair<int64_t, double>> floorElevations{};
        if(config.multiFloor) {
            for(size_t rank = 0; rank < floorCount; ++rank) {
                const auto& floorConfig = config.floors[byElevation[rank]];
                floorElevations.emplace_back(floorConfig.id, floorConfig.elevation);
            }
        }

        JspTrajectoryWriter writer(
            outputPath,
            config.dt,
            args.everyNthFrame,
            args.compressionLevel,
            std::move(agentProfiles),
            config.sourceModel,
            std::move(fdsJspMetadata),
            std::move(floorElevations));

        // In floor-column order, so the records of a frame come out grouped by
        // storey rather than by the order the file happened to list them in.
        std::vector<std::pair<Simulation*, uint32_t>> frameFloors{};
        for(size_t rank = 0; rank < floorCount; ++rank) {
            const size_t f = byElevation[rank];
            frameFloors.emplace_back(floors[f].simulation.get(), floorColumn[f]);
        }

        const auto totalAgentCount = [&]() {
            size_t total = 0;
            for(const auto& runtime : floors) {
                total += runtime.simulation->AgentCount();
            }
            return total;
        };

        size_t presentAtStartTotal = 0;
        writer.WriteFrame(frameFloors, {}, true);

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

        // How far from the foot of a stairway somebody may step to find room.
        // Three rings of shoulder width is about a metre - the landing itself,
        // not a licence to appear across the room.
        constexpr size_t kLandingRings = 3;

        /// Somebody who is inside a stairway: no longer on the storey they left,
        /// not yet on the one they are going to. They are set down once the
        /// descent time has passed, and later still if the arrival point is
        /// occupied at that moment - which is how a crowded landing throttles
        /// the stairway above it.
        ///
        /// While in here a person is written to a frame only when the stairway
        /// has a drawn flight (end_x/end_y): they are then placed along it, on
        /// the storey they departed from. Without a flight there is no line to
        /// walk along, and they appear in no frame until they are set down.
        struct PendingTransfer {
            GenericAgent agent;
            size_t connection{};
            size_t targetFloor{};
            uint64_t releaseIteration{};
            /// Where the descent started and how long it takes, so the person
            /// can be placed along the flight while it lasts.
            uint64_t startedAt{};
            uint32_t departureColumn{};
        };
        std::vector<PendingTransfer> pending{};
        size_t transfersDone = 0;
        uint64_t longestLandingWait = 0;
        size_t peakInStairways = 0;

        // Walls for the line-of-sight test. Simulation::Geo() returns by value,
        // so it is fetched once here rather than per agent and per tick, and
        // only when the smoke trigger is actually in use.
        std::optional<CollisionGeometry> sightGeometry{};
        if(config.fdsHazard.has_value() && config.fdsHazard->smokeAlertsBelow > 0.0 &&
           config.fdsHazard->smokeSightRange > 0.0) {
            sightGeometry = floors.front().simulation->Geo();
        }

        double nextFdsUpdate = 0.0;
        bool warnedOutsideFdsMeshes = false;
        bool warnedAfterFdsData = false;
        Simulation& clock = *floors.front().simulation;
        while((totalAgentCount() > 0 || !pending.empty()) &&
              clock.Iteration() < config.maxIterations) {
            // Set down everybody whose descent is over. Done here, before
            // anyone is moved, so that a person leaves one storey's frame and
            // enters the other's without ever appearing in both.
            if(!pending.empty()) {
                peakInStairways = std::max(peakInStairways, pending.size());
                // The foot of a stairway serves one person at a time. Once it
                // is found blocked, everybody else queued behind on that same
                // stairway waits too - which is both what a staircase does and
                // what keeps this from retrying every waiting person against
                // every free spot on every iteration.
                std::set<size_t> blockedFoot{};
                std::vector<PendingTransfer> stillWalking{};
                for(auto& transfer : pending) {
                    if(clock.Iteration() < transfer.releaseIteration ||
                       blockedFoot.count(transfer.connection) > 0) {
                        stillWalking.push_back(std::move(transfer));
                        continue;
                    }
                    // Somebody stepping off a stair does not have to land on
                    // one exact point - they step aside. Without that, the
                    // whole stairway would discharge only while a single
                    // coordinate happens to be free, and a busy landing would
                    // hold up the storeys above for minutes at a time.
                    //
                    // AddAgent rejects a position that overlaps another person
                    // or falls outside the walkable area, so trying the spots
                    // in turn also keeps arrivals out of the walls.
                    bool placed = false;
                    const double step = 2.0 * std::get<CollisionFreeSpeedModelData>(
                                                  transfer.agent.model)
                                                  .radius;
                    for(size_t ring = 0; ring <= kLandingRings && !placed; ++ring) {
                        const size_t spots = (ring == 0) ? 1 : 6 * ring;
                        for(size_t spot = 0; spot < spots && !placed; ++spot) {
                            const double angle =
                                2.0 * std::numbers::pi * static_cast<double>(spot) /
                                static_cast<double>(spots);
                            const double radius = step * static_cast<double>(ring);
                            GenericAgent candidate = transfer.agent;
                            candidate.pos = Point{
                                transfer.agent.pos.x + radius * std::cos(angle),
                                transfer.agent.pos.y + radius * std::sin(angle)};
                            try {
                                floors[transfer.targetFloor].simulation->AddAgent(
                                    std::move(candidate));
                                placed = true;
                            } catch(const std::exception&) {
                                // Occupied or outside the storey; try the next.
                            }
                        }
                    }
                    if(placed) {
                        ++transfersDone;
                        longestLandingWait = std::max(
                            longestLandingWait,
                            clock.Iteration() - transfer.releaseIteration);
                    } else {
                        // The landing is full. Wait on the bottom step and try
                        // again next iteration.
                        blockedFoot.insert(transfer.connection);
                        stillWalking.push_back(std::move(transfer));
                    }
                }
                pending = std::move(stillWalking);
            }

            if(fdsVisibility.has_value() &&
               clock.ElapsedTime() + std::numeric_limits<double>::epsilon() >=
                   nextFdsUpdate) {
                const auto& fds = *config.fdsHazard;
                // Only worth recording while somebody is still held back.
                // Without pre_movement this stays a single bool test per tick.
                const bool trackFdsFactors = !preMovementTimes.empty();
                std::size_t agentsOutsideMeshes = 0;
                for(auto& runtime : floors) {
                    for(auto& agent : runtime.simulation->Agents()) {
                    auto& modelData = std::get<CollisionFreeSpeedModelData>(agent.model);
                    const auto baseSpeed = baseDesiredSpeeds.at(agent.id.getID());
                    const auto visibility = fdsVisibility->Sample(
                        clock.ElapsedTime(),
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
                               clock.ElapsedTime(),
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
                                clock.ElapsedTime() + fds.smokeReaction;
                            waitIt->second = std::min(waitIt->second, noticed);
                        }
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
                if(clock.ElapsedTime() > fdsVisibility->LastTime() &&
                   !warnedAfterFdsData) {
                    fmt::print(
                        stderr,
                        "warning: JuPedSIM time exceeds the last Smoke3D frame; the final smoke "
                        "state is held constant\n");
                    warnedAfterFdsData = true;
                }
                do {
                    nextFdsUpdate += fds.updateInterval;
                } while(nextFdsUpdate <= clock.ElapsedTime());
            }

            // Pre-movement time. Must run AFTER the FDS block, whose v0 writes
            // would otherwise send a still-waiting agent off for this step, and
            // BEFORE simulation.Iterate(), which reads v0. Nothing in between
            // reads v0; only positions are saved there.
            if(!preMovementTimes.empty()) {
                const double now = clock.ElapsedTime();
                for(auto& runtime : floors) {
                    for(auto& agent : runtime.simulation->Agents()) {
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
            }

            // Who left which opening, and where they went. Done BEFORE
            // Iterate(), not after, because Iterate() starts by draining the
            // removal list (AgentRemovalSystem, called from Simulation.cpp
            // before anything else). At this point the list still holds
            // everything since the last drain: what the previous iteration's
            // Exit stages reported, and what AddAgent queued when a person was
            // set down inside an opening - a landing that lies in the stairway
            // it continues into, which is what a shaft through several storeys
            // looks like. Scanning after Iterate() would miss the latter
            // entirely and those people would vanish from the run.
            //
            // The stage the agent was targeting says which opening it was: the
            // strategical system sets stageId to the stage that reported itself
            // completed, so there is nothing to guess from the position.
            for(size_t f = 0; f < floorCount; ++f) {
                auto& runtime = floors[f];
                auto& simulation = *runtime.simulation;
                for(const auto& removed : simulation.RemovedAgents()) {
                    const auto& agent = simulation.Agent(removed);
                    size_t opening = runtime.exitStages.size();
                    for(size_t e = 0; e < runtime.exitStages.size(); ++e) {
                        if(runtime.exitStages[e] == agent.stageId) {
                            opening = e;
                            break;
                        }
                    }
                    if(opening == runtime.exitStages.size()) {
                        continue;
                    }
                    const size_t link = runtime.connectionForExit[opening];
                    if(link == kLeavesBuilding) {
                        ExitArea& area = exitAreas[f][opening];
                        ++area.left;
                        if(clock.Iteration() == 0) {
                            // Somebody who was standing in the doorway before
                            // the run began. Counted so the tally matches the
                            // population, but deliberately left out of
                            // first/lastLeftTime: they never crossed the
                            // opening, and letting them stretch the window to
                            // t = 0 would understate the specific flow.
                            ++area.presentAtStart;
                            ++presentAtStartTotal;
                            continue;
                        }
                        if(area.firstLeftTime < 0.0) {
                            area.firstLeftTime = clock.ElapsedTime();
                        }
                        area.lastLeftTime = clock.ElapsedTime();
                        continue;
                    }

                    // Into the stairway, as the same person: the agent id comes
                    // from a process-wide counter, so it stays unique across
                    // storeys and the trajectory keeps one continuous track.
                    const auto& connection = config.connections[link];
                    const size_t target = floorIndexById.at(connection.toFloor);
                    const bool ascending =
                        config.floors[target].elevation > config.floors[f].elevation;
                    // Where the person is set down decides which way on is
                    // nearest, exactly as their starting position would have.
                    const auto [onwardJourney, onwardStage] =
                        journeyNearest(target, connection.arrival);
                    pending.push_back(PendingTransfer{
                        .agent = GenericAgent{
                            agent.id,
                            onwardJourney,
                            onwardStage,
                            connection.arrival,
                            agent.orientation,
                            agent.model},
                        .connection = link,
                        .targetFloor = target,
                        .releaseIteration =
                            clock.Iteration() +
                            traversalIterations(
                                link,
                                ascending,
                                baseDesiredSpeeds.at(agent.id.getID())),
                        .startedAt = clock.Iteration(),
                        .departureColumn = floorColumn[f]});
                }
            }

            for(size_t f = 0; f < floorCount; ++f) {
                floors[f].simulation->Iterate();
            }

            // Everybody currently on a stairway, placed along the flight that
            // was drawn for it. Without a drawn flight there is no line to walk
            // along, and the person stays out of the frame as before - showing
            // them at a single point would put a crowd in a doorway that is not
            // there.
            std::vector<JspTrajectoryWriter::TransitRecord> onStairways{};
            onStairways.reserve(pending.size());
            for(const auto& transfer : pending) {
                const auto& link = config.connections[transfer.connection];
                if(!link.flightEnd.has_value()) {
                    continue;
                }
                const uint64_t span = transfer.releaseIteration > transfer.startedAt
                                          ? transfer.releaseIteration - transfer.startedAt
                                          : 1;
                const double progress = std::clamp(
                    static_cast<double>(clock.Iteration() - transfer.startedAt) /
                        static_cast<double>(span),
                    0.0,
                    1.0);
                const Point start = link.entrance;
                const Point finish = *link.flightEnd;
                const Point here{
                    start.x + (finish.x - start.x) * progress,
                    start.y + (finish.y - start.y) * progress};
                const Point heading = (finish - start).Normalized();
                onStairways.push_back(
                    JspTrajectoryWriter::TransitRecord{
                        .agentId = transfer.agent.id.getID(),
                        .position = here,
                        .orientation = heading == Point{} ? Point{1.0, 0.0} : heading,
                        .floorColumn = transfer.departureColumn});
            }

            writer.WriteFrame(
                frameFloors,
                onStairways,
                (totalAgentCount() == 0 && pending.empty()) ||
                    clock.Iteration() >= config.maxIterations);

            if(clock.Iteration() % progressEvery == 0) {
                fmt::print(
                    "progress={} of {} agents={}\n",
                    clock.Iteration(),
                    config.maxIterations,
                    totalAgentCount() + pending.size());
                std::fflush(stdout);
            }

            if(clock.Iteration() % stallCheckEvery == 0) {
                for(size_t f = 0; f < floorCount; ++f) {
                    std::vector<size_t> queueSize(exitAreas[f].size(), 0);
                    for(const auto& agent : floors[f].simulation->Agents()) {
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

                        for(size_t e = 0; e < exitAreas[f].size(); ++e) {
                            if(distanceToExit(exitAreas[f][e], agent.pos) <= kExitVicinity) {
                                ++queueSize[e];
                            }
                        }
                    }
                    for(size_t e = 0; e < exitAreas[f].size(); ++e) {
                        if(queueSize[e] > exitAreas[f][e].maxQueue) {
                            exitAreas[f][e].maxQueue = queueSize[e];
                            exitAreas[f][e].maxQueueTime = clock.ElapsedTime();
                        }
                    }
                }
            }
        }

        if(presentAtStartTotal > 0) {
            fmt::print(
                stderr,
                "warning: {} agents were already inside an exit when the simulation "
                "started; they are counted in left= but excluded from the flow window\n",
                presentAtStartTotal);
        }

        for(size_t rank = 0; rank < floorCount; ++rank) {
            const size_t f = byElevation[rank];
            const std::string where =
                config.multiFloor ? fmt::format("floor={} ", config.floors[f].id)
                                  : std::string{};
            for(size_t e = 0; e < exitAreas[f].size(); ++e) {
                const ExitArea& area = exitAreas[f][e];
                const double width = std::min(area.maxX - area.minX, area.maxY - area.minY);
                if(floors[f].connectionForExit[e] != kLeavesBuilding) {
                    // A stair head is not a way out. How many went down it is
                    // not reported per stairway; only the building-wide
                    // storey_changes= total below covers them.
                    continue;
                }
                const double span = area.lastLeftTime - std::max(0.0, area.firstLeftTime);
                // Only persons that actually crossed the opening carry information
                // about its capacity; those already standing inside at t=0 are in
                // left= for the population count but must not inflate the flow.
                const size_t traversed = area.left - area.presentAtStart;
                const double flow = (traversed > 1 && span > 1e-9)
                                        ? static_cast<double>(traversed) / span
                                        : 0.0;
                fmt::print(
                    "{}exit={} width={:.2f} m left={} first={:.1f} s last={:.1f} s "
                    "flow={:.2f} persons/s\n",
                    where,
                    e + 1,
                    width,
                    area.left,
                    std::max(0.0, area.firstLeftTime),
                    area.lastLeftTime,
                    flow);
                fmt::print(
                    "congestion {}exit={} max_persons_within_{:.0f}m={} at t={:.1f} s\n",
                    where,
                    e + 1,
                    kExitVicinity,
                    area.maxQueue,
                    area.maxQueueTime);
            }
        }

        if(config.multiFloor) {
            fmt::print(
                "storey_changes={} max_persons_in_stairways={} "
                "longest_wait_for_landing={:.1f} s\n",
                transfersDone,
                peakInStairways,
                static_cast<double>(longestLandingWait) * config.dt);
            if(!pending.empty()) {
                fmt::print(
                    "still_in_stairways={} at the end of the run\n", pending.size());
            }
        }

        std::vector<std::pair<uint64_t, Point>> stuck{};
        size_t queueing = 0;
        for(size_t f = 0; f < floorCount; ++f) {
            for(const auto& agent : floors[f].simulation->Agents()) {
                const auto it = stalledSamples.find(agent.id.getID());
                if(it == stalledSamples.end() || it->second < kStallSamples) {
                    continue;
                }
                const bool atExit = std::any_of(
                    exitAreas[f].begin(), exitAreas[f].end(), [&](const ExitArea& area) {
                        return distanceToExit(area, agent.pos) <= kExitVicinity;
                    });
                if(atExit) {
                    ++queueing;
                } else {
                    stuck.emplace_back(agent.id.getID(), agent.pos);
                }
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

        const size_t remaining = totalAgentCount() + pending.size();
        const bool completed = remaining == 0;
        if(completed) {
            fmt::print("Simulation completed.\n");
        } else {
            fmt::print("Simulation stopped at max_iterations.\n");
        }
        fmt::print(
            "iterations={} elapsed_time={} remaining_agents={}\n",
            clock.Iteration(),
            clock.ElapsedTime(),
            remaining);
        fmt::print("jsp_output={}\n", writer.Path().string());

        return completed ? 0 : 2;
    } catch(const std::exception& ex) {
        fmt::print(stderr, "Error: {}\n", ex.what());
        return 1;
    }
}
