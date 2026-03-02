// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModel.hpp"
#include "CollisionFreeSpeedModelData.hpp"
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

constexpr uint32_t JSP_VERSION = 1;
constexpr uint32_t JSP_FLAG_DEFLATE = 1;
constexpr uint32_t JSP_RECORD_SIZE = 24; // u64 agent_id + 4x f32
constexpr char JSP_META_MAGIC[4] = {'J', 'S', 'P', 'M'};
constexpr uint32_t JSP_META_VERSION = 1;
constexpr uint32_t JSP_META_RECORD_SIZE = 24; // u64 id + u8 age + u8 avatar + u16 + 3x f32
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

struct AgentConfig {
    Point position{};
    double radius{0.2};
    double timeGap{1.0};
    double desiredSpeed{1.2};
    std::string ageGroup{};
    std::string avatarHint{};
};

struct AgentSpawnProfile {
    double radius{0.2};
    double timeGap{1.0};
    double desiredSpeed{1.2};
    std::string ageGroup{};
    std::string avatarHint{};
    double weight{1.0};
};

enum class DistributionMode {
    ByNumber,
    ByDensity,
    InCirclesByNumber,
    InCirclesByDensity,
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

struct AgentDistributionConfig {
    DistributionMode mode{DistributionMode::ByNumber};
    double distanceToAgents{0.4};
    double distanceToPolygon{0.0};
    std::optional<uint64_t> numberOfAgents{};
    std::optional<double> density{};
    std::optional<double> percent{};
    std::optional<Point> centerPoint{};
    std::vector<CircleSegmentConfig> circleSegments{};
    std::optional<uint64_t> seed{};
    uint64_t maxIterations{10000};
    uint32_t k{30};
    AgentSpawnProfile defaultProfile{};
    std::vector<AgentSpawnProfile> profiles{};
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
        double speedFactor{0.6};
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
};

struct CliArgs {
    std::string scenarioPath{};
    std::optional<uint64_t> maxIterationsOverride{};
    std::optional<std::string> outputPath{};
    uint32_t everyNthFrame{1};
    int compressionLevel{6};
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
        "           speed_factor=\"0.6\" waiting_time=\"0.0\"/> <!-- optional -->\n"
        "    <ramp x=\"...\" y=\"...\" length=\"10.0\" distance=\"0.6\" ascending=\"true\"\n"
        "          up_speed_factor=\"0.6\" down_speed_factor=\"1.0\" waiting_time=\"0.0\"/>\n"
        "          <!-- optional, use either <stair> or <ramp> -->\n"
        "    <agents>\n"
        "      <agent x=\"1\" y=\"1\" radius=\"0.2\" time_gap=\"1.0\" desired_speed=\"1.2\"\n"
        "             age_group=\"young|adult|elderly\" avatar_hint=\"young|adult|grandpa|grandma\"/>\n"
        "      <!-- Or generate agents automatically -->\n"
        "      <distribution mode=\"by_number\" number_of_agents=\"200\"\n"
        "                    distance_to_agents=\"0.45\" distance_to_polygon=\"0.20\" seed=\"42\" />\n"
        "      <distribution mode=\"in_circles_by_density\" center_x=\"40\" center_y=\"15\"\n"
        "                    distance_to_agents=\"0.45\" distance_to_polygon=\"0.20\" seed=\"42\">\n"
        "        <segment min_radius=\"0\" max_radius=\"8\" density=\"2.0\"/>\n"
        "        <segment min_radius=\"8\" max_radius=\"16\" density=\"1.0\"/>\n"
        "      </distribution>\n"
        "      <profile desired_speed=\"1.55\" radius=\"0.19\" time_gap=\"0.75\"\n"
        "               age_group=\"young\" avatar_hint=\"young\" weight=\"1.0\"/>\n"
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
    if(value == "grandpa") {
        return AVATAR_HINT_GRANDPA;
    }
    if(value == "grandma") {
        return AVATAR_HINT_GRANDMA;
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
    if(mode == "until_filled") {
        return DistributionMode::UntilFilled;
    }
    if(mode == "by_percentage") {
        return DistributionMode::ByPercentage;
    }
    throw std::runtime_error(
        "Unknown distribution mode '" + value +
        "'. Supported: by_number, by_density, in_circles_by_number, "
        "in_circles_by_density, until_filled, by_percentage");
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
    profile.ageGroup =
        ToLowerAscii(node.get<std::string>("<xmlattr>.age_group", profile.ageGroup));
    profile.avatarHint =
        ToLowerAscii(node.get<std::string>("<xmlattr>.avatar_hint", profile.avatarHint));

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
        const auto& profile = PickProfile(effectiveProfiles, profileRng);
        generatedAgents.push_back(
            AgentConfig{
                .position = p,
                .radius = profile.radius,
                .timeGap = profile.timeGap,
                .desiredSpeed = profile.desiredSpeed,
                .ageGroup = profile.ageGroup,
                .avatarHint = profile.avatarHint,
            });
    }
    return generatedAgents;
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

    const auto& geometryNode = scenario.get_child("geometry");
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
        stair.speedFactor =
            stairNode.get<double>("<xmlattr>.speed_factor", stair.speedFactor);
        stair.waitingTime =
            stairNode.get<double>("<xmlattr>.waiting_time", stair.waitingTime);

        if(stair.distance <= 0.0) {
            throw std::runtime_error("scenario.stair.distance must be > 0");
        }
        if(stair.length < 0.0) {
            throw std::runtime_error("scenario.stair.length must be >= 0");
        }
        if(stair.speedFactor <= 0.0) {
            throw std::runtime_error("scenario.stair.speed_factor must be > 0");
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
        agent.ageGroup =
            ToLowerAscii(node.get<std::string>("<xmlattr>.age_group", std::string{}));
        agent.avatarHint =
            ToLowerAscii(node.get<std::string>("<xmlattr>.avatar_hint", std::string{}));
        if(agent.radius <= 0.0) {
            throw std::runtime_error(context + ".radius must be > 0");
        }
        if(agent.timeGap <= 0.0) {
            throw std::runtime_error(context + ".time_gap must be > 0");
        }
        if(agent.desiredSpeed <= 0.0) {
            throw std::runtime_error(context + ".desired_speed must be > 0");
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

class JspTrajectoryWriter
{
public:
    JspTrajectoryWriter(
        fs::path path,
        double dt,
        uint32_t everyNthFrame,
        int compressionLevel,
        std::vector<AgentProfileEntry> agentProfiles)
        : _path(std::move(path))
        , _everyNthFrame(everyNthFrame)
        , _compressionLevel(compressionLevel)
        , _agentProfiles(std::move(agentProfiles))
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
        if(config.stair.has_value()) {
            const auto& stair = *config.stair;
            const auto stairStage = simulation.AddStage(StairDescription{
                .position = stair.position,
                .distance = stair.distance,
                .length = stair.length,
                .speedFactor = stair.speedFactor,
                .waitingTime = stair.waitingTime,
                .timeStep = config.dt,
            });
            journeyStages.emplace(stairStage, FixedTransitionDescription(downstreamStage));
            initialStage = stairStage;
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
        }
        const auto journeyId = simulation.AddJourney(journeyStages);

        std::vector<AgentProfileEntry> agentProfiles{};
        agentProfiles.reserve(allAgents.size());

        for(const auto& agentConfig : allAgents) {
            CollisionFreeSpeedModelData modelData{};
            modelData.timeGap = agentConfig.timeGap;
            modelData.v0 = agentConfig.desiredSpeed;
            modelData.radius = agentConfig.radius;

            GenericAgent agent{
                GenericAgent::ID::Invalid,
                journeyId,
                initialStage,
                agentConfig.position,
                Point{1.0, 0.0},
                modelData};
            const auto agentId = simulation.AddAgent(std::move(agent)).getID();

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

        JspTrajectoryWriter writer(
            outputPath,
            config.dt,
            args.everyNthFrame,
            args.compressionLevel,
            std::move(agentProfiles));

        writer.WriteFrame(simulation, true);
        while(simulation.AgentCount() > 0 && simulation.Iteration() < config.maxIterations) {
            simulation.Iterate();
            writer.WriteFrame(
                simulation,
                simulation.AgentCount() == 0 ||
                    simulation.Iteration() >= config.maxIterations);
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
