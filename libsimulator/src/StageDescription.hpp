// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"
#include "Polygon.hpp"

#include <variant>
#include <vector>

struct DirectSteeringDescription {
};

struct WaypointDescription {
    Point position;
    double distance;
};

struct ExitDescription {
    Polygon polygon;
};

struct NotifiableWaitingSetDescription {
    std::vector<Point> slots;
};

struct NotifiableQueueDescription {
    std::vector<Point> slots;
};

struct StairDescription {
    Point position;
    double distance{0.6};
    double length{5.0};
    bool ascending{true};
    // Defaults keep the previous single-factor behaviour: both directions 0.6
    // until a scenario states otherwise. RiMEA 4.1.1 Tab. 3 has them differ
    // markedly - 0.50 up against 0.65 down for the 30-50 age group.
    double upSpeedFactor{0.6};
    double downSpeedFactor{0.6};
    // Absolute traversal speed in m/s, 0 meaning "not set" so the factor applies.
    // DIN 18009-2 Tab. F.2 gives stair speeds as absolute values per age group
    // and direction, precisely because the reduction is not proportional to how
    // fast the person walks on the level.
    double upSpeed{0.0};
    double downSpeed{0.0};
    double waitingTime{0.0};
    double timeStep{0.01};
};

struct RampDescription {
    Point position;
    double distance{0.6};
    double length{5.0};
    bool ascending{true};
    double upSpeedFactor{0.6};
    double downSpeedFactor{1.0};
    double waitingTime{0.0};
    double timeStep{0.01};
};

using StageDescription = std::variant<
    DirectSteeringDescription,
    WaypointDescription,
    ExitDescription,
    NotifiableWaitingSetDescription,
    NotifiableQueueDescription,
    StairDescription,
    RampDescription>;
