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
    double speedFactor{0.6};
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
