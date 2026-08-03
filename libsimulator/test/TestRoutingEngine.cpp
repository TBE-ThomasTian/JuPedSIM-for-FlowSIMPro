// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GeometryBuilder.hpp"
#include "RoutingEngine.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <vector>

namespace
{
/// RiMEA Test 12b, short variant: two 10 x 10 m rooms joined by a 1 m opening in
/// a 0.20 m wall, the way out along the far side of the second room. This is the
/// geometry in which a person was caught for 430 s of a 600 s run, because the
/// route it was given led through the wall and it walked into it for good.
RoutingEngine BottleneckRooms()
{
    GeometryBuilder b{};
    b.AddAccessibleArea({{0.0, 0.0}, {20.2, 0.0}, {20.2, 10.0}, {0.0, 10.0}});
    b.ExcludeFromAccessibleArea({{10.0, 0.0}, {10.2, 0.0}, {10.2, 4.5}, {10.0, 4.5}});
    b.ExcludeFromAccessibleArea({{10.0, 5.5}, {10.2, 5.5}, {10.2, 10.0}, {10.0, 10.0}});
    const auto geometry = b.Build();
    return RoutingEngine{geometry.Polygon()};
}

/// The first point of `path` that is not inside the navigable area, sampled
/// every centimetre. A route is only a route if a person can walk it.
std::optional<Point> FirstPointInsideAWall(RoutingEngine& routing, const std::vector<Point>& path)
{
    for(size_t i = 1; i < path.size(); ++i) {
        const auto from = path[i - 1];
        const auto to = path[i];
        const auto length = (to - from).Norm();
        const auto steps = static_cast<int>(length / 0.01) + 1;
        for(int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const Point p{from.x + t * (to.x - from.x), from.y + t * (to.y - from.y)};
            if(!routing.IsRoutable(p)) {
                return p;
            }
        }
    }
    return std::nullopt;
}
} // namespace

TEST(RoutingEngine, GeometryUnderTestReallyHasTheWall)
{
    // Guards the two tests below: if the obstacles were not subtracted, a route
    // straight through them would be correct and the tests would pass while
    // proving nothing.
    auto routing = BottleneckRooms();
    EXPECT_FALSE(routing.IsRoutable(Point{10.1, 2.0})) << "lower wall is missing";
    EXPECT_FALSE(routing.IsRoutable(Point{10.1, 8.0})) << "upper wall is missing";
    EXPECT_TRUE(routing.IsRoutable(Point{10.1, 5.0})) << "the opening is walled up";
}

TEST(RoutingEngine, RouteThroughAnOpeningNeverCrossesTheWall)
{
    // The funnel used to stop one turn before the end, so the corner it had to
    // round was still inside it when it stopped and was dropped. From most of
    // the left room the answer was then the destination itself: a straight line
    // through 0.20 m of wall. Swept over the room rather than checked at one
    // point, because the positions that failed and the ones that did not lay
    // centimetres apart.
    auto routing = BottleneckRooms();
    const Point destination{19.95, 5.0};

    for(int ix = 0; ix <= 18; ++ix) {
        for(int iy = 0; iy <= 18; ++iy) {
            const Point start{0.5 + ix * 0.5, 0.5 + iy * 0.5};
            if(!routing.IsRoutable(start)) {
                continue;
            }
            const auto path = routing.ComputeAllWaypoints(start, destination);
            const auto blocked = FirstPointInsideAWall(routing, path);
            EXPECT_FALSE(blocked.has_value())
                << "route from (" << start.x << ", " << start.y << ") leaves the "
                << "navigable area at (" << blocked->x << ", " << blocked->y << ")";
        }
    }
}

TEST(RoutingEngine, RouteEndsAtTheDestinationExactlyOnce)
{
    // The closing portal is the destination, so the funnel can emit it before
    // the tail does. A duplicate makes the last leg zero-length and can hand
    // ComputeWaypoint a waypoint identical to the one before it.
    auto routing = BottleneckRooms();
    const Point destination{19.95, 5.0};
    const auto path = routing.ComputeAllWaypoints(Point{2.0, 2.0}, destination);

    ASSERT_GE(path.size(), 2u);
    EXPECT_LT(Distance(path.back(), destination), 1e-9);
    EXPECT_GT(Distance(path[path.size() - 2], destination), 1e-9)
        << "destination appears twice at the end of the path";
}
