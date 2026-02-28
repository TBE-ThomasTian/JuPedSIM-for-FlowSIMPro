// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Journey.hpp"
#include "Stage.hpp"
#include "conversion.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

#include <cstdint>
#include <tuple>
#include <vector>

namespace py = pybind11;

void init_transition(py::module_& m)
{
    py::class_<NonTransitionDescription>(m, "NonTransitionDescription");
    py::class_<FixedTransitionDescription>(m, "FixedTransitionDescription");
    py::class_<RoundRobinTransitionDescription>(m, "RoundRobinTransitionDescription");
    py::class_<LeastTargetedTransitionDescription>(m, "LeastTargetedTransitionDescription");
    py::class_<AdaptiveTransitionDescription>(m, "AdaptiveTransitionDescription");

    m.def("create_none_transition", []() { return NonTransitionDescription{}; });

    m.def("create_fixed_transition", [](uint64_t stageId) {
        return FixedTransitionDescription(stageId);
    });
    m.def(
        "create_round_robin_transition",
        [](const std::vector<std::tuple<uint64_t, uint64_t>>& stageWeights) {
            auto weights = std::vector<std::tuple<BaseStage::ID, uint64_t>>{};
            weights.reserve(stageWeights.size());
            for(const auto& [stage_id, weight] : stageWeights) {
                weights.emplace_back(stage_id, weight);
            }
            return RoundRobinTransitionDescription(weights);
        });
    m.def("create_least_targeted_transition", [](const std::vector<uint64_t>& stages) {
        return LeastTargetedTransitionDescription(intoVecT<BaseStage::ID>(stages));
    });
    m.def(
        "create_adaptive_transition",
        [](const std::vector<uint64_t>& stages,
           double expectedTimeWeight,
           double densityWeight,
           double queueWeight,
           double switchPenalty,
           uint64_t decisionInterval,
           double reconsiderationThreshold) {
            return AdaptiveTransitionDescription(
                intoVecT<BaseStage::ID>(stages),
                expectedTimeWeight,
                densityWeight,
                queueWeight,
                switchPenalty,
                decisionInterval,
                reconsiderationThreshold);
        },
        py::kw_only(),
        py::arg("stage_ids"),
        py::arg("expected_time_weight") = 1.0,
        py::arg("density_weight") = 1.0,
        py::arg("queue_weight") = 0.0,
        py::arg("switch_penalty") = 0.0,
        py::arg("decision_interval") = 1,
        py::arg("reconsideration_threshold") = 0.0);
}
