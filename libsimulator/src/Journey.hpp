// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "GenericAgent.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"
#include "Stage.hpp"
#include "UniqueID.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

class NonTransitionDescription
{
};

class FixedTransitionDescription
{
    BaseStage::ID next{};

public:
    FixedTransitionDescription(BaseStage::ID next_) : next(next_)
    {
        if(next_ == BaseStage::ID::Invalid.getID()) {
            throw SimulationError("Can not create fixed transition from invalid stage id.");
        }
    };

    BaseStage::ID NextId() const { return next; }
};

class RoundRobinTransitionDescription
{
    std::vector<std::tuple<BaseStage::ID, uint64_t>> weightedStages{};

public:
    RoundRobinTransitionDescription(
        const std::vector<std::tuple<BaseStage::ID, uint64_t>>& weightedStages_)
        : weightedStages(weightedStages_)
    {
        for(const auto& [stageId, _] : weightedStages) {
            if(stageId == BaseStage::ID::Invalid.getID()) {
                throw SimulationError(
                    "Can not create round robin transition from invalid stage id.");
            }
        }
    };

    const std::vector<std::tuple<BaseStage::ID, uint64_t>>& WeightedStages() const
    {
        return weightedStages;
    }
};

class LeastTargetedTransitionDescription
{
private:
    std::vector<BaseStage::ID> targetCandidates;

public:
    LeastTargetedTransitionDescription(std::vector<BaseStage::ID> targetCandidates_)
        : targetCandidates(std::move(targetCandidates_))
    {
        for(const auto& stageId : targetCandidates) {
            if(stageId == BaseStage::ID::Invalid.getID()) {
                throw SimulationError(
                    "Can not create least targeted transition from invalid stage id.");
            }
        }
    }

    const std::vector<BaseStage::ID>& TargetCandidates() const { return targetCandidates; }
};

class AdaptiveTransitionDescription
{
private:
    std::vector<BaseStage::ID> targetCandidates;
    double expectedTimeWeight{};
    double densityWeight{};
    double queueWeight{};
    double switchPenalty{};
    uint64_t decisionInterval{};
    double reconsiderationThreshold{};

public:
    AdaptiveTransitionDescription(
        std::vector<BaseStage::ID> targetCandidates_,
        double expectedTimeWeight_ = 1.0,
        double densityWeight_ = 1.0,
        double queueWeight_ = 0.0,
        double switchPenalty_ = 0.0,
        uint64_t decisionInterval_ = 1,
        double reconsiderationThreshold_ = 0.0)
        : targetCandidates(std::move(targetCandidates_))
        , expectedTimeWeight(expectedTimeWeight_)
        , densityWeight(densityWeight_)
        , queueWeight(queueWeight_)
        , switchPenalty(switchPenalty_)
        , decisionInterval(decisionInterval_)
        , reconsiderationThreshold(reconsiderationThreshold_)
    {
        if(targetCandidates.empty()) {
            throw SimulationError("Adaptive transition requires at least one target stage.");
        }
        for(const auto& stageId : targetCandidates) {
            if(stageId == BaseStage::ID::Invalid.getID()) {
                throw SimulationError(
                    "Can not create adaptive transition from invalid stage id.");
            }
        }
        if(expectedTimeWeight < 0.0 || densityWeight < 0.0 || queueWeight < 0.0 ||
           switchPenalty < 0.0) {
            throw SimulationError("Adaptive transition weights and penalty must be >= 0.");
        }
        if(decisionInterval == 0) {
            throw SimulationError("Adaptive transition decision_interval must be > 0.");
        }
        if(reconsiderationThreshold < 0.0) {
            throw SimulationError(
                "Adaptive transition reconsideration_threshold must be >= 0.");
        }
    }

    const std::vector<BaseStage::ID>& TargetCandidates() const { return targetCandidates; }
    double ExpectedTimeWeight() const { return expectedTimeWeight; }
    double DensityWeight() const { return densityWeight; }
    double QueueWeight() const { return queueWeight; }
    double SwitchPenalty() const { return switchPenalty; }
    uint64_t DecisionInterval() const { return decisionInterval; }
    double ReconsiderationThreshold() const { return reconsiderationThreshold; }
};

using TransitionDescription = std::variant<
    NonTransitionDescription,
    FixedTransitionDescription,
    RoundRobinTransitionDescription,
    LeastTargetedTransitionDescription,
    AdaptiveTransitionDescription>;

class Transition
{
public:
    virtual ~Transition() = default;
    virtual BaseStage* NextStage(const GenericAgent& agent) = 0;
};

class FixedTransition : public Transition
{
private:
    BaseStage* next;

public:
    FixedTransition(BaseStage* next_) : next(next_) {};

    BaseStage* NextStage(const GenericAgent&) override { return next; }
};

class RoundRobinTransition : public Transition
{
private:
    std::vector<std::tuple<BaseStage*, uint64_t>> weightedStages{};
    uint64_t nextCalled{};
    uint64_t sumWeights{};

public:
    RoundRobinTransition(std::vector<std::tuple<BaseStage*, uint64_t>> weightedStages_)
        : weightedStages(std::move(weightedStages_))
    {
        for(auto const& [_, weight] : weightedStages) {
            if(weight == 0) {
                throw SimulationError("RoundRobinTransition no weight may be zero.");
            }
            sumWeights += weight;
        }
    }

    BaseStage* NextStage(const GenericAgent&) override
    {
        uint64_t sumWeightsSoFar = 0;
        BaseStage* candidate{};
        for(const auto& [stage, weight] : weightedStages) {
            if(sumWeightsSoFar <= nextCalled) {
                candidate = stage;
            } else {
                break;
            }
            sumWeightsSoFar += weight;
        }
        nextCalled = (nextCalled + 1) % sumWeights;
        return candidate;
    }
};

class LeastTargetedTransition : public Transition
{
private:
    std::vector<BaseStage*> targetCandidates;

public:
    LeastTargetedTransition(std::vector<BaseStage*> targetCandidates_)
        : targetCandidates(std::move(targetCandidates_))
    {
    }

    BaseStage* NextStage(const GenericAgent&) override
    {
        auto leastTargeted = std::min_element(
            std::begin(targetCandidates),
            std::end(targetCandidates),
            [](auto const& a, auto const& b) { return a->CountTargeting() < b->CountTargeting(); });
        return *leastTargeted;
    }
};

class AdaptiveTransition : public Transition
{
private:
    struct AgentDecisionState {
        BaseStage* currentChoice{nullptr};
        uint64_t callsSinceReevaluation{0};
    };

    std::vector<BaseStage*> targetCandidates;
    double expectedTimeWeight{};
    double densityWeight{};
    double queueWeight{};
    double switchPenalty{};
    uint64_t decisionInterval{};
    double reconsiderationThreshold{};
    std::unordered_map<GenericAgent::ID, AgentDecisionState> decisionState{};

    static double DesiredSpeedFromAgent(const GenericAgent& agent)
    {
        return std::visit(
            [](const auto& modelData) -> double {
                using T = std::decay_t<decltype(modelData)>;
                if constexpr(std::is_same_v<T, SocialForceModelData>) {
                    return modelData.desiredSpeed;
                } else {
                    return modelData.v0;
                }
            },
            agent.model);
    }

    static double QueueLoad(const BaseStage* stage)
    {
        if(const auto* queue = dynamic_cast<const NotifiableQueue*>(stage); queue != nullptr) {
            return static_cast<double>(queue->Occupants().size());
        }
        if(const auto* waitingSet = dynamic_cast<const NotifiableWaitingSet*>(stage);
           waitingSet != nullptr) {
            return static_cast<double>(waitingSet->Occupants().size());
        }
        return 0.0;
    }

    bool IsKnownCandidate(const BaseStage* stage) const
    {
        return std::find(std::begin(targetCandidates), std::end(targetCandidates), stage) !=
               std::end(targetCandidates);
    }

    double Cost(
        const GenericAgent& agent,
        BaseStage* candidate,
        BaseStage* previousChoice) const
    {
        const auto distance = (candidate->Target(agent) - agent.pos).Norm();
        const auto speed = std::max(0.1, DesiredSpeedFromAgent(agent));
        const auto expectedTime = distance / speed;
        const auto density = static_cast<double>(candidate->CountTargeting());
        const auto queue = QueueLoad(candidate);
        const auto switchingPenalty =
            (previousChoice != nullptr && previousChoice != candidate) ? switchPenalty : 0.0;
        return (expectedTimeWeight * expectedTime) + (densityWeight * density) +
               (queueWeight * queue) + switchingPenalty;
    }

public:
    AdaptiveTransition(
        std::vector<BaseStage*> targetCandidates_,
        double expectedTimeWeight_,
        double densityWeight_,
        double queueWeight_,
        double switchPenalty_,
        uint64_t decisionInterval_,
        double reconsiderationThreshold_)
        : targetCandidates(std::move(targetCandidates_))
        , expectedTimeWeight(expectedTimeWeight_)
        , densityWeight(densityWeight_)
        , queueWeight(queueWeight_)
        , switchPenalty(switchPenalty_)
        , decisionInterval(decisionInterval_)
        , reconsiderationThreshold(reconsiderationThreshold_)
    {
        if(targetCandidates.empty()) {
            throw SimulationError("Adaptive transition requires at least one target stage.");
        }
    }

    BaseStage* NextStage(const GenericAgent& agent) override
    {
        auto& state = decisionState[agent.id];
        if(state.currentChoice != nullptr && IsKnownCandidate(state.currentChoice)) {
            if(state.callsSinceReevaluation + 1 < decisionInterval) {
                ++state.callsSinceReevaluation;
                return state.currentChoice;
            }
        } else {
            state.currentChoice = nullptr;
            state.callsSinceReevaluation = 0;
        }

        BaseStage* bestStage = nullptr;
        auto bestCost = std::numeric_limits<double>::infinity();
        for(auto* candidate : targetCandidates) {
            const auto candidateCost = Cost(agent, candidate, state.currentChoice);
            if(candidateCost < bestCost) {
                bestCost = candidateCost;
                bestStage = candidate;
            }
        }

        if(bestStage == nullptr) {
            throw SimulationError("Adaptive transition has no candidate stage.");
        }

        if(state.currentChoice != nullptr && IsKnownCandidate(state.currentChoice) &&
           state.currentChoice != bestStage) {
            const auto keepCost = Cost(agent, state.currentChoice, state.currentChoice);
            const auto switchCost = Cost(agent, bestStage, state.currentChoice);
            if(switchCost + reconsiderationThreshold >= keepCost) {
                bestStage = state.currentChoice;
            }
        }

        state.currentChoice = bestStage;
        state.callsSinceReevaluation = 0;
        return state.currentChoice;
    }
};

struct JourneyNode {
    BaseStage* stage;
    std::unique_ptr<Transition> transition;
};

class Journey
{
public:
    using ID = jps::UniqueID<Journey>;

private:
    ID id{};
    std::map<BaseStage::ID, JourneyNode> stages{};

public:
    ~Journey() = default;

    Journey(std::map<BaseStage::ID, JourneyNode> stages_) : stages(std::move(stages_)) {}

    ID Id() const { return id; }

    std::tuple<Point, BaseStage::ID> Target(const GenericAgent& agent) const
    {
        auto& node = stages.at(agent.stageId);
        auto stage = node.stage;
        const auto& transition = node.transition;

        if(stage->IsCompleted(agent)) {
            stage = transition->NextStage(agent);
        }

        return std::make_tuple(stage->Target(agent), stage->Id());
    }

    size_t CountStages() const { return stages.size(); }

    bool ContainsStage(BaseStage::ID stageId) const
    {
        const auto find_iter = stages.find(stageId);
        return find_iter != std::end(stages);
    }

    const std::map<BaseStage::ID, JourneyNode>& Stages() const { return stages; };
};
