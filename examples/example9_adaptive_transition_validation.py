#! /usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later
import jupedsim as jps


def build_simulation(transition_builder, agent_count=60):
    simulation = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(),
        geometry=[(0, 0), (60, 0), (60, 20), (0, 20)],
    )

    exit_slow = simulation.add_exit_stage(
        [(59, 13), (60, 13), (60, 17), (59, 17)]
    )
    exit_fast = simulation.add_exit_stage(
        [(59, 3), (60, 3), (60, 7), (59, 7)]
    )

    queue_slow = simulation.add_queue_stage(
        [(48, 15), (46, 15), (44, 15), (42, 15), (40, 15), (38, 15)]
    )
    queue_fast = simulation.add_queue_stage(
        [(48, 5), (46, 5), (44, 5), (42, 5), (40, 5), (38, 5)]
    )
    decision_stage = simulation.add_waypoint_stage((15, 10), 0.8)

    journey = jps.JourneyDescription(
        [decision_stage, queue_slow, queue_fast, exit_slow, exit_fast]
    )
    journey.set_transition_for_stage(
        decision_stage, transition_builder(queue_slow, queue_fast)
    )
    journey.set_transition_for_stage(
        queue_slow, jps.Transition.create_fixed_transition(exit_slow)
    )
    journey.set_transition_for_stage(
        queue_fast, jps.Transition.create_fixed_transition(exit_fast)
    )
    journey_id = simulation.add_journey(journey)

    params = jps.CollisionFreeSpeedModelAgentParameters(
        journey_id=journey_id,
        stage_id=decision_stage,
        radius=0.19,
        desired_speed=1.25,
        time_gap=1.0,
    )
    for idx in range(agent_count):
        params.position = (2.0 + 0.6 * (idx % 10), 5.0 + 1.1 * (idx // 10))
        simulation.add_agent(params)

    return simulation, queue_slow, queue_fast


def run_case(name, transition_builder):
    simulation, queue_slow, queue_fast = build_simulation(transition_builder)
    queue_slow_proxy = simulation.get_stage(queue_slow)
    queue_fast_proxy = simulation.get_stage(queue_fast)

    chosen_targets = {}
    max_iterations = 18000
    while (
        simulation.agent_count() > 0
        and simulation.iteration_count() < max_iterations
    ):
        for agent in simulation.agents():
            if (
                agent.id not in chosen_targets
                and agent.stage_id in (queue_slow, queue_fast)
            ):
                chosen_targets[agent.id] = agent.stage_id

        # Emulate asymmetric capacity: upper queue is slower than lower queue.
        if simulation.iteration_count() % 40 == 0:
            queue_slow_proxy.pop(1)
        if simulation.iteration_count() % 10 == 0:
            queue_fast_proxy.pop(1)

        simulation.iterate()

    slow_choices = sum(1 for stage in chosen_targets.values() if stage == queue_slow)
    fast_choices = sum(1 for stage in chosen_targets.values() if stage == queue_fast)

    return {
        "name": name,
        "iterations": simulation.iteration_count(),
        "evacuated": simulation.agent_count() == 0,
        "slow_choices": slow_choices,
        "fast_choices": fast_choices,
    }


def main():
    round_robin = run_case(
        "round_robin",
        lambda queue_slow, queue_fast: jps.Transition.create_round_robin_transition(
            [(queue_slow, 1), (queue_fast, 1)]
        ),
    )
    adaptive = run_case(
        "adaptive",
        lambda queue_slow, queue_fast: jps.Transition.create_adaptive_transition(
            [queue_slow, queue_fast],
            expected_time_weight=0.5,
            density_weight=0.8,
            queue_weight=3.0,
            switch_penalty=0.8,
            decision_interval=2,
            reconsideration_threshold=0.1,
        ),
    )

    print("Validation comparison:")
    for result in (round_robin, adaptive):
        print(
            f"  {result['name']}: "
            f"evacuated={result['evacuated']} "
            f"iterations={result['iterations']} "
            f"slow={result['slow_choices']} fast={result['fast_choices']}"
        )

    if not round_robin["evacuated"] or not adaptive["evacuated"]:
        raise RuntimeError("At least one scenario did not evacuate all agents.")

    if adaptive["iterations"] >= round_robin["iterations"]:
        raise RuntimeError(
            "Adaptive transition did not improve evacuation iterations in this validation setup."
        )

    print("Validation passed: adaptive transition improved evacuation time.")


if __name__ == "__main__":
    main()
