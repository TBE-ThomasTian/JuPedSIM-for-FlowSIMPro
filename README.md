# JuPedSim for FlowSIMPro

This repository is a customized fork of [JuPedSim](https://github.com/PedestrianDynamics/jupedsim), adapted for use with FlowSIMPro.

For the original project documentation and upstream development, see the
[upstream repository](https://github.com/PedestrianDynamics/jupedsim).

## FlowSIMPro-specific changes

### 1) Stair and ramp stage support (C++ core + Python API)

- Added a new stage type: `Stair`.
- Added a new stage type: `Ramp`.
- Stair traversal time is modeled per agent as:
  `t = length / (desired_speed * speed_factor) + waiting_time`.
- Ramp traversal time uses a direction-dependent speed factor:
  `up_speed_factor` for ascending and `down_speed_factor` for descending.
- Added geometry validation for stair positions (must be inside walkable area).
- Added geometry validation for ramp positions (must be inside walkable area).
- Added cleanup for removed agents so stair/ramp internal state does not leak.
- Exposed in Python via:
  - `Simulation.add_stair_stage(...)`
  - `Simulation.add_ramp_stage(...)`
  - `StairStage` proxy wrapper
  - `RampStage` proxy wrapper
  - `jupedsim.StairStage` in `__init__`.
  - `jupedsim.RampStage` in `__init__`.

### 2) Native XML CLI (`jupedsim`)

- Added optional native executable `jupedsim`.
- Supports XML scenarios with:
  - geometry and single-exit mode (`<exit>`)
  - multi-exit decision mode (`<decision>` + `<exits mode="...">`)
  - optional stair or ramp definition
  - explicit agents and/or automatic agent distribution.
- Writes compressed `.jsp` trajectory files with frame index and optional per-agent metadata.
- Compression uses `libdeflate`.

Supported distribution modes:

- `by_number`
- `by_density`
- `in_circles_by_number`
- `in_circles_by_density`
- `until_filled`
- `by_percentage`

### 3) New XML examples and format documentation

- Added runnable XML scenarios in `examples/xml/`, including:
  - minimal scenarios
  - stair scenario
  - ramp scenario
  - multi-exit adaptive/round-robin validation scenarios
  - multiple distribution-mode examples
  - 192-agent bottleneck scenarios (uniform and age-mix variants)
- Added adaptive-transition validation script:
  - `examples/example9_adaptive_transition_validation.py` (compares round-robin vs adaptive).
- Added native Monte-Carlo uncertainty analysis script:
  - `scripts/evac_uncertainty_cli.py` (P50/P95, throughput, hotspots via `jupedsim`).
- Added docs:
  - `examples/xml/README_distribution.md` for distribution options
  - `examples/xml/README_transition_validation.md` for multi-exit transition validation
  - `examples/xml/jsp_format.md` for binary JSP v1 layout.

## License

Based on [JuPedSim](https://github.com/PedestrianDynamics/jupedsim), licensed under [GNU LGPLv3](LICENSE).
