# JuPedSim for FlowSIMPro

This repository is a customized fork of [JuPedSim](https://github.com/PedestrianDynamics/jupedsim), adapted for use with FlowSIMPro.

For the original project documentation and upstream development, see the
[upstream repository](https://github.com/PedestrianDynamics/jupedsim).

## FlowSIMPro-specific changes

### 1) Stair stage support (C++ core + Python API)

- Added a new stage type: `Stair`.
- Stair traversal time is modeled per agent as:
  `t = length / (desired_speed * speed_factor) + waiting_time`.
- Added geometry validation for stair positions (must be inside walkable area).
- Added cleanup for removed agents so stair-internal state does not leak.
- Exposed in Python via:
  - `Simulation.add_stair_stage(...)`
  - `StairStage` proxy wrapper
  - `jupedsim.StairStage` in `__init__`.

### 2) Native XML CLI (`jupedsim-cli`)

- Added optional native executable `jupedsim-cli`.
- Supports XML scenarios with:
  - geometry and exit
  - optional stair definition
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
  - multiple distribution-mode examples
  - 192-agent bottleneck scenarios (uniform and age-mix variants)
- Added docs:
  - `examples/xml/README_distribution.md` for distribution options
  - `examples/xml/jsp_format.md` for binary JSP v1 layout.

## Building

### Standard build

```bash
cmake -S . -B build
cmake --build build -j
```

### Build with native XML CLI

```bash
cmake -S . -B build -DBUILD_XML_CLI=ON
cmake --build build -j
```

`BUILD_XML_CLI=ON` requires `libdeflate` (e.g. system `libdeflate-dev` package).
If needed, set `-DLIBDEFLATE_ROOT=/path/to/libdeflate`.

## CLI usage examples

```bash
./build/bin/jupedsim-cli examples/xml/minimal_scenario.xml \
  --out-jsp examples/xml/minimal_scenario.jsp

./build/bin/jupedsim-cli examples/xml/distribution_by_number.xml \
  --out-jsp examples/xml/distribution_by_number.jsp \
  --every-nth-frame 2 \
  --compression-level 6
```

## Python usage example (stair stage)

```python
stair_stage = simulation.add_stair_stage(
    position=(10.0, 5.0),
    length=8.0,
    distance=0.6,
    speed_factor=0.6,
    waiting_time=0.0,
    time_step=0.01,
)
```

## License

Based on [JuPedSim](https://github.com/PedestrianDynamics/jupedsim), licensed under [GNU LGPLv3](LICENSE).
