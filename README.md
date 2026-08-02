# JuPedSim for FlowSIMPro

This repository is a customized fork of [JuPedSim](https://github.com/PedestrianDynamics/jupedsim), adapted for use with FlowSIMPro.

For the original project documentation and upstream development, see the
[upstream repository](https://github.com/PedestrianDynamics/jupedsim).

## Building

The project builds from the same sources on Linux (GCC/Clang) and Windows (MSVC).

### Prerequisites

- CMake >= 3.22, a C++20 compiler, and a build tool (Ninja recommended)
- All third-party dependencies are vendored as git submodules, so **clone recursively**:

```
git submodule update --init --recursive
```

This step is mandatory. Without it CMake fails on the very first `add_subdirectory(fmt)`,
and `BUILD_XML_CLI=ON` fails to find `libdeflate`.

### Linux

```
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_XML_CLI=ON -DBUILD_TESTS=ON
cmake --build build -j
./build/bin/libsimulator-tests      # unit tests
./build/bin/jupedsim --help         # native XML CLI
```

### Windows

```
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_XML_CLI=ON -DBUILD_TESTS=ON
cmake --build build --config Release -j
```

### CMake options

| Option | Default | Notes |
| --- | --- | --- |
| `BUILD_XML_CLI` | `OFF` | Native `jupedsim` XML CLI. Builds `libdeflate` from `third-party/`. |
| `BUILD_TESTS` | `OFF` | GoogleTest unit tests (`libsimulator-tests`). |
| `BUILD_PYTHON_BINDINGS` | `OFF` | `py_jupedsim` extension module. Requires a Python dev install. Not needed for the CLI. |
| `BUILD_BENCHMARKS` | `OFF` | Google Benchmark micro benchmarks. |
| `WERROR` | `OFF` | Warnings as errors. GCC/Clang only. |
| `WITH_FORMAT` | `OFF` | clang-format targets. Unix only. |
| `BUILD_WITH_ASAN` | `OFF` | Address sanitizer. Clang only (the flags are not wired up for GCC). |
| `LIBDEFLATE_ROOT` | *(empty)* | Only needed to point at an out-of-tree libdeflate instead of the submodule. |

`BUILD_TESTS=ON` together with `BUILD_PYTHON_BINDINGS=ON` additionally requires `pytest`
to be importable, because the system tests are wired into the CMake configure step.

## Building the installer

`BUILD_INSTALLER=ON` adds install rules and CPack packaging for the **FlowSIM Pro Evac Add-On**.

```
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_XML_CLI=ON -DBUILD_INSTALLER=ON
cmake --build build -j
cd build && cpack
```

The add-on carries a year-based release number, `FLOWSIMPRO_ADDON_VERSION`
(default `2026`), independent of the upstream JuPedSim version.

| Platform | Generators | Result |
| --- | --- | --- |
| Linux | `TGZ`, `DEB`, `RPM` *(if `rpmbuild` is present)* | `flowsimpro-evac-addon_2026_amd64.deb`, `flowsimpro-evac-addon-2026-1.x86_64.rpm`, `flowsimpro-evac-addon-2026-Linux.tar.gz` |
| Windows | `ZIP`, `NSIS` *(if `makensis` is present)* | `flowsimpro-evac-addon-2026-win64.exe`, `flowsimpro-evac-addon-2026-win64.zip` |

The install prefix is chosen so that FlowSIM Pro auto-detects the solver with no
user configuration — these are exactly the paths probed by
`JuPedSimHelper::defaultCandidates()`:

- Linux: `/opt/FlowSIMProEvacAddOn/bin/jupedsim`
- Windows: `C:\Program Files\FlowSIMProEvacAddOn\bin\jupedsim.exe`

Example scenarios and format docs are installed alongside under
`share/jupedsim/`. The Linux packages declare only `libc6`, `libgcc-s1` and
`libstdc++6`; CGAL, Boost, fmt, glm and libdeflate are linked statically.

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
  - `examples/xml/jsp_format.md` for the binary JSP layout and optional metadata blocks.

## License

Based on [JuPedSim](https://github.com/PedestrianDynamics/jupedsim), licensed under [GNU LGPLv3](LICENSE).
