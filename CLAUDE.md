# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

mc_mujoco is a MuJoCo simulator interface for mc_rtc (a robotics control framework). It bridges MuJoCo physics simulation with mc_rtc's controller system, allowing FSM controllers to run in MuJoCo with real-time GUI visualization.

## Development Environment

**All building, testing, and debugging MUST be done inside the Docker container.** Do not attempt to build or run locally — the host may lack required dependencies (mc_rtc, MuJoCo, ROS 2, etc.).

```bash
# Build and get a dev shell (builds mc_mujoco, then drops to bash inside container)
cd docker && make run

# Run CI tests (build + standalone library test)
cd docker && make ci-test
```

Inside the container:
- Source is mounted at `/workspace/mc_mujoco`
- Build dir: `/workspace/mc_mujoco/build` (persistent Docker volume)
- MuJoCo at `/opt/mujoco`
- Rebuild: `cmake --build /workspace/mc_mujoco/build`
- Reinstall: `cmake --install /workspace/mc_mujoco/build`
- Run standalone tests: `ctest -V --test-dir /ci-build-standalone`

To run arbitrary commands in the dev container:
```bash
docker compose -f docker/docker-compose.yml run --rm mc-rtc-mujoco bash -c "<command>"
```

**CMake options:** `USE_GL` (default ON) enables OpenGL rendering; `MUJOCO_ROOT_DIR` is set to `/opt/mujoco` in the container.

**Submodules:** Clone with `--recursive` (jvrc_mj_description for robot model, mc_rtc-imgui for GUI).

## Formatting & Linting

- `.clang-format`: Allman style, 120 column limit, 2-space indent
- `pre-commit run --all-files` runs clang-format, cmake-format, and standard checks
- `ext/` directory is excluded from formatting

## Architecture

### Core flow (src/main.cpp)
1. Parses CLI args (config file, torque control, step-by-step, GUI flags)
2. Loads MuJoCo plugins from `$MUJOCO_BIN_DIR/mujoco_plugin`
3. Creates `MjSim` instance with `MjConfiguration`
4. Spawns separate simulation thread and rendering thread

### Key source files
- **mj_sim.h/cpp** — Public simulator API: `stepSimulation()`, `render()`, `resetSimulation()`, access to `MCGlobalController`, MuJoCo model/data
- **mj_sim_impl.h** — Internal state: MuJoCo model/data lifecycle, mc_rtc controller integration, PD control, collision/contact handling
- **mj_utils.cpp** — XML model loading, robot configuration, PD gain setup
- **mj_utils_merge_mujoco_models.cpp** — Merges multiple MuJoCo XML models into a single scene
- **MujocoClient.h/cpp** — GUI client extending `mc_rtc::imgui::Client` for 2D (ImGui) + 3D (ImGuizmo) rendering
- **widgets/** — GUI elements (Arrow, Force, Point3D, Polygon, Transform, etc.)
- **mj_configuration.h** — Configuration struct (visualization, controller, torque control, real-time sync flags)

### mc_rtc framework concepts

mc_rtc is a unified interface for simulation and robot control. Key concepts relevant to this project:

- **MCGlobalController** — Top-level controller manager. Handles sensor data, loads controllers dynamically, manages plugins and observers. mc_mujoco instantiates this to run mc_rtc controllers against MuJoCo physics.
- **MCController** — Base class for all controllers. Manages robots, tasks, constraints, QPSolver, datastore, logger, and GUI. Assumes at least 2 robots (first is "main").
- **FSM Controller** — Inherits MCController. Manages state machines for complex robot behaviors with configurable transitions.
- **States** — FSM building blocks with lifecycle: construct → init → run (until done) → teardown → destruct. Loaded dynamically from shared libraries via `StateFactory`. Registered with `EXPORT_SINGLE_STATE("Name", Type)`.
- **GlobalPlugin** — Plugins loaded by MCGlobalController with `init()`, `reset()`, `before()`, `after()` hooks. Registered with `EXPORT_MC_RTC_PLUGIN("Name", Type)`.
- **Observers** — State estimators (encoders, IMU fusion) with `run()` before controller and `update()` to write estimates. Registered with `EXPORT_OBSERVER_MODULE("Name", Type)`.
- **Datastore** — Key-value store shared between controller and interface. mc_mujoco exposes callbacks (`SetPosW`, `SetPDGains`, `GetPDGains`, etc.) through the datastore.
- **Configuration** — mc_rtc reads config from `$HOME/.config/mc_rtc/mc_rtc.conf` (user) and per-controller/plugin YAML files.

### Robot models (robots/)
Object models (box, table, ground) use `.in.yaml` templates with `@VAR@` substitution. Objects are configured in `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml`.

### Vendored libraries (ext/)
imgui, implot, ImGuizmo, pugixml, glfw (conditional). Built as part of the `mc_mujoco_lib` static library.
