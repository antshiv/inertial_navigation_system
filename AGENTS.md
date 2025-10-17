# Repository Guidelines

## Project Structure & Module Map
The INS top level orchestrates the flight controller stack. `include/` exposes HAL and orchestration APIs, while `src/` implements scheduling, sensor plumbing, and wrappers. `external/stateEstimation`, `external/controlSystems`, and `external/dynamic_models` host the sensor fusion, controller, and physics submodules; `external/attitudeMathLibrary` supplies quaternion math shared across modules. Examples live in `examples/` (e.g., `example_flight_control.c`), tests reside in `tests/`, and fixtures plus logs sit under `data/`. Keep submodules pinned via `.gitmodules`.

## Build, Test, and Development Commands
- `git submodule update --init --recursive` to sync math, estimation, dynamics, and control layers before any build.
- `cmake -S inertial_navigation_system -B build/ins -DCMAKE_BUILD_TYPE=RelWithDebInfo` configures the aggregator; reuse `build/` sibling directories for the submodules.
- `cmake --build build/ins --target example_flight_control` compiles the end-to-end demo; swap the target for `test_ins` or any module unit.
- `ctest --test-dir build/ins --output-on-failure` runs the INS suite; call `ctest --test-dir external/stateEstimation/build` etc. when iterating on a specific library.

## Coding Style & Naming
Use C11, 4-space indentation, and `lower_snake_case` for functions (`ins_update`, `dm_integrate_rk4`). Types use `TitleCase` structs (`AltitudeEstimator`), constants stay in all caps. Keep headers self-contained, include order as: module public header, standard library, then third-party. Leverage existing patterns such as the `ins_` prefix for orchestration glue and `dm_` for dynamic models. Prefer small, pure helpers and add a brief comment only when the intent is non-obvious.

## Testing Expectations
- Add or extend tests alongside code in the relevant `tests/` tree; mirror file naming (`test_attitude.c`, `test_physics_model.c`).
- Ensure new dynamics or controllers run through the INS integration path by wiring them into `tests/test_ins.c` or an `examples/` scenario.
- Capture coverage gaps in PR notes when hardware loops or sensors cannot be simulated.

## Commit & PR Checklist
- Follow the existing imperative, sentence-case commit style (“Add quaternion debug toggle”); group cross-repo work into separate commits per module.
- Reference dependent module updates in the PR description and list any submodules that require synchronized bumps.
- Provide build/test proofs (`cmake --build …`, `ctest …`) and attach logs or plots for dynamics regressions; include screenshots when demos change telemetry.

## Integration Tips
- Update shared interfaces in attitude math first, then regenerate state estimator bindings before touching controllers.
- When adding sensors, extend `include/hal.h` and propagate data structs through estimation and control layers in the same branch.
- Document new tuning parameters in `doc/` and mirror the explanation in example configs so agents can reproduce results.
