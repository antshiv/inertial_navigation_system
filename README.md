# Inertial Navigation System

`inertial_navigation_system` is the pure-C integration layer for calibrated
sensor samples, estimator timing, and flight-state output. Its first accepted
milestone is attitude and heading orchestration; full position/velocity inertial
navigation is not yet implemented.

## Accepted Path

```text
synthetic or calibrated gyro + accel + mag
                    |
                    v
        timestamp-checked INS update
                    |
                    v
      complementary AHRS quaternion/rates
                    |
                    v
       PID -> wrench -> rotor mixer
                    |
                    v
          RK4 rigid-body plant
                    |
             sensor generation
                    +---------------- feedback
```

The closed-loop fixture starts the simulated vehicle at a 12-degree roll
disturbance. The controller receives only estimated quaternion and body rates;
the plant's true state is used only to generate ideal sensor vectors and score
the trajectory.

Current deterministic result:

```text
initial attitude error: 0.209440 rad
final attitude error:   0.000258 rad
maximum AHRS error:     0.000054 rad
simulation duration:    5 seconds at 500 Hz
```

## Repository Boundaries

- `stateEstimation` owns AHRS and Kalman equations plus independent oracles.
- `controlSystems` owns PID and rotor allocation.
- `dynamic_models` owns the rigid-body plant and numerical propagation.
- This repository owns sensor timestamps, estimator orchestration, dependency
  composition, and end-to-end scenarios.
- Board drivers, calibration, Zephyr tasks, DShot, safety policy, and hardware
  tests belong to the flight-controller integration repository.

## Build And Test

```bash
git clone --recursive https://github.com/antshiv/inertial_navigation_system.git
cd inertial_navigation_system
cmake -S . -B build \
  -DINS_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_FLAGS="-Wall -Wextra -Wpedantic -Werror"
cmake --build build
ctest --test-dir build --output-on-failure
```

## Deliberate Limits

The current loop assumes calibrated, synchronous, disturbance-free sensors and
an ideal algebraic rotor model. It does not yet model gyro/accelerometer bias
inside the closed loop, magnetic interference, linear acceleration rejection,
motor lag, actuator saturation, wind, delay, dropout, position, velocity,
barometric altitude, GPS, or hardware timing.

PID is the only controller accepted in the closed loop. LQR should be added
only after a state-space model is tied to the same plant parameters and checked
against an independent reference. MPC and other controllers follow the same
rule.

## License

The repository does not currently contain a license file. Until one is added,
no open-source permission should be inferred from this source being public.
