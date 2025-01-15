# Inertial Navigation System (INS) Library

The **Inertial Navigation System (INS)** library is a modular and extensible framework written in **C** to provide high performance, portability, and efficiency for embedded systems. It integrates state estimation, control systems, and dynamic models into a single cohesive library for real-time applications such as drones, robotics, and test rigs.

---

## Features

- **Written in C**:
  - Lightweight and portable for cross-platform compatibility.
  - Optimized for speed and low memory usage, ideal for low-power MCU applications.
- **State Estimation**:
  - Sensor fusion algorithms for IMU, GPS, and barometer data using the `stateEstimation` library.
- **Control Systems**:
  - Real-time control algorithms like PID, LQR, and MPC using the `controlSystems` library.
- **Dynamic Models**:
  - Physics-based dynamic models for drones and other systems using the `dynamic_models` library.
- **HAL Integration**:
  - Hardware abstraction layer (HAL) to interface with sensors and actuators.
- **Scalability**:
  - Modular design to extend functionality or integrate additional libraries as needed.

---

## Folder Structure

```plaintext
├── CMakeLists.txt             # Build system configuration
├── data/                      # Sample input/output data for testing and simulation
├── examples/                  # Example applications
│   ├── example_flight_control.c  # Example: INS for drone flight control
│   └── example_test_rig.c        # Example: INS in the test rig environment
├── external/                  # External libraries included as Git submodules
│   ├── controlSystems         # Control systems library
│   ├── dynamic_models         # Dynamic models library
│   └── stateEstimation        # State estimation library
├── include/                   # Public headers
│   ├── hal.h                  # Hardware abstraction interface
│   ├── ins.h                  # Main entry point for INS
│   └── wrappers/              # Wrapper headers for integrating libraries
├── README.md                  # Overview and usage
├── src/                       # Core implementation
│   ├── hal.c                  # HAL implementation
│   ├── ins.c                  # INS core implementation
│   └── wrappers/              # Wrapper implementations for integrating libraries
└── tests/                     # Unit and integration tests
    ├── test_attitude_math.c   # Test for attitude math functionality
    ├── test_control_systems.c # Test for control systems functionality
    ├── test_dynamic_models.c  # Test for dynamic models functionality
    ├── test_hal.c             # Test for HAL integration
    ├── test_ins.c             # Test for INS integration
    └── test_state_estimation.c # Test for state estimation functionality
```

---

## Getting Started

### **Cloning the Repository**

To get started, clone the repository and initialize the submodules:
```bash
git clone --recurse-submodules https://github.com/yourusername/inertial_navigation_system.git
cd inertial_navigation_system
```

If you’ve already cloned the repository without submodules, initialize and update them manually:
```bash
git submodule update --init --recursive
```

### **Pulling Updates for Submodules**

If any submodule is updated, you can pull the latest changes:
```bash
git submodule update --remote
```

---

## Building the Library

1. **Create a Build Directory**:
   ```bash
   mkdir build && cd build
   ```

2. **Configure the Build System**:
   Use `cmake` to configure the build:
   ```bash
   cmake ..
   ```

3. **Compile the Library**:
   Compile the library and examples:
   ```bash
   make
   ```

4. **Run Tests** (optional):
   Execute the unit and integration tests:
   ```bash
   make test
   ```

---

## Usage

### **Integration**
Include the `ins.h` header file in your application to access the INS API:
```c
#include "ins.h"
```

### **Initialization**
Initialize the INS library and its components:
```c
ins_init();
```

### **Update Loop**
Call `ins_update()` periodically (e.g., at 100 Hz) to update the INS state:
```c
float dt = 0.01; // 10 ms
ins_update(dt);
```

### **Retrieve State**
Get the current position, velocity, and orientation:
```c
float position[3], velocity[3], orientation[3];
ins_get_state(position, velocity, orientation);
```

---

## Examples

- **Drone Flight Control**:
  Demonstrates how to use the INS library for real-time drone control (`examples/example_flight_control.c`).
- **Test Rig Simulation**:
  Validates INS functionality in a simulated test rig environment (`examples/example_test_rig.c`).

---

## Testing

Run unit and integration tests:
```bash
cd build
make test
```

---

## Contributing

We welcome contributions! To contribute:
1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request.

---

## License

This library is licensed under the [MIT License](LICENSE).
```