# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### macOS (Primary platform)
```bash
# Automated build (recommended)
./build.sh

# Manual build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
make -j$(sysctl -n hw.ncpu)

# Run the simulation
cd build && ./vsg_ode_robot
# Or use the generated script with proper environment
./run_robot.sh
```

### Linux
```bash
# Install dependencies
sudo apt-get install -y build-essential cmake libode-dev libglfw3-dev libgl1-mesa-dev libvulkan-dev

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## High-Level Architecture

This is a sophisticated hexapod robot simulation combining **VulkanSceneGraph (VSG)** for graphics and **Open Dynamics Engine (ODE)** for physics, with automatic fallback to OpenGL/GLFW when VSG is unavailable.

### Core Design Patterns

1. **Cross-Platform Graphics Backend**
   - Code uses `#ifdef USE_OPENGL_FALLBACK` to switch between VSG and OpenGL
   - All graphics code is abstracted through the `Visualizer` class
   - Type aliases (`vsg_vec3`, `vsg_quat`) ensure cross-platform compatibility

2. **Component Architecture**
   - `Robot` (src/Robot.cpp): Hexapod with 6 legs, 3 segments per leg, manages ODE bodies/joints
   - `PhysicsWorld` (src/PhysicsWorld.cpp): ODE wrapper handling collision detection at 240Hz
   - `RobotController` (src/RobotController.cpp): High-level control with multiple modes (Manual, Autonomous, Learning, Demo)
   - `Terrain` (src/Terrain.cpp): Procedural terrain with trimesh collision
   - `InputHandler` (src/InputHandler.cpp): Keyboard/mouse input processing
   - `NoiseManager` (src/NoiseManager.cpp): Sensor noise modeling system
   - `Sensor`/`Actuator` (src/Sensor.cpp, src/Actuator.cpp): Hardware abstraction with observer pattern

3. **Physics-Graphics Synchronization**
   - Physics runs at 240Hz for stability
   - Each ODE body has a corresponding `vsg::MatrixTransform` node
   - Per-frame sync: ODE world position/rotation → VSG transform matrix
   - Robot leg visuals are updated in `Robot::updateLegPositions()`

### Key Conventions

1. **Coordinate System**: Z-up throughout the codebase
   - Helper function: `makePosition(x, zUp, yForward)` in include/PositionUtils.h
   - Camera math consistently uses Z-up convention

2. **Contact Point API**: 
   - `getContactPoints(dGeomID geom)` returns contacts with normals pointing outward from the queried geometry
   - Used in `RobotController::maintainBalance()` for foot contact forces

3. **Build System Intelligence**:
   - Auto-detects VSG/Vulkan availability
   - Falls back to OpenGL if VSG not found
   - Installs missing dependencies via Homebrew on macOS
   - Handles Vulkan SDK environment setup automatically

### Control Modes

The simulation cycles through different control modes:
- **Autonomous**: Path planning with obstacle avoidance
- **Learning**: Adaptive behavior exploration
- **Demonstration**: Record and replay movement patterns
- **Manual** (future): Direct motor control via keyboard

### Performance Considerations

- Physics: 240Hz update rate for stable joint dynamics
- Graphics: Target 60Hz with V-Sync disabled by default
- Known issue: Low FPS on macOS with VSG (0.43 FPS) - OpenGL fallback recommended for macOS

### Common Development Tasks

When modifying physics:
- Update both `PhysicsWorld` for ODE integration
- Update visualization in both VSG and OpenGL backends in `Visualizer`
- Ensure Z-up convention is maintained

When adding new features:
- Follow the component-based architecture
- Use observer pattern for event-driven updates
- Implement for both graphics backends using conditional compilation

### Testing

Currently no formal test framework. Ad-hoc test files exist:
- `src/test_vsg.cpp`, `src/minimal_vsg_test.cpp`: Graphics tests
- `test_noise_debug.sh`, `test_keys.sh`: System tests

### Debugging

- Enable `ComprehensiveEventDebugger` for detailed event logging
- Physics debug visualization available in both graphics backends
- Check console output for active graphics backend ("VSG renderer" vs "OpenGL/GLFW fallback")