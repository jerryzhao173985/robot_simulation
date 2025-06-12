# VSG-ODE Robot Simulation

An advanced hexapod robot simulation using VulkanSceneGraph (VSG) for graphics and Open Dynamics Engine (ODE) for physics.

## Features

### Robot Capabilities
- **Hexapod Design**: Six-legged robot with 3 segments per leg
- **Advanced Gait Patterns**: Walk, trot, and gallop gaits with smooth transitions
- **Dynamic Stabilization**: Real-time balance control using PID controllers
- **Adaptive Movement**: Terrain-aware locomotion with automatic gait optimization
- **Sensor System**: Proximity sensors, gyroscope, accelerometer, and contact sensors

### Control Modes
- **Manual Control**: Direct motor command interface
- **Autonomous Navigation**: Path planning with obstacle avoidance
- **Learning Mode**: Adaptive behavior learning system
- **Demonstration Mode**: Record and replay movement patterns

### Graphics Features (VSG)
- **High-Performance Rendering**: Vulkan-based scene graph
- **Dynamic Lighting**: Directional, point, and spot lights with shadows
- **Post-Processing Effects**: Bloom, SSAO, motion blur
- **Advanced Materials**: PBR materials with metallic/roughness workflow
- **Camera Modes**: Free, follow, and orbit camera controls
- **Debug Visualization**: Real-time physics and sensor visualization

### Physics Features (ODE)
- **Realistic Dynamics**: Full rigid body simulation for all robot parts
- **Joint Constraints**: Ball joints for hips, hinge joints for knees/ankles
- **Collision Detection**: Accurate mesh-based terrain collision
- **Contact Forces**: Configurable friction and restitution
- **Adaptive Stepping**: Variable time step for stability

### Terrain System
- **Procedural Generation**: Multiple terrain types (flat, hills, mountains, rough, stairs)
- **Dynamic Modification**: Real-time terrain deformation
- **Physics Integration**: Trimesh collision for accurate ground interaction
- **Environmental Features**: Vegetation placement (grass, trees, rocks)
- **Height Queries**: Efficient terrain sampling for navigation

## Prerequisites

- C++17 compatible compiler
- CMake 3.16 or higher
- VulkanSceneGraph (VSG)
- vsgXchange
- Open Dynamics Engine (ODE)
- Vulkan SDK

## Building

### Ubuntu/Debian

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libvulkan-dev \
    libode-dev \
    libglm-dev

# Clone and build VSG (if not installed)
git clone https://github.com/vsg-dev/VulkanSceneGraph.git
cd VulkanSceneGraph
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build

# Clone and build vsgXchange
cd ..
git clone https://github.com/vsg-dev/vsgXchange.git
cd vsgXchange
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build

# Build the robot simulation
cd ../robot_simulation
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### macOS

```bash
# Install dependencies using Homebrew
brew install cmake vulkan-sdk ode

# Build VSG and vsgXchange as above
# Then build the robot simulation
```

### Windows

```powershell
# Use vcpkg or manually install dependencies
# Build with Visual Studio or MinGW
```

## Running

```bash
cd build
./vsg_ode_robot
```

## Controls

The simulation runs autonomously by default, with the robot navigating to random goals while avoiding obstacles. Control modes automatically cycle through:

1. **Autonomous Mode**: Robot navigates using path planning
2. **Learning Mode**: Robot explores and learns from experience
3. **Demonstration Mode**: Robot replays recorded patterns

### Keyboard Controls (when implemented)
- `W/A/S/D`: Manual movement control
- `Space`: Jump
- `C`: Crouch
- `1-3`: Switch gait (walk/trot/gallop)
- `Tab`: Toggle camera mode
- `F1`: Show/hide statistics
- `F2`: Show/hide debug visualization
- `ESC`: Exit

## Architecture

### Core Components

1. **Robot**: Main robot class managing body, legs, and sensors
2. **PhysicsWorld**: ODE physics simulation wrapper
3. **Visualizer**: VSG rendering and window management
4. **RobotController**: High-level control algorithms
5. **Terrain**: Procedural terrain generation and physics

### Design Patterns

- **Component-Based**: Modular design for easy extension
- **Observer Pattern**: Event-driven sensor updates
- **Strategy Pattern**: Swappable control algorithms
- **Factory Pattern**: Dynamic object creation

## Performance

The simulation is optimized for:
- 60 Hz physics updates
- Multi-threaded rendering
- Efficient collision detection
- LOD terrain rendering
- Instanced vegetation rendering

## Extending

### Adding New Gaits
1. Modify `Robot::updateGait()` in `Robot.cpp`
2. Add gait parameters to `Robot.h`
3. Update `RobotController::optimizeGait()`

### Adding Sensors
1. Add sensor type to `Robot::Sensor::Type`
2. Implement sensor in `Robot::createSensors()`
3. Update `Robot::getSensorReadings()`

### Custom Terrain
1. Add terrain type to `Terrain::TerrainType`
2. Implement generation in `Terrain.cpp`
3. Update physics mesh generation

## Troubleshooting

### Common Issues

1. **Vulkan not found**: Ensure Vulkan SDK is installed and in PATH
2. **ODE linking errors**: Check ODE installation and CMake configuration
3. **Performance issues**: Reduce terrain resolution or disable vegetation
4. **Robot instability**: Adjust PID parameters in RobotController

### Debug Options

- Enable physics debug visualization
- Check console output for sensor readings
- Monitor energy consumption and stability metrics

## License

This project is provided as-is for educational and research purposes.

## Acknowledgments

- VulkanSceneGraph developers
- Open Dynamics Engine community
- Robotics research community