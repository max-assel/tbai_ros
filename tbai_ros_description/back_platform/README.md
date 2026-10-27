# Back Platform URDF Package

## Description

This ROS2 package contains the URDF (Unified Robot Description Format) description for the back_platform robot. It includes 3D models, configuration files, and launch files for visualization and simulation.

## Package Structure

```
back_platform/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package dependencies and metadata
├── README.md              # This file
├── config/                # Configuration files
│   └── joint_names_backPlatform.yaml
├── launch/                # Launch files
│   ├── display.launch.py  # ROS2 visualization launch file
│   ├── gazebo.launch.py   # ROS2 Gazebo simulation launch file
│   ├── display.launch     # Legacy ROS1 launch file (deprecated)
│   └── gazebo.launch      # Legacy ROS1 launch file (deprecated)
├── meshes/                # 3D mesh files
│   ├── base_link.STL
│   ├── camera_link.STL
│   └── lidar_link.STL
├── rviz/                  # RViz configuration
│   └── back_platform.rviz
├── textures/              # Texture files (if any)
└── urdf/                  # URDF description files
    ├── back_platform.urdf
    └── back_platform.csv
```

## Robot Description

The back_platform robot consists of the following components:

- **base_link**: Main platform base
- **camera_link**: Camera sensor mount
- **lidar_link**: LiDAR sensor mount

The robot includes fixed joints connecting the sensors to the base platform.

## Prerequisites

Make sure you have ROS2 installed with the following packages:

```bash
sudo apt install ros-<ros2-distro>-robot-state-publisher \
                 ros-<ros2-distro>-joint-state-publisher \
                 ros-<ros2-distro>-joint-state-publisher-gui \
                 ros-<ros2-distro>-rviz2 \
                 ros-<ros2-distro>-gazebo-ros-pkgs \
                 ros-<ros2-distro>-xacro
```

Replace `<ros2-distro>` with your ROS2 distribution (e.g., `humble`, `iron`, `jazzy`).

## Building the Package

1. Navigate to your ROS2 workspace:
   ```bash
   cd /path/to/your/ros2_ws
   ```

2. Build the package:
   ```bash
   colcon build --packages-select back_platform
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Visualization in RViz2

To launch the robot description in RViz2 for visualization:

```bash
ros2 launch back_platform display.launch.py
```

Optional parameters:
- `gui:=false` - Disable joint state publisher GUI
- `rviz:=false` - Don't start RViz2
- `use_sim_time:=true` - Use simulation time (for Gazebo)

Example with parameters:
```bash
ros2 launch back_platform display.launch.py gui:=false rviz:=true
```

### Gazebo Simulation

To launch the robot in Gazebo simulation:

```bash
ros2 launch back_platform gazebo.launch.py
```

Optional parameters:
- `world:=/path/to/world.sdf` - Load a specific world file
- `x_pose:=1.0` - Set initial x position
- `y_pose:=2.0` - Set initial y position
- `z_pose:=0.1` - Set initial z position

Example with parameters:
```bash
ros2 launch back_platform gazebo.launch.py x_pose:=1.0 y_pose:=1.0
```

### Manual Robot State Publishing

If you want to manually publish the robot description:

```bash
# Publish the robot description
ros2 param set /robot_state_publisher robot_description "$(cat install/back_platform/share/back_platform/urdf/back_platform.urdf)"

# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher

# Start joint state publisher (in another terminal)
ros2 run joint_state_publisher joint_state_publisher
```

## ROS2 Topics and Services

When the launch files are running, the following topics are available:

### Topics
- `/robot_description` - Robot URDF description
- `/joint_states` - Joint state information
- `/tf` - Transform tree
- `/tf_static` - Static transforms

### Services
- Various robot_state_publisher services for updating parameters

## Configuration

### Joint Names Configuration
The joint names are defined in `config/joint_names_backPlatform.yaml`. This file can be used to configure joint controllers if needed.

### RViz Configuration
The default RViz configuration is stored in `rviz/back_platform.rviz`. You can save your own custom configurations and specify them when launching:

```bash
ros2 run rviz2 rviz2 -d /path/to/custom_config.rviz
```

## Customization

### Modifying the URDF
The main URDF file is located at `urdf/back_platform.urdf`. You can modify this file to:
- Add new links and joints
- Change physical properties (mass, inertia)
- Update visual and collision meshes
- Add sensors or actuators

### Adding New Meshes
Place new STL files in the `meshes/` directory and reference them in the URDF:
```xml
<mesh filename="package://back_platform/meshes/your_new_mesh.STL" />
```

### Creating Custom Launch Files
You can create custom launch files in the `launch/` directory. Use the existing files as templates.

## Troubleshooting

### Common Issues

1. **Package not found error**:
   ```bash
   # Make sure you've built and sourced the workspace
   colcon build --packages-select back_platform
   source install/setup.bash
   ```

2. **Mesh files not loading**:
   - Check that mesh files exist in the `meshes/` directory
   - Verify the file paths in the URDF are correct
   - Ensure package name matches in URDF (`package://back_platform/...`)

3. **RViz2 not showing robot**:
   - Check that `/robot_description` topic is being published
   - Verify the Fixed Frame is set to `base_link` in RViz2
   - Make sure RobotModel display is enabled and configured correctly

4. **Gazebo not spawning robot**:
   - Check that Gazebo is properly installed
   - Verify the robot description is valid
   - Look at Gazebo console for error messages

### Debugging Commands

```bash
# Check if robot description is published
ros2 topic echo /robot_description

# List all active topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames

# Validate URDF syntax
check_urdf install/back_platform/share/back_platform/urdf/back_platform.urdf
```

## Contributing

When contributing to this package:

1. Follow ROS2 naming conventions
2. Update this README if adding new features
3. Test both visualization and simulation launch files
4. Ensure URDF validation passes

## License

This package is licensed under the BSD-3-Clause License.

## Maintainer

- **Maintainer**: TODO <rcao73@gatech.edu>

## Dependencies

- ROS2 (Humble/Iron/Jazzy or later)
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- rviz2
- gazebo_ros_pkgs
- xacro

For the complete list of dependencies, see `package.xml`.