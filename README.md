# Orbit Planner Package

A ROS2 package for autonomous exploration and monitoring in orchard environments using orbit-based navigation strategies.

## Overview

Orbit Planner is an autonomous exploration system designed specifically for orchard environments. It combines frontier-based exploration with tree row detection and orbit-based monitoring to ensure systematic coverage of orchard rows. The system integrates with LIO-SAM SLAM and 4-wheel steering control systems for complete autonomous operation.

## Key Features

### Core Capabilities
- **Tree Clustering & Row Detection**: PCL-based point cloud clustering with PCA for tree row orientation
- **Frontier-based Exploration**: Automatic detection of unexplored area boundaries
- **Orbit Anchor Generation**: 4-anchor system (Front, Back, Left, Right) for systematic row monitoring
- **Path Planning**: A* algorithm with ESDF-based collision avoidance
- **RViz2 Integration**: Interactive GUI for exploration area selection and monitoring
- **Real-time Performance**: Multi-threaded architecture for onboard operation

### Orchard-Specific Features
- **Tree Detection**: Individual tree identification and position estimation from point clouds
- **Row Structure Analysis**: Automatic recognition and analysis of orchard row patterns
- **Systematic Monitoring**: Row-by-row coverage with minimal blind spots
- **Adaptive Planning**: Dynamic strategy adjustment based on environmental structure

### Advanced Features
- **Voxblox Integration**: TSDF/ESDF mapping for real-time 3D environment representation
- **TSP Scheduling**: Traveling Salesman Problem optimization for efficient anchor sequencing
- **Receding Horizon Planning**: Short-term sequence evaluation for adaptive exploration
- **Multi-threaded Architecture**: Parallel processing for real-time performance

## System Architecture

### Core Components

1. **orbit_planner_node**: Main planner node handling exploration logic
2. **orbit_voxblox_interface**: Voxblox library interface for TSDF/ESDF mapping
3. **tree_clusterer**: Tree detection and row structure analysis
4. **frontier_detector**: Unexplored area boundary detection
5. **path_planner**: A* path planning with collision avoidance
6. **orbit_anchor_generator**: 4-anchor generation for systematic monitoring
7. **orbit_panel_plugin**: RViz2 panel plugin for interactive control

### Data Flow

```
LIO-SAM → Voxblox Interface → Planner Node → Control System
    ↓              ↓              ↓
Point Cloud → TSDF/ESDF → Tree Detection → Row Analysis
    ↓              ↓              ↓
Occupancy Grid → Frontier Detection → Anchor Generation
    ↓              ↓              ↓
Path Planning → TSP Scheduling → Trajectory Execution
```

## Installation and Build

### Dependencies

- **ROS2 Humble**
- **Voxblox**: TSDF/ESDF mapping library
- **PCL**: Point Cloud Library for 3D processing
- **RViz2**: Visualization tools
- **Eigen3**: Linear algebra library
- **OpenCV**: Computer vision library (optional)
- **tf2**: Transform library

### System Requirements

- Ubuntu 22.04 LTS
- ROS2 Humble
- Minimum 8GB RAM
- NVIDIA GPU (recommended for Voxblox)

### Installation

1. **Install ROS2 Humble** (if not already installed):
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. **Install dependencies**:
```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install ros-humble-rviz2 ros-humble-tf2-eigen
sudo apt install libpcl-dev libeigen3-dev libopencv-dev
```

3. **Clone and build**:
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Clone the package (if using git)
# git clone <repository_url> src/aomr/orbit_planner

# Build the package
colcon build --packages-select orbit_planner

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

```bash
# Launch the complete system
ros2 launch orbit_planner orbit_exploration.launch.py

# Launch without RViz2
ros2 launch orbit_planner orbit_exploration.launch.py use_rviz:=false
```

### Individual Node Execution

```bash
# Run only the planner node
ros2 run orbit_planner orbit_planner_node

# Run with custom parameters
ros2 run orbit_planner orbit_planner_node --ros-args -p robot_radius:=0.5 -p max_planning_distance:=100.0
```

### RViz2 Panel Usage

1. **Launch RViz2**:
```bash
ros2 run rviz2 rviz2 -d src/aomr/orbit_planner/rviz/orbit_planner.rviz
```

2. **Add the Orbit Planner Panel**:
   - Go to `Panels` → `Add New Panel` → `Orbit Planner Panel`
   - The panel will appear on the right side of RViz2

3. **Define Exploration Area**:
   - Click "Start Point" to select the robot's starting position
   - Click "Add Polygon Point" to enter polygon mode
   - Click on the map to define exploration boundaries
   - Click "Clear Polygon" to reset the area

4. **Start Exploration**:
   - Click "Start Exploration" to begin autonomous exploration
   - Monitor progress in the status panel
   - Click "Stop Exploration" to halt the system

### Testing the System

#### 1. **Unit Tests**
```bash
# Run all tests
colcon test --packages-select orbit_planner

# Run specific test
ros2 test src/aomr/orbit_planner/test/test_orbit_planner.cpp
```

#### 2. **Simulation Testing**
```bash
# Launch with simulation data
ros2 launch orbit_planner orbit_exploration.launch.py use_sim_time:=true

# Test with bag file
ros2 bag play your_data.bag
```

#### 3. **Visualization Testing**
```bash
# Check if all markers are published
ros2 topic list | grep orbit_planner

# Monitor specific topics
ros2 topic echo /orbit_planner/frontiers
ros2 topic echo /orbit_planner/orbit_anchors
ros2 topic echo /orbit_planner/trajectory
```

#### 4. **Performance Testing**
```bash
# Monitor CPU usage
htop

# Check memory usage
ros2 run orbit_planner orbit_planner_node --ros-args --log-level debug
```

## Configuration

### Parameter Settings

Key parameters can be configured in `config/orbit_planner_params.yaml`:

```yaml
# Robot parameters
robot_radius: 0.4
safety_margin: 0.1

# Exploration parameters
max_planning_distance: 50.0
frontier_cluster_min_size: 5.0
goal_tolerance: 1.0

# Tree detection parameters
tree_height_min: 0.4
tree_height_max: 0.7
tree_cluster_tolerance: 0.5
tree_min_cluster_size: 10

# Orbit anchor parameters
orbit_radius: 2.0
orbit_spacing: 1.0

# Path planning parameters
path_resolution: 0.1
path_smoothing_factor: 0.5

# Cost weights
yaw_change_weight: 0.5
frontier_gain_weight: 1.0
distance_weight: 1.0
```

### Topic Remapping

To integrate with different SLAM systems, remap topics:

```bash
ros2 launch orbit_planner orbit_exploration.launch.py \
  --ros-args -r /lio_sam/mapping/map_global:=/your_pointcloud_topic \
             -r /lio_sam/mapping/odometry:=/your_odometry_topic
```

## API Reference

### Services

- `/orbit_planner/start_exploration` (std_srvs/srv/Empty): Start autonomous exploration
- `/orbit_planner/stop_exploration` (std_srvs/srv/Empty): Stop exploration

### Subscribed Topics

- `/lio_sam/mapping/map_global` (sensor_msgs/msg/PointCloud2): Input point cloud from LIO-SAM
- `/lio_sam/mapping/odometry` (geometry_msgs/msg/PoseStamped): Robot pose from LIO-SAM
- `/orbit_planner/exploration_area` (geometry_msgs/msg/PolygonStamped): Exploration area definition

### Published Topics

- `/orbit_planner/trajectory` (nav_msgs/msg/Path): Planned exploration path
- `/orbit_planner/goal` (geometry_msgs/msg/PoseStamped): Current exploration goal
- `/orbit_planner/frontiers` (visualization_msgs/msg/MarkerArray): Frontier visualization
- `/orbit_planner/orbit_anchors` (visualization_msgs/msg/MarkerArray): Orbit anchor visualization
- `/orbit_planner/visited` (visualization_msgs/msg/MarkerArray): Visited goals visualization
- `/orbit_planner/occupancy_grid` (nav_msgs/msg/OccupancyGrid): Occupancy grid map

## Developer Guide

### Code Structure

```
orbit_planner/
├── src/                           # Source code
│   ├── orbit_planner_node.cpp     # Main planner node
│   ├── orbit_voxblox_interface.cpp # Voxblox integration
│   ├── tree_clusterer.cpp         # Tree detection & clustering
│   ├── frontier_detector.cpp      # Frontier detection
│   ├── path_planner.cpp           # Path planning algorithms
│   ├── orbit_anchor_generator.cpp # Orbit anchor generation
│   └── orbit_panel_plugin.cpp     # RViz2 panel plugin
├── include/orbit_planner/         # Header files
│   ├── orbit_planner_node.hpp
│   ├── orbit_voxblox_interface.hpp
│   ├── tree_clusterer.hpp
│   ├── frontier_detector.hpp
│   ├── path_planner.hpp
│   ├── orbit_anchor_generator.hpp
│   └── orbit_panel_plugin.hpp
├── launch/                        # Launch files
│   └── orbit_exploration.launch.py
├── config/                        # Configuration files
│   └── orbit_planner_params.yaml
├── rviz/                          # RViz configurations
│   ├── orbit_planner.rviz
│   └── orbit_panel_plugin.xml
└── test/                          # Unit tests
    └── test_orbit_planner.cpp
```

### Adding New Features

1. **Define Interface**: Add function declarations in header files
2. **Implement Logic**: Write implementation in corresponding `.cpp` files
3. **Update Build**: Add new sources to `CMakeLists.txt`
4. **Add Parameters**: Update configuration files with new parameters
5. **Write Tests**: Create unit tests for new functionality
6. **Update Documentation**: Update README and code comments

## Troubleshooting

### Common Issues

1. **Voxblox Compilation Error**: Check Eigen3 dependency installation
2. **RViz Plugin Load Failure**: Verify package build and environment setup
3. **Memory Issues**: Adjust voxel_size parameter in configuration
4. **TF Transform Errors**: Ensure proper frame_id configuration
5. **Point Cloud Processing Issues**: Check PCL library installation

### Debugging

```bash
# Set debug logging level
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Debug specific node
ros2 run orbit_planner orbit_planner_node --ros-args --log-level debug

# Check topic connections
ros2 topic list
ros2 topic info /orbit_planner/trajectory

# Monitor system performance
ros2 run orbit_planner orbit_planner_node --ros-args --log-level info
```

### Performance Optimization

1. **Reduce Voxel Size**: For higher resolution but more memory usage
2. **Adjust Planning Rate**: Lower frequency for less CPU usage
3. **Limit Exploration Distance**: Reduce max_planning_distance parameter
4. **Optimize Tree Detection**: Adjust clustering parameters for your environment

## License

MIT License

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## References

- [Voxblox: Incremental 3D Euclidean Signed Distance Fields for on-board MAV planning](https://github.com/ethz-asl/voxblox)
- [LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping](https://github.com/TixiaoShan/LIO-SAM)
- [ROS2 Navigation Stack](https://github.com/ros-planning/navigation2)
- [PCL: Point Cloud Library](https://pointclouds.org/)
- [RViz2: 3D Visualization Tool](https://github.com/ros2/rviz)

## Citation

If you use this package in your research, please cite:

```bibtex
@article{orbit_planner_2025,
  title={Orbit Planner: Map-Free Navigation for Orchard Monitoring and Mapping},
  author={Woo, Sangbeom and Kim, Duksu},
  journal={Computers and Electronics in Agriculture},
  year={2025}
}
```

## Contact

Development Team: developer@example.com

## Acknowledgments

- KoreaTech School of Computer Science and Engineering
- Active Orchard SLAM project team
- ROS2 community for excellent tools and documentation
