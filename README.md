# radiation_simulation

ROS 2 and Gazebo-based radiation simulation and radiation-aware navigation package.

## Overview
This repository provides a ROS 2 package for simulating radiation-aware mobile robot operation in Gazebo.  
It includes custom Gazebo plugins for radiation sources, radiation sensors, and attenuating obstacles, along with radiation map generation, visualization, local/global radiation-aware costmap layers, and waypoint-based autonomous navigation.

The radiation-related Gazebo plugin implementation in this repository was developed with reference to prior work on simulating ionising radiation in Gazebo for robotic nuclear inspection.  
The layered costmap design for radiation-aware navigation was also informed by prior work on real-time radiation avoidance using layered costmaps for mobile robots.

## Dependencies
- ROS 2 Humble
- Python 3.10 / C++17
- Gazebo Classic / gazebo_ros
- Nav2
- RViz2
- TurtleBot3 description
- pluginlib
- tf2 / tf2_ros / tf2_geometry_msgs
- OpenCV

Recommended installation:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-rviz2 \
  ros-humble-turtlebot3-description \
  ros-humble-pluginlib \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher \
  libgazebo11-dev \
  qtbase5-dev \
  libopencv-dev
````

## Additional External Packages

This repository only contains the `radiation_simulation` package under `src/radiation_simulation`.

If you want to run frontier exploration experiments based on `m-explore-ros2` / `explore_lite`, install that package separately in your ROS 2 workspace instead of expecting it to be included in this repository.

## Installation

```bash
mkdir -p ~/radiation_simulation_ws/src
cd ~/radiation_simulation_ws/src
git clone https://github.com/<org>/<repo>.git

cd ~/radiation_simulation_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1) Gazebo radiation simulation + radiation map generation + visualization

```bash
ros2 launch radiation_simulation reactor_room_spawn.launch.py
```

### 2) Waypoint-based autonomous navigation

```bash
ros2 launch radiation_simulation reactor_room_waypoint.launch.py
```

## Examples

1. **Radiation Simulation** – TurtleBot3 with radiation source/sensor/obstacle plugins in Gazebo

   ```bash
   ros2 launch radiation_simulation reactor_room_spawn.launch.py
   ```

2. **Radiation-Aware Navigation** – Nav2 waypoint navigation with radiation-aware costmaps

   ```bash
   ros2 launch radiation_simulation reactor_room_waypoint.launch.py
   ```

## Repository Structure

```bash
<repo>/
└── src/
    └── radiation_simulation/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        ├── src/
        ├── include/
        ├── config/
        ├── rviz/
        ├── urdf/
        ├── models/
        ├── map/
        ├── data/
        └── scripts/
```

## Main Components

* **Radiation Source Plugin**
  Publishes source intensity from Gazebo models and allows runtime intensity updates.

* **Radiation Sensor Plugin**
  Detects radiation intensity using source distance, attenuation, and obstacle interaction, and publishes sensor pose and measurement topics.

* **Radiation Obstacle Plugin**
  Publishes attenuation coefficients for obstacle models based on material properties.

* **Radiation Map Generator**
  Builds a radiation-related occupancy/intensity representation and publishes visualization markers and map data.

* **Radiation Visualization Node**
  Displays source-wise and total detected radiation information for monitoring and debugging.

* **Radiation Local / Global Costmap Layers**
  Extends Nav2 layered costmaps with radiation-aware costs for avoidance behavior.

* **Waypoint Navigation**
  Executes waypoint-based autonomous navigation using Nav2 with radiation-aware planning support.

## Assets and Provenance

* The `reactor_room` world used in this repository is based on source code obtained from the work:
  **Simulating Ionising Radiation in Gazebo for Robotic Nuclear Inspection Challenges**.
* Radiation-related plugin and navigation components in this repository were developed by adapting and extending ideas from the references listed below.

## Notes

* This README intentionally focuses on simulation, plugins, radiation-aware costmaps, and autonomous navigation. 
* This repository is intended to contain only source files. 
* Some files may still contain machine-specific absolute paths. These should be converted to package-relative paths before sharing or deploying on another machine.
* Depending on your experiment setup, additional ROS 2 packages such as `m-explore-ros2` may be required separately.
* Some logs, messages, comments, or auxiliary outputs in this repository are written in **Korean**.

## References

The radiation-related plugin design was developed with reference to:

* Wright, T., West, A., Licata, M., Hawes, N., and Lennox, B.
  **Simulating Ionising Radiation in Gazebo for Robotic Nuclear Inspection Challenges**.
  *Robotics*, 2021, 10(86).

The radiation-aware navigation and layered costmap design was developed with reference to:

* West, A., Wright, T., Tsitsimpelis, I., Groves, K., Joyce, M. J., and Lennox, B.
  **Real-Time Avoidance of Ionising Radiation Using Layered Costmaps for Mobile Robots**.
  *Frontiers in Robotics and AI*, 2022, 9:862067.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contact

* Maintainer: jth8090 ([jth8090@khu.ac.kr](mailto:jth8090@khu.ac.kr))
* Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)

