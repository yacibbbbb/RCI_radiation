# radiation_simulation

ROS 2 and Gazebo-based radiation simulation and radiation-aware navigation package.

## Overview
This repository provides a ROS 2 package for simulating radiation-aware mobile robot operation in Gazebo.  
It includes custom Gazebo plugins for radiation sources, radiation sensors, and attenuating obstacles, along with radiation map generation, visualization, local/global radiation-aware costmap layers, and waypoint-based autonomous navigation.

The radiation-related Gazebo plugin implementation in this repository was developed with reference to prior work on simulating ionising radiation in Gazebo for robotic nuclear inspection.  
The layered costmap design for radiation-aware navigation was also informed by prior work on real-time radiation avoidance using layered costmaps for mobile robots.

This repository contains only the `radiation_simulation` package under `src/radiation_simulation`.

## Features
- Gazebo-based radiation source, sensor, and obstacle plugins
- Radiation map generation and visualization
- Radiation-aware local/global costmap layers for Nav2
- Waypoint-based autonomous navigation
- Preconfigured experiment worlds for `reactor_room` and `single_source`
- Radiation logging and post-processing utilities for cumulative exposure analysis

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

If you want to run frontier exploration experiments based on `m-explore-ros2` / `explore_lite`, install that package separately in your ROS 2 workspace instead of expecting it to be included in this repository.

## Installation

```bash
mkdir -p ~/radiation_simulation_ws/src
cd ~/radiation_simulation_ws/src
git clone https://github.com/RCILab/RCI_radiation.git

cd ~/radiation_simulation_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1) Reactor room simulation

<img width="2260" height="1243" alt="스크린샷 2025-05-15 22-43-25" src="https://github.com/user-attachments/assets/599dafd7-090a-4187-8b31-f3e06d55ac55" />

Launch the Gazebo simulation, robot model, radiation map generation, and visualization in the `reactor_room` world:

```bash
ros2 launch radiation_simulation reactor_room_spawn.launch.py
```

Run waypoint-based navigation in the `reactor_room` environment:

```bash
ros2 launch radiation_simulation reactor_room_waypoint.launch.py
```

### 2) Single-source simulation

<img width="1449" height="1165" alt="single_source" src="https://github.com/user-attachments/assets/cf116dca-a5e3-4560-9050-99d27c0faa32" />


Launch the simplified `single_source` world:

```bash
ros2 launch radiation_simulation single_source_spawn.launch.py
```

Run waypoint-based navigation and radiation logging in the `single_source` world:

```bash
ros2 launch radiation_simulation single_source_waypoint.launch.py
```


### 3) Optional additional launch files

The package also includes additional launch files such as:

```bash
ros2 launch radiation_simulation explore_radiation.launch.py
ros2 launch radiation_simulation map_sweeper.launch.py
```

## Experiment Worlds

### `reactor_room`

`reactor_room` is a more complex environment for radiation simulation and navigation experiments.
The `reactor_room` world assets included in this repository are based on source code obtained from:

**Simulating Ionising Radiation in Gazebo for Robotic Nuclear Inspection Challenges**

### `single_source`

`single_source` is a simplified rectangular-room world with a radiation source placed at the center of the room.
It is useful for more intuitive and visually interpretable demonstrations of radiation-aware navigation and avoidance behavior.

## Logging and Post-processing

### Radiation logging

`single_source_waypoint.launch.py` includes `scripts/radiation_logger.py`, which records radiation intensity and robot pose during navigation.

The logger stores CSV files under:

```bash
src/radiation_simulation/data/cumulation/<world_name>/
```

For the current `single_source` waypoint setup, the logged CSV is configured as:

```bash
data/cumulation/single_source/avoid_thres30_60deg_ver4.csv
```

The logger records:

* time
* radiation intensity
* robot position
* robot yaw

### Post-processing

You can analyze cumulative exposure results using:

```bash
python3 src/radiation_simulation/scripts/cumulation_analyzer.py
```

The analyzer is configured for the `single_source` experiment and compares:

* `no_avoid_vel10.csv`
* `avoid_thres30_60deg_ver4.csv`

It produces three plots:

1. **Time-Intensity**
2. **Trajectory**
3. **Time-Cumulative Exposure**

Example:  

<img width="1143" height="302" alt="Result" src="https://github.com/user-attachments/assets/fe91e2b4-7be4-41bc-a7d5-06439a53f004" />

## Repository Structure

```bash
RCI_radiation/
├── .gitignore
├── LICENSE
├── README.md
└── src/
    └── radiation_simulation/
        ├── CMakeLists.txt
        ├── package.xml
        ├── plugin_description.xml
        ├── config/
        ├── data/
        ├── include/
        ├── launch/
        ├── map/
        ├── models/
        ├── rviz/
        ├── scripts/
        ├── src/
        ├── urdf/
        └── worlds/
```

## Main Components

* **Radiation Source Plugin**
  Publishes source intensity from Gazebo models and supports runtime updates.

* **Radiation Sensor Plugin**
  Detects radiation intensity using source distance, attenuation, and obstacle interaction, and publishes measurement topics.

* **Radiation Obstacle Plugin**
  Publishes attenuation coefficients for obstacle models based on material properties.

* **Radiation Map Generator**
  Builds and publishes radiation-related map and visualization data.

* **Radiation Visualization Node**
  Displays total and source-wise detected radiation information.

* **Radiation Local / Global Costmap Layers**
  Extends Nav2 layered costmaps with radiation-aware cost information.

* **Waypoint Navigation**
  Executes waypoint-based autonomous navigation using Nav2.

## Notes

* This README intentionally focuses on simulation, plugins, radiation-aware costmaps, navigation, and logging utilities.
* Source localization-related components are not covered here.
* This repository is intended to contain source files only. Do **not** commit `build/`, `install/`, or `log/` directories.
* Some logs, messages, comments, or auxiliary outputs in this repository are written in **Korean**.

## References

The radiation-related plugin design was developed with reference to:

* Wright, T., West, A., Licata, M., Hawes, N., and Lennox, B.
  **Simulating Ionising Radiation in Gazebo for Robotic Nuclear Inspection Challenges**.
  *Robotics*, 2021, 10(3):86.
  DOI: 10.3390/robotics10030086

The radiation-aware navigation and layered costmap design was developed with reference to:

* West, A., Wright, T., Tsitsimpelis, I., Groves, K., Joyce, M. J., and Lennox, B.
  **Real-Time Avoidance of Ionising Radiation Using Layered Costmaps for Mobile Robots**.
  *Frontiers in Robotics and AI*, 2022, 9:862067.
  DOI: 10.3389/frobt.2022.862067

## Software Registration

The autonomous navigation simulation program for radiation map construction developed in this project has been **officially registered for copyright**.

<img width="698" height="642" alt="radiation_map" src="https://github.com/user-attachments/assets/420fd1fe-7779-40e9-af0f-7ec12ed5eec6" />

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

Maintainer: jth8090 ([jth8090@khu.ac.kr](mailto:jth8090@khu.ac.kr))  
Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)

