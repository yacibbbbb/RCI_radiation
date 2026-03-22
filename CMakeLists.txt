cmake_minimum_required(VERSION 3.8)
project(radiation_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt5Widgets REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-MATH6_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# source plugin
add_library(radiation_source_plugin SHARED
  src/radiation_source_plugin.cpp
)
ament_target_dependencies(radiation_source_plugin
  rclcpp
  gazebo_ros
  std_msgs
)

target_link_libraries(radiation_source_plugin ${GAZEBO_LIBRARIES})

# sensor plugin
add_library(radiation_sensor_plugin SHARED
  src/radiation_sensor_plugin.cpp
)
ament_target_dependencies(radiation_sensor_plugin
  rclcpp
  gazebo_ros
  std_msgs
  geometry_msgs
)
target_link_libraries(radiation_sensor_plugin 
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

# obstacle plugin
add_library(radiation_obstacle_plugin SHARED
  src/radiation_obstacle_plugin.cpp
)
ament_target_dependencies(radiation_obstacle_plugin
  rclcpp
  gazebo_ros
  std_msgs
)
target_link_libraries(radiation_obstacle_plugin ${GAZEBO_LIBRARIES})


# visualization node
add_executable(radiation_visualization_node
  src/radiation_visualization_node.cpp
)
ament_target_dependencies(radiation_visualization_node
  rclcpp
  std_msgs
)

# radiation_map_generator
add_executable(radiation_map_generator
  src/radiation_map_generator.cpp
)
ament_target_dependencies(radiation_map_generator
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(radiation_map_generator
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  gazebo
  gazebo_common
  gazebo_physics
  gazebo_transport
  gazebo_sensors
  ignition-math6
  tbb
)

# Radiation Static Layer
add_library(radiation_static_layer SHARED
  src/radiation_static_layer.cpp
)

ament_target_dependencies(radiation_static_layer
  rclcpp
  nav2_costmap_2d
)

target_link_libraries(radiation_static_layer
  ${rclcpp_LIBRARIES}
)

# Radiation Dynamic Layer
add_library(radiation_layer SHARED
  src/radiation_layer.cpp
)

ament_target_dependencies(radiation_layer
  rclcpp
  nav2_costmap_2d
  nav_msgs
  tf2_geometry_msgs
  pluginlib
  yaml-cpp
)

target_link_libraries(radiation_layer
  ${rclcpp_LIBRARIES}
  yaml-cpp
)

# radiation_virtual_sensor (world plugin)
add_library(radiation_virtual_sensor SHARED
  src/radiation_virtual_sensor.cpp
)

ament_target_dependencies(radiation_virtual_sensor
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
)

target_include_directories(radiation_virtual_sensor
  PRIVATE ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(radiation_virtual_sensor
  ${GAZEBO_LIBRARIES}
)

# plugin 등록
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_description.xml)

install(TARGETS
  radiation_source_plugin
  radiation_sensor_plugin
  radiation_obstacle_plugin
  radiation_static_layer
  radiation_layer
  radiation_visualization_node
  radiation_map_generator
  radiation_virtual_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  worlds
  urdf
  models
  rviz
  data
  map
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES 
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_nav.py
  scripts/radiation_logger.py
  scripts/convert_gp_to_Pgm_Yaml.py
  scripts/ground_overlay_publisher.py
  scripts/trajectory_publisher.py
  scripts/SL_sweeper_node.py
  scripts/obstacle_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
