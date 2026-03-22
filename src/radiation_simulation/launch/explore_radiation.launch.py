from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sim = get_package_share_directory('radiation_simulation')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_explore = get_package_share_directory('explore_lite')

    world_name = 'reactor_room'
    # world_name = 'single_source'

    path_tmp = world_name + '/nav2_params.yaml'

    nav2_params_yaml_path = os.path.join(pkg_sim, 'config', path_tmp)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_sim, 'maps', 'dummy.yaml'),  # placeholder for nav2
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        # SLAM Toolbox
        # 기존: ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Navigation2 bringup (planner, controller, costmap 등 포함)
        # 위에서 params_file만 custom된 radiation_simulation 패키지 내부의 yaml 이용
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'map': map_file,
                'params_file': nav2_params_yaml_path      # custom된 yaml 파일
            }.items()
        ),

        # Explore Lite 자동 탐색
        # 기존: ros2 launch explore_lite explore.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_explore, 'launch', 'explore.launch.py')
        #     ),
        #     launch_arguments={'use_sim_time': use_sim_time}.items()
        # ),
    ])
