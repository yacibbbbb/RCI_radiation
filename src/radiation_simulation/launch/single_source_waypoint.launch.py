from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():

    # 각종 경로 ----------------------------------------------

    world_name = 'single_source'

    # 패키지 경로 설정
    radiation_sim_dir = get_package_share_directory('radiation_simulation')
    origin_dir = Path(__file__).resolve().parents[5] 
    save_dir = os.path.join(origin_dir, "src/radiation_simulation")

    # nav2_parmas.yaml 경로
    nav2_params_path = os.path.join(radiation_sim_dir, 'config', world_name, 'nav2_params.yaml')

    # only obstacle
    map_path = os.path.join(radiation_sim_dir, 'map/obstacle', 'single_source.yaml')

    # merged map: obstacle + radiation >> static layer
    # map_path = os.path.join(radiation_sim_dir, 'map/merged', 'merged_map.yaml')

    # waypoint 순서
    waypoint_name = world_name + '.csv'
    waypoint_csv_path = os.path.join(radiation_sim_dir, 'data/waypoint', waypoint_name)

    radiation_static_map_yaml_path = os.path.join(radiation_sim_dir, 'map/obstacle', 'single_source.yaml')

    # radiation logger에서 csv 파일 저장 위치 + 측정 간격
    radiation_log_name = 'avoid_thres30_60deg_ver4'
    radiation_log_world_name = world_name + '/' + radiation_log_name + '.csv'
    radiation_log_csv_path = os.path.join(save_dir, 'data/cumulation', radiation_log_world_name)
    sample_dt = 0.2

    # ------------------------------------------------------

    # 1. Nav2 bringup 실행 노드
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'params_file': nav2_params_path
        }.items()
    )

    # 2. Waypoint 탐사 노드
    waypoint_nav_node = TimerAction(
        period=5.0,  # nav2 초기화 대기 시간 (초)
        actions=[
            Node(
                package='radiation_simulation',
                executable='waypoint_nav.py',
                name='waypoint_navigator',
                output='screen',
                arguments=[waypoint_csv_path]
            )
        ]
    )

    # 3. radiation 누적 로거
    radiation_logger = Node(
        package='radiation_simulation',
        executable='radiation_logger.py',
        name='radiation_logger',
        output='screen',
        parameters=[{
            'intensity_topic': '/radiation_sensor/detected_intensity',
            'pose_topic': '/odom',
            'use_odom': True,
            'sample_dt': sample_dt, # t 간격
            'csv_name':  radiation_log_csv_path,
            'listen_mission_done': True,
        }]
    )

    # 4. Static Layer 테스트 (radiation static map publish node)
    # Only Radiation Static Layer
    # nav2_params.yaml 수정해야 사용할 수 있음.
    radiation_static_map_publish_node = LoadComposableNodes(
        target_container='/nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='radiation_static_map_server',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': radiation_static_map_yaml_path
                }],
                remappings=[('/map', '/radiation_static_map')]
            )
        ]
    )

    radiation_static_map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_static_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['radiation_static_map_server']
        }]
    )

    radiation_static_map_publish_node_delayed = TimerAction(
        period=5.0,
        actions=[radiation_static_map_publish_node]
    )

    return LaunchDescription([
        nav2_bringup_launch,
        waypoint_nav_node,
        radiation_logger,
        # radiation_static_map_publish_node,
        # radiation_static_map_lifecycle_node
    ])
