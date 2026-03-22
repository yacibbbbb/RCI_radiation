import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    # 패키지 경로 설정
    radiation_sim_dir = get_package_share_directory('radiation_simulation')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # world 파일 경로
    world_file = os.path.join(radiation_sim_dir, 'worlds/reactor_room', 'reactor_room.world')

    # 로봇 경로
    integrated_urdf = os.path.join(radiation_sim_dir, 'urdf', 'tb3_with_RadSensor.urdf')
    sdf_file = os.path.join(radiation_sim_dir, 'urdf', 'tb3_with_RadSensor.sdf')

    # map data 설정 파일 경로
    world_name = 'reactor_room'
    path_tmp = world_name + '/map_data.yaml'
    map_data_config_file = os.path.join(radiation_sim_dir, 'config', path_tmp)
    with open(map_data_config_file, 'r') as f:
        full_data = yaml.safe_load(f)
    map_data = full_data.get('map_data', {})

    # RViz 설정 파일 경로
    rviz_config_file = os.path.join(radiation_sim_dir, 'rviz', 'radiation_visualization.rviz')
    
    # 환경변수 설정
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    os.environ['GZ_LOG_LEVEL'] = 'debug'
    
    # Gazebo 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbos': 'true' # 디버그 출력 활성화
        }.items()
    )
    
    # 통합된 로봇 상태 퍼블리셔 - urdf
    robot_state_publisher_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(integrated_urdf, 'r').read()
        }]
    )

    # 통합된 터틀봇 스폰 - sdf
    spawn_robot_sdf = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', sdf_file,
            '-entity', 'turtlebot3_with_radiation_sensor',
            '-x', '0', 
            '-y', '0', 
            '-z', '0.01'
        ],
        output='screen'
    )
    
    # map과 odom 프레임 연결 - 전역 좌표계 연결용
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 방사선 맵 생성기 노드
    radiation_map_generator = Node(
        package='radiation_simulation',
        executable='radiation_map_generator',
        name='radiation_map_generator',
        output='screen',
        parameters=[map_data]
    )

    # 방사선 시각화 노드
    radiation_visualization = Node(
        package='radiation_simulation',
        executable='radiation_visualization_node',
        name='radiation_visualization',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    trajectory_publisher = Node(
        package='radiation_simulation',
        executable='trajectory_publisher.py',
        name='trajectory_publisher',
        output='screen'
    )
    
    # RViz 실행
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        robot_state_publisher_urdf,
        spawn_robot_sdf,
        map_to_odom_tf, 
        radiation_map_generator,
        radiation_visualization,
        trajectory_publisher,
        rviz
    ])

# =================================== 매우 중요!!!! ===================================
# urdf파일 내부에 있는 가제보 플러그인이 아닌 일반 플러그인을 인식하지 못하는 문제 때문에,
# Spawn은 sdf파일로 진행하고, 로봇 상태 퍼블리싱 및 tf 변환은 urdf파일로 진행해야 함.
# 주의해야 할 점은 link 및 joint의 이름과 entity의 namespace가 동일해야 같은 개체로 운용 가능함.