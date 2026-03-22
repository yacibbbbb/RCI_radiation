import os, csv, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _load_sources(csv_path):
        names, xs, ys = [], [], []
        try:
            with open(csv_path, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    names.append(row['name'])
                    xs.append(float(row['x']))
                    ys.append(float(row['y']))
        except FileNotFoundError:
            # 파일이 없으면 빈 배열 전달 (노드에서 경고하도록)
            pass
        return names, xs, ys

def generate_launch_description():
    pkg = get_package_share_directory('radiation_simulation')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world_name = 'reactor_room'
    # world_name = 'single_source'

    # world 경로 설정
    world = os.path.join(pkg, 'worlds', 'reactor_room', 'reactor_room.world')
    # world = os.path.join(pkg, 'worlds', 'single_source.world')

    # RViz 경로 설정
    rviz_cfg = os.path.join(pkg, 'rviz', 'radiation_visualization.rviz')

    # Source 목록 로딩
    default_sources_csv = os.path.join(pkg, 'data', world_name , 'sources.csv')
    source_csv_arg = DeclareLaunchArgument(
        'source_csv',
        default_value=default_sources_csv,
        description='CSV file containing source list with columns: name,x,y'
    )
    source_csv = LaunchConfiguration('source_csv')
    src_names, src_xs, src_ys = _load_sources(default_sources_csv)

    # map_data.yaml 파라미터
    map_yaml = os.path.join(pkg, 'config',  world_name, 'map_data.yaml')
    with open(map_yaml, 'r') as f:
        full = yaml.safe_load(f) or {}
    map_data = dict(full.get('map_data', {}))

    # idw.yaml 파라미터
    idw_yaml = os.path.join(pkg, 'config', world_name, 'IDW.yaml')
    with open(idw_yaml, 'r') as f:
        idw_data = yaml.safe_load(f)
    idw_params = idw_data.get('idw', {})

    # occupancy grid 경로
    occ_map_yaml = os.path.join(pkg, 'map', 'obstacle', f'{world_name}.yaml')

    # 기본값
    sweeper_params = {
        'use_sim_time': True,
        'world_name': world_name,
        'map_frame':            map_data.get('map_frame', 'map'),
        'radiation_sensor_frame': map_data.get('radiation_sensor_frame', 'radiation_sensor_link'),
        'map_resolution':       float(map_data.get('map_resolution', 0.25)),
        'map_width':            float(map_data.get('map_width', 19.0)),
        'map_height':           float(map_data.get('map_height', 20.0)),
        'map_origin_x':         float(map_data.get('map_origin_x', -9.5)),
        'map_origin_y':         float(map_data.get('map_origin_y', -10.0)),
        'map_update_rate':      float(map_data.get('map_update_rate', 5.0)),
        'max_intensity':        float(map_data.get('max_intensity', 150.0)),

        # 마커 표시 옵션(없으면 기본)
        'marker_ns':            map_data.get('marker_ns', 'rad_sweep'),
        'marker_stride':        int(map_data.get('marker_stride', 1)),
        'marker_fixed_z':       float(map_data.get('marker_fixed_z', 0.05)),
        'publish_markers':      True,

        # 가상 센서 I/O 토픽(radiation_virtual_sensor.cpp에서의 토픽과 동일해야 함.)
        'virtual_pose_topic':        '/virtual_sensor/pose',
        'virtual_intensity_topic':   '/virtual_sensor/detected_intensity',
    }

    # ================= Map Server (nav2_map_server) =================
    # /reactor_map 토픽으로 OccupancyGrid 퍼블리시
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': occ_map_yaml,
            'topic_name': 'reactor_map',
        }]
    )

    # ================= Lifecycle Manager (map_server 활성화) =================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )


    # --- Env: 플러그인 탐색 경로만 잡기
    set_gz_plugins = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.environ.get('GAZEBO_PLUGIN_PATH', '') +
              (':' if os.environ.get('GAZEBO_PLUGIN_PATH') else '') +
              os.path.join(os.path.dirname(pkg), 'install', 'radiation_simulation', 'lib')
    )

    # --- Gazebo
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': 'false'
            }.items()
    )

    # --- Sweeper (가상 센서 포즈 쏘고 intensity 수집/마킹)
    sweeper = Node(
        package='radiation_simulation',
        executable='SL_sweeper_node.py',
        name='radiation_sweeper',
        output='screen',
        parameters=[sweeper_params]
    )

    # --- 장애물 마커 퍼블리셔
    obstacle_loader = Node(
        package='radiation_simulation',
        executable='obstacle_marker_publisher.py',
        name='obstacle_marker_publisher',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'world_path': world,
            'marker_topic': '/obstacle_markers',
            'ns': 'obstacles',
            'color_rgba': [0.3, 0.3, 0.3, 0.8],
        }]
    )

    # --- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=None
    )

    delayed_sweeper = TimerAction(
         period=5.0,
         actions=[sweeper]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        set_gz_plugins,
        gz,
        map_server,
        lifecycle_manager,
        delayed_sweeper,
        obstacle_loader,
        rviz
    ])
