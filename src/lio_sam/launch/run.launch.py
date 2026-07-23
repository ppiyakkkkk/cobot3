import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params_rs16.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    # LIO-SAM의 map 프레임 원점은 SLAM 시작 지점인데, GPS factor가 없으면
    # world 절대좌표와 안 맞는다. sim_config.py의 DRONE_CONFIGS 스폰 좌표를
    # world -> map 정적 TF로 넣어 정렬한다. 정지 비행 테스트 때는 SLAM 시작
    # 지점이 곧 스폰 위치였지만, 실제 임무 bag은 SEARCHING 진입 시점(이륙+
    # 호버링 완료 후)부터 녹화되므로 z만 이륙 고도(기본 6.0m)만큼 더해야
    # 한다(x,y/회전은 그 시점에도 스폰과 거의 동일함을 실제 로그로 확인함).
    # 기본값은 quadrotor_01 스폰+이륙 위치([-34.0, 40.0, 37.0], 회전 없음).
    world_x = LaunchConfiguration('world_x')
    world_y = LaunchConfiguration('world_y')
    world_z = LaunchConfiguration('world_z')
    world_origin_declare = [
        DeclareLaunchArgument('world_x', default_value='-34.0'),
        DeclareLaunchArgument('world_y', default_value='40.0'),
        # 31.0(스폰 z) + 6.0(이륙 고도) = 37.0. bag 녹화는 SEARCHING 진입
        # 시점(이륙+호버링 완료 후)부터라 map 원점이 스폰 위치가 아니다.
        DeclareLaunchArgument('world_z', default_value='37.0'),
    ]

    # imuTopic도 params_isaacsim.yaml엔 quadrotor_01로 고정되어 있어서,
    # 다른 드론으로 테스트하면 라이다는 그 드론 것인데 IMU는 계속
    # quadrotor_01 것을 쓰는 문제가 있었다. 런치 인자로 드론별 지정 가능하게 뺀다.
    imu_topic = LaunchConfiguration('imu_topic')
    imu_topic_declare = DeclareLaunchArgument(
        'imu_topic', default_value='/quadrotor_01/imu/data')

    # 라이브 Isaac Sim 실시간 테스트는 지금까지처럼 false(wall time)로 두고,
    # bag 재생으로 지도를 만들 때만 true로 넘겨 bag의 /clock을 따라가게 한다.
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # robot.urdf.xacro의 base_link/chassis_link/imu_link/laser_sensor_frame/navsat_link는
    # RViz TF축 표시 외에 쓰는 곳이 없고, robot_state_publisher가 정적으로 방송하는
    # base_link->lidar_link가 lidarFrame==baselinkFrame(params_isaacsim.yaml)일 때
    # TransformFusion의 동적 odom->lidar_link와 부모가 겹쳐 충돌한다. isaacsim 경로에서는
    # 꺼서 이 충돌을 없앤다. 지상로봇용 params(params.yaml/params_rs16.yaml)에서는 기본값
    # true로 기존 동작을 그대로 유지한다.
    publish_robot_urdf = LaunchConfiguration('publish_robot_urdf')
    publish_robot_urdf_declare = DeclareLaunchArgument(
        'publish_robot_urdf', default_value='true')

    # {drone_name}/base_scan(Isaac Sim 원본 point cloud의 frame_id)은 lidar_link와
    # 물리적으로 같은 라이다 위치라 identity static TF 하나로 map까지 연결한다.
    drone_name = LaunchConfiguration('drone_name')
    drone_name_declare = DeclareLaunchArgument(
        'drone_name', default_value='quadrotor_01')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        *world_origin_declare,
        imu_topic_declare,
        use_sim_time_declare,
        publish_robot_urdf_declare,
        drone_name_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                world_x, world_y, world_z, '0.0', '0.0', '0.0',
                'world', 'map',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            condition=IfCondition(publish_robot_urdf),
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path]),
                'use_sim_time': use_sim_time,
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                'lidar_link', [drone_name, '/base_scan'],
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
            ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file, {'imuTopic': imu_topic, 'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file, {'imuTopic': imu_topic, 'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_simpleGpsOdom',
            name='lio_sam_simpleGpsOdom',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
