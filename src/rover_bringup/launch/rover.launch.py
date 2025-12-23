import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction  # <--- 1. BU IMPORT'U EKLE

def generate_launch_description():
    pkg_name = 'rover_bringup'
    
    # Dosya yollarını bul
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'rover.urdf')
    ekf_filter_node_odom_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf_filter_node_odom.yaml')
    ekf_filter_node_map_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf_filter_node_map.yaml')
    navsat_transform_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'navsat_transform.yaml')
    twist_mux_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'twist_mux.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # ---------------------------------------------
    # NAVSAT NODE TANIMINI BURAYA ALDIK (Listenin dışına)
    # ---------------------------------------------
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_config,
                    {'yaw_offset': 0.0}], 
        respawn=True,
        respawn_delay=4.0,
        remappings=[
            ('imu', '/gps/imu'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/local')
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ---------------------------------------------
    # GECİKME MEKANİZMASI (TIMER ACTION)
    # ---------------------------------------------
    delayed_navsat_transform = TimerAction(
        period=5.0,  # <-- 5 Saniye Gecikme (GPS ve IMU otursun diye)
        actions=[navsat_transform_node]
    )

    return LaunchDescription([
        # 1. Robot Modelini Yayınla (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config]
        ),
        Node(
            package='rover_bringup',
            executable='empty_map_pub',
            name='empty_map_publisher',
            output='screen'
        ),
        # 5. SWEGEO RTK GPS SÜRÜCÜSÜ
        Node(
            package='swegeo_driver',
            executable='swegeo_node',
            name='swegeo_gps_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'ntrip_enable': True,
                'ntrip_host': '212.156.70.42', 
                'ntrip_port': 2101,
                'ntrip_mountpoint': 'VRSRTCM34',
                'ntrip_user': 'K0746003602',
                'ntrip_pass': 'GXihcS'
            }]
        ),

        # ---------------------------------------------
        # 3. LOKAL EKF 
        # ---------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_filter_node_odom_config],
            remappings=[
                ('odometry/filtered', '/odometry/local')
            ]
        ),
        
        # ---------------------------------------------
        # 2. NAVSAT TRANSFORM (Buraya artık Node yerine Delayed değişkeni geliyor)
        # ---------------------------------------------
        delayed_navsat_transform,  # <--- LİSTEYE NODE DEĞİL, TIMER EKLENDİ

        # ---------------------------------------------
        # 4. GLOBAL EKF
        # ---------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_filter_node_map_config],
            remappings=[
                ('odometry/filtered', '/odometry/global')
            ]
        )
    ])