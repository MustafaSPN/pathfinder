import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

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
        # 5. SWEGEO RTK GPS SÜRÜCÜSÜ (Bağımsız Paketten)
        Node(
            package='swegeo_driver',      # <-- Yeni paket adı
            executable='swegeo_node',    # <-- setup.py'da verdiğimiz isim
            name='swegeo_gps_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',  # GPS'in bağlı olduğu port (Kontrol et!)
                'baudrate': 115200,
                'ntrip_enable': True,
                # NTRIP Bilgilerini Buraya Gir:
                'ntrip_host': '212.156.70.42', 
                'ntrip_port': 2101,
                'ntrip_mountpoint': 'VRSRTCM34',
                'ntrip_user': 'K0746003602',
                'ntrip_pass': 'GXihcS'
            }]
        ),

        # ---------------------------------------------
        # 3. LOKAL EKF (Pürüzsüz Sürüş - Odom Frame)
        # ---------------------------------------------
        # Girdi: /odom_esp (Tekerlek), /gps/imu (Heading)
        # Çıktı: /odometry/local
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom', # YAML dosyasındaki başlık ile AYNI OLMALI
            output='screen',
            parameters=[ekf_filter_node_odom_config],
            remappings=[
                ('odometry/filtered', '/odometry/local') # Çıktı topic ismini özelleştiriyoruz
            ]
        ),
        # ---------------------------------------------
        # 2. NAVSAT TRANSFORM (GPS -> Odometry Çevirici)
        # ---------------------------------------------
        # Girdi: /fix (GPS), /gps/imu (Heading)

        # Çıktı: /odometry/gps (Metre cinsinden GPS verisi)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',          # YAML ile aynı olsun
            output='screen',
            parameters=[navsat_transform_config],
            respawn=True,         # Hata verince tekrar başlat
            respawn_delay=4.0,    # Çökünce hemen açma, 4 saniye bekle (GPS iyice kendine gelsin)
            remappings=[
                ('imu', '/gps/imu'),
                ('gps/fix', '/fix'),
                ('odometry/filtered', '/odometry/local')
                ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        # ---------------------------------------------
        # 4. GLOBAL EKF (Harita Konumu - Map Frame)
        # ---------------------------------------------
        # Girdi: /odom_esp, /gps/imu, /odometry/gps (Navsat çıktısı)
        # Çıktı: /odometry/global
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map', # YAML dosyasındaki başlık ile AYNI OLMALI
            output='screen',
            parameters=[ekf_filter_node_map_config],
            remappings=[
                ('odometry/filtered', '/odometry/global') # Çıktı topic ismini özelleştiriyoruz
            ]
        )
    ])