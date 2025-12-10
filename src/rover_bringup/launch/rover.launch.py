import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'rover_bringup'
    
    # Dosya yollarını bul
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'rover.urdf')
    ekf_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')

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

        # 2. Odometriyi TF'e Çevir (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        Node(
            package='rover_bringup',
            executable='empty_map_pub',
            name='empty_map_publisher',
            output='screen'
        ),

        # 3. Sahte Harita Bağlantısı (Map -> Odom)
        # Lidar/SLAM olmadığı için haritayı odometriye çiviliyoruz.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    ])