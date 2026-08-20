import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ép SLAM Toolbox trỏ đích danh đến file config của bạn
    ld_slam_dir = get_package_share_directory('ld_slam_py')
    slam_config_file = os.path.join(ld_slam_dir, 'config', 'mapper_params_online_async.yaml')
    ekf_config_file = os.path.join(ld_slam_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. Khởi động Não bộ Nhúng (ESP32 Kinematics & Odom (Phát /wheel/odom và /imu/data))
        Node(
            package='ld_parser_cpp',
            executable='base_controller_node',
            name='base_controller_node',
            output='screen'
        ),

        # 2. Khởi động Mắt thần (LiDAR LD14)
        Node(
            package='ld_parser_cpp',
            executable='raw_data_node',
            name='raw_data_node',
            output='screen'
        ),

	# 3. TF Tĩnh
        # TF: Từ base_link lên IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0.031', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # TF: Từ base_link lên LiDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['-0.0124', '0', '0.076', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        
        # 4. NODE EKF (Chạy file config ekf.yaml)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file]
        ),

        # 5. KHỞI ĐỘNG SLAM Toolbox TRỰC TIẾP
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox', # Tên node bắt buộc phải khớp với tên trong file YAML
            parameters=[slam_config_file, {'use_sim_time': False}],
            output='screen'
        )
    ])
