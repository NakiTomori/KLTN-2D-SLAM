import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld_slam_dir = get_package_share_directory('ld_slam_py')
    slam_config_file = os.path.join(ld_slam_dir, 'config', 'mapper_params_online_async.yaml')

    # ============================================================
    # CẤU HÌNH EKF CHO TRƯỜNG HỢP 1: CHỈ DÙNG ENCODER (BỎ QUA IMU)
    # ============================================================
    ekf_params_case1 = {
        'frequency': 30.0,
        'two_d_mode': True,
        'publish_tf': True,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        
        # CHỈ ĐỌC DỮ LIỆU TỪ BÁNH XE
        'odom0': '/wheel/odom',
        'odom0_config': [False, False, False,
                         False, False, False,
                         True,  False, False,  # Bật thu thập Linear X
                         False, False, True,   # Bật thu thập Angular Z (Từ bánh xe)
                         False, False, False]
                         
        # HOÀN TOÀN KHÔNG KHAI BÁO imu0 Ở ĐÂY.
        # Hệ thống sẽ phớt lờ mọi gói tin đến từ topic /imu/data.
    }

    return LaunchDescription([
        # 1. ESP32 (Phát /wheel/odom và /imu/data)
        Node(
            package='ld_parser_cpp',
            executable='base_controller_node',
            name='base_controller_node',
            output='screen'
        ),

        # 2. LiDAR LD14
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
        
        # 4. NODE EKF (Chạy cấu hình Case 1)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_case1]
        ),

        # 5. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config_file, {'use_sim_time': False}],
            output='screen'
        )
    ])
