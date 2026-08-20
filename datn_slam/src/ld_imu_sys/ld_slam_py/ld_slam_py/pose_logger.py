import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
import math
import time

def euler_from_quaternion(x, y, z, w):
    # Chuyển đổi Quaternion sang Euler (Yaw)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Mở file CSV để ghi dữ liệu
        self.csv_file = open('robot_trajectory.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'EKF_X', 'EKF_Y', 'EKF_Yaw', 'SLAM_X', 'SLAM_Y', 'SLAM_Yaw'])
        
        self.start_time = time.time()
        # Lấy mẫu mỗi 0.1s (Delta t = 0.1)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info('Đang ghi tọa độ ra file robot_trajectory.csv...')

    def on_timer(self):
        current_time = time.time() - self.start_time
        
        ekf_x, ekf_y, ekf_yaw = 0.0, 0.0, 0.0
        slam_x, slam_y, slam_yaw = 0.0, 0.0, 0.0
        
        # 1. Bắt tọa độ của EKF (odom -> base_link)
        try:
            t_ekf = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            ekf_x = t_ekf.transform.translation.x
            ekf_y = t_ekf.transform.translation.y
            q = t_ekf.transform.rotation
            ekf_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        except TransformException as ex:
            pass # Bỏ qua nếu chưa có TF

        # 2. Bắt tọa độ của SLAM (map -> base_link)
        try:
            t_slam = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            slam_x = t_slam.transform.translation.x
            slam_y = t_slam.transform.translation.y
            q = t_slam.transform.rotation
            slam_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        except TransformException as ex:
            pass
            
        # Ghi vào CSV
        self.csv_writer.writerow([current_time, ekf_x, ekf_y, ekf_yaw, slam_x, slam_y, slam_yaw])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
