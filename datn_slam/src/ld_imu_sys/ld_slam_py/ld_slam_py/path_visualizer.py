import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Khởi tạo bộ lắng nghe TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Khởi tạo 2 Publisher phát quỹ đạo lên RViz
        self.ekf_path_pub = self.create_publisher(Path, '/ekf_path', 10)
        self.slam_path_pub = self.create_publisher(Path, '/slam_path', 10)
        
        # Khởi tạo đối tượng Path
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom' # Quỹ đạo EKF bám theo gốc odom
        
        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map'   # Quỹ đạo SLAM bám theo gốc map
        
        # Timer cập nhật quỹ đạo mỗi 0.2 giây (5Hz)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("Đang phát quỹ đạo thời gian thực lên RViz2...")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Bắt tọa độ của EKF (odom -> base_link)
        try:
            t_ekf = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'odom'
            pose.pose.position.x = t_ekf.transform.translation.x
            pose.pose.position.y = t_ekf.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t_ekf.transform.rotation
            
            self.ekf_path.poses.append(pose)
            self.ekf_path.header.stamp = now
            self.ekf_path_pub.publish(self.ekf_path)
        except TransformException:
            pass
            
        # 2. Bắt tọa độ của SLAM (map -> base_link)
        try:
            t_slam = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = t_slam.transform.translation.x
            pose.pose.position.y = t_slam.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t_slam.transform.rotation
            
            self.slam_path.poses.append(pose)
            self.slam_path.header.stamp = now
            self.slam_path_pub.publish(self.slam_path)
        except TransformException:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
