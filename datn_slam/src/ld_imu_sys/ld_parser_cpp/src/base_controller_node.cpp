#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <algorithm>

class BaseControllerNode : public rclcpp::Node {
public:
    BaseControllerNode() : Node("base_controller_node") {
    	// Khai báo Publisher
        // Publisher tách biệt cho EKF
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        
        // THÊM SUBSCRIBER LẮNG NGHE LỆNH TỪ BÀN PHÍM
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&BaseControllerNode::cmd_vel_callback, this, std::placeholders::_1));

	// Mở kết nối với ESP32
        try {
            ser_.setPort("/dev/esp32_robot"); // ESP32 đã được udev rules là esp32_robot
            ser_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(50); //(100 -> 50) Ép C++ đọc nhanh hơn
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Không thể mở cổng Serial ESP32!");
        }

        if(ser_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Đã kết nối ESP32 thành công. Đã giao TF cho EKF.");
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20), // 50Hz để đọc nhanh
                std::bind(&BaseControllerNode::update_odometry, this));
        }
        last_time_ = this->now();
    }

private:
    // HÀM XỬ LÝ LỆNH TELEOP -> CHUYỂN THÀNH PWM CHO ESP32 (Chỉ cập nhật mục tiêu, KHÔNG gửi Serial ngay)
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    	// Nếu hệ thống đang báo trượt/kẹt, KHÔNG CHO PHÉP gửi lệnh chạy xuống ESP32
        if (is_slipping_) {
            RCLCPP_WARN(this->get_logger(), "Đang khóa ga, từ chối lệnh từ bàn phím!");
            return; 
        }
        target_v_ = msg->linear.x;    // Vận tốc thẳng (m/s)
        target_w_ = msg->angular.z;   // Vận tốc góc xoay (rad/s)
    }

    void update_odometry() {
        if (!ser_.available()) return;

        std::string data = ser_.readline();
        // Xóa ký tự \n \r
        data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());
        data.erase(std::remove(data.begin(), data.end(), '\r'), data.end());

        if (data.rfind("D,", 0) == 0) { // Nếu chuỗi bắt đầu bằng "D,"
            std::stringstream ss(data);
            std::string item;
            std::vector<std::string> tokens;
            while (std::getline(ss, item, ',')) {
                tokens.push_back(item);
            }

	    // CÓ 5 TOKENS (Gồm: D,XungTrai,XungPhai,GyroZ,AccelX)
            if (tokens.size() >= 5) {
                try {
                    long current_left_ticks = std::stol(tokens[1]);
                    long current_right_ticks = std::stol(tokens[2]);
                    double gyro_z = std::stod(tokens[3]); // Vận tốc góc từ IMU
                    double accel_x = std::stod(tokens[4]); // Lấy gia tốc tịnh tiến từ IMU

                    if (first_run_) {
                        last_left_ticks_ = current_left_ticks;
                        last_right_ticks_ = current_right_ticks;
                        last_time_ = this->now(); //Tránh lỗi nhảy vọt (Spike) ở lần chạy đầu tiên
                        first_run_ = false;
                        return;
                    }

                    // 1. TÍNH TOÁN KINEMATICS
                    long delta_left = current_left_ticks - last_left_ticks_;
                    long delta_right = current_right_ticks - last_right_ticks_;

                    double distance_left = delta_left * METERS_PER_TICK;
                    double distance_right = delta_right * METERS_PER_TICK;
                    
                    // Quãng đường xe di chuyển ở tâm
                    double distance_center = (distance_right + distance_left) / 2.0;
                
                    // Vận tốc dài (Linear Velocity) và Vận tốc góc (Angular Velocity)
                    rclcpp::Time current_time = this->now();
                    double dt = (current_time - last_time_).seconds();
                    
                    // Chỉ xuất vận tốc cho EKF (EKF sẽ tự lo phần tích phân tọa độ)
                    double v_x = distance_center / dt;
                    
                    // Tính vận tốc góc xoay (Yaw) từ 2 bánh xe
                    double v_yaw_wheel = (distance_right - distance_left) / WHEEL_BASE / dt;

                    // ==========================================
                    // 2. KHỞI ĐỘNG MỀM (SOFT-START ENGINE) - CHỐNG SỤT ÁP ESP32
                    // ==========================================
                    double max_dv = 0.03; // Gia tốc tối đa: 0.03 m/s mỗi 20ms
                    double max_dw = 0.1;  // Xoay tối đa: 0.1 rad/s mỗi 20ms
                    
                    if (current_v_ < target_v_) current_v_ = std::min(current_v_ + max_dv, target_v_);
                    else if (current_v_ > target_v_) current_v_ = std::max(current_v_ - max_dv, target_v_);

                    if (current_w_ < target_w_) current_w_ = std::min(current_w_ + max_dw, target_w_);
                    else if (current_w_ > target_w_) current_w_ = std::max(current_w_ - max_dw, target_w_);

                    // Chỉ gửi lệnh xuống ESP32 nếu không kẹt
                    if (!is_slipping_ && ser_.isOpen()) {
                    	// Tính vận tốc từng bánh (m/s)
                        double v_L = current_v_ - (current_w_ * WHEEL_BASE / 2.0);
                        double v_R = current_v_ + (current_w_ * WHEEL_BASE / 2.0);
                        
                        // Quy đổi ra PWM (-255 đến 255)
        		// Giả sử tốc độ max của động cơ là 0.3 m/s tương đương PWM 255
                        const double MAX_SPEED = 0.3; 
                        
                        int pwm_L = static_cast<int>((v_L / MAX_SPEED) * 255.0);
                        int pwm_R = static_cast<int>((v_R / MAX_SPEED) * 255.0);
                        
                        // Khống chế trong khoảng an toàn
                        if (pwm_L > 255) pwm_L = 255; if (pwm_L < -255) pwm_L = -255;
                        if (pwm_R > 255) pwm_R = 255; if (pwm_R < -255) pwm_R = -255;

			// Gửi lệnh xuống ESP32
                        std::string cmd = "M," + std::to_string(pwm_L) + "," + std::to_string(pwm_R) + "\n";
                        ser_.write(cmd);
                    }
                
                    // ==========================================
                    // 3. THUẬT TOÁN ĐÂM TƯỜNG DUAL-THRESHOLD & BẤT ĐỒNG BỘ GÓC
                    //    DYNAMIC COVARIANCE (PHÁT HIỆN TRƯỢT/KẸT) & PHẢN XẠ TỰ VỆ
                    // ==========================================
                    double covariance_x = 0.001; // Mặc định: Tin tưởng bánh xe tuyệt đối
                    
                    // XỬ LÝ ĐỘNG HỌC BẤT ĐỒNG BỘ
                    double v_yaw_wheel_corrected = v_yaw_wheel;
                    
                    // Phân loại kiểu xoay dựa vào chiều di chuyển của 2 bánh xe:
                    // Tích 2 quãng đường < 0 nghĩa là 1 bánh tiến, 1 bánh lùi -> Xe đang quay tại chỗ
                    if (distance_left * distance_right < 0) {
                        // Áp dụng hệ số x0.5 để triệt tiêu hiện tượng đo phóng đại 180 độ (chênh lệch vận tốc 2 bánh lớn)
                        v_yaw_wheel_corrected = v_yaw_wheel * 0.5; 
                    }
                    // Nếu tích >= 0 (Cùng tiến/lùi hoặc 1 bánh đứng im - Pivot turn), giữ nguyên v_yaw_wheel
                    
                    // Tính độ lệch (Bất đồng bộ) giữa vận tốc góc Encoder ĐÃ HIỆU CHỈNH và IMU
                    double yaw_mismatch = std::abs(v_yaw_wheel_corrected - gyro_z);
                    
                    // === CÁC KỊCH BẢN PHÁT HIỆN LỖI ===
                    // Gia tốc MPU6050 tính bằng m/s^2.
                    // Kịch bản A: Đâm xuyên cực mạnh (Xung lực Đâm chính diện) -> Bỏ qua biến đếm, dừng ngay lập tức
                    if (std::abs(accel_x) > 4.0) {
                        slip_counter_ = 2;
                    }
                    
                    // Kịch bản B: Trượt, kẹt nhẹ xát tường (Quẹt tường) -> Chờ 2 vòng lặp để lọc nhiễu
                    // Phải có gia tốc lớn liên tiếp 2 lần (~40ms) mới coi là Đâm tường
                    // (Điều này loại bỏ hoàn toàn nhiễu giật khi mới khởi động)
                    else if (std::abs(accel_x) > 2.0 && std::abs(v_x) > 0.05) {
                        slip_counter_++; 
                        // RCLCPP_INFO(this->get_logger(), "slip_counter = %d", slip_counter_);
                    }
                    
                    // Kịch bản C: Trượt bùn/mất độ bám êm ái
                    // Ngưỡng 0.5 rad/s ~ 28 độ
                    else if (yaw_mismatch > 0.5) {
                        slip_counter_++; 
                    }
                    
                    // Nếu xe chạy êm, mọi thông số đều đồng bộ -> reset biến đếm
                    else {
                        slip_counter_ = 0;
                    }

		    // --- XỬ LÝ PHANH KHẨN CẤP VÀ EKF ---
                    if (slip_counter_ >= 2) {
                        if (!is_slipping_) {
                            is_slipping_ = true;
                            // Xóa bỏ hoàn toàn lệnh cũ, ép xe dừng hẳn
                            target_v_ = 0.0; target_w_ = 0.0;
                            current_v_ = 0.0; current_w_ = 0.0;
                            
                            // Cảnh báo log chi tiết để phân biệt do Gia tốc hay Lệch góc
                            RCLCPP_WARN(this->get_logger(), 
                            	"PHÁT HIỆN KẸT/TRƯỢT/ĐÂM TƯỜNG (A_x: %.2f, Lệch góc: %.2f)! PHANH KHẨN CẤP!", 
                                accel_x, yaw_mismatch);
                                
                            // GỬI LỆNH DỪNG ĐỘNG CƠ XUỐNG ESP32 NGAY LẬP TỨC
                            if (ser_.isOpen()) {
                                ser_.write("M,0,0\n");
                            }
                        }
                        slip_timer_ = current_time;	
                    }
		    
		    // Duy trì trạng thái choáng 
                    // EKF cần một khoảng thời gian ngắn để ổn định lại vị trí sau khi sự cố xảy ra, thay vì tin bánh xe lại ngay lập tức.
                    if (is_slipping_) {
                        covariance_x = 100.0; // ĐỘ TIN CẬY CỰC THẤP -> EKF sẽ vứt bỏ Odom
                        
                        // Nếu đã qua 2.5 giây mà không có rung lắc nào thêm -> Coi như đã thoát
                        if ((current_time - slip_timer_).seconds() > 2.5) {
                            is_slipping_ = false;
                            RCLCPP_INFO(this->get_logger(), "Đã thoát kẹt, có thể điều khiển lại."); 
                        }
                    }
                  
                    // ==========================================
                    // 4. PUBLISH TF VÀ GÓI TIN (Giữ nguyên Cấu trúc)
                    // ==========================================
                    // 4.1. GỬI TOPIC BÁNH XE VỚI ĐỘ TIN CẬY ĐỘNG
                    nav_msgs::msg::Odometry odom_msg;
                    odom_msg.header.stamp = current_time;
                    odom_msg.header.frame_id = "odom";
                    odom_msg.child_frame_id = "base_link";
                    
                    odom_msg.pose.pose.orientation.x = 0.0;
                    odom_msg.pose.pose.orientation.y = 0.0;
                    odom_msg.pose.pose.orientation.z = 0.0;
                    odom_msg.pose.pose.orientation.w = 1.0;
                     
                    odom_msg.twist.twist.linear.x = v_x; // Vận tốc
                    odom_msg.twist.twist.angular.z = v_yaw_wheel; // Góc quay từ bánh xe
                    
                    // Gán độ tin cậy (Covariance). Càng nhỏ càng tin cậy.
		    // Áp dụng Dynamic Covariance vào đây!
		    // => Khi đâm tường, EKF vứt vận tốc và góc quay của bánh xe
                    odom_msg.twist.covariance[0] = covariance_x;    
                    odom_msg.twist.covariance[35] = covariance_x; 
                     
                    odom_pub_->publish(odom_msg);
		    
		    // ==========================================
		    // 4.2. GỬI TOPIC IMU
                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp = current_time;
                    imu_msg.header.frame_id = "imu_link";
                    
                    imu_msg.orientation.x = 0.0;
                    imu_msg.orientation.y = 0.0;
                    imu_msg.orientation.z = 0.0;
                    imu_msg.orientation.w = 1.0;
                    
                    imu_msg.angular_velocity.z = gyro_z;
                    
                    // Tin tưởng tuyệt đối vào góc xoay của IMU MPU6050
                    imu_msg.angular_velocity_covariance[8] = 0.0001; 
                    
                    imu_pub_->publish(imu_msg);
                    // ==========================================

		    // Cập nhật biến
                    last_left_ticks_ = current_left_ticks;
                    last_right_ticks_ = current_right_ticks;
                    last_time_ = current_time;
                } 
                catch (const std::exception& e) {
                    // Nếu đọc trúng rác, chỉ cảnh báo chứ không cho phép Crash Node
                    RCLCPP_WARN(this->get_logger(), "Nhiễu Serial, bỏ qua 1 frame. Lỗi: %s", e.what());
                    return;
                }    
            } 
        }
     }

    // THÔNG SỐ CƠ KHÍ CỦA ROBOT
    const double WHEEL_RADIUS = 0.034;
    const double WHEEL_BASE = 0.194;
    const double TICKS_PER_REV = 494.5;
    const double METERS_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

    serial::Serial ser_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_; 
    rclcpp::TimerBase::SharedPtr timer_;

    bool first_run_ = true;
    long last_left_ticks_ = 0;
    long last_right_ticks_ = 0;
    rclcpp::Time last_time_;
    
    // Biến phục vụ khởi động mềm và phân tích đâm tường
    double target_v_ = 0.0;
    double target_w_ = 0.0;
    double current_v_ = 0.0;
    double current_w_ = 0.0;
    // Các biến dùng cho Dynamic Covariance
    bool is_slipping_ = false;
    int slip_counter_ = 0; // BIẾN NÀY ĐỂ LỌC NHIỄU KHỞI ĐỘNG
    rclcpp::Time slip_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseControllerNode>());
    rclcpp::shutdown();
    return 0;
}
