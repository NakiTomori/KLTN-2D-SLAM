#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <serial/serial.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class Ld14Node : public rclcpp::Node {
public:
    Ld14Node() : Node("raw_data_node") {
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        try {
            // CHÚ Ý: Đổi tên cổng này theo tên udev rules của mạch CP2102
            ser_.setPort("/dev/ldlidar"); 
            ser_.setBaudrate(115200); // LD14 luôn chạy ở 115200
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Không thể mở cổng Serial LiDAR!");
        }

        if(ser_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Đã kết nối LiDAR LD14 thành công.");
            // Chạy vòng lặp cực nhanh (1ms) để dọn buffer liên tục
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(&Ld14Node::process_serial, this));
        }

        // Khởi tạo mảng đệm rỗng (0.0)
        ranges_buffer_.assign(360, 0.0f);
        intensities_buffer_.assign(360, 0.0f);
    }

private:
    void process_serial() {
        if (!ser_.available()) return;

        size_t len = ser_.available();
        std::vector<uint8_t> temp_buf(len);
        ser_.read(temp_buf.data(), len);
        
        // Nạp vào buffer tổng
        buffer_.insert(buffer_.end(), temp_buf.begin(), temp_buf.end());

        // Tìm và cắt các gói tin hợp lệ (47 bytes)
        while (buffer_.size() >= 47) {
            // Header của LD14 luôn là 0x54 và byte thứ 2 luôn là 0x2C
            if (buffer_[0] == 0x54 && buffer_[1] == 0x2C) {
                std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 47);
                parse_packet(packet);
                buffer_.erase(buffer_.begin(), buffer_.begin() + 47);
            } else {
                // Nếu sai Header, vứt bỏ 1 byte và dò lại
                buffer_.erase(buffer_.begin());
            }
        }
    }

    void parse_packet(const std::vector<uint8_t>& data) {
        // Đọc góc bắt đầu và kết thúc (Scale 100)
        float start_angle = static_cast<float>(data[5] << 8 | data[4]) / 100.0f;
        float end_angle = static_cast<float>(data[43] << 8 | data[42]) / 100.0f;

        // Nếu góc bị tụt xuống (vòng quay mới bắt đầu), tiến hành publish
        if (last_start_angle_ != -1.0f && start_angle < last_start_angle_) {
            publish_scan();
        }
        last_start_angle_ = start_angle;

        // Tính khoảng cách góc giữa các điểm đo (12 điểm)
        float diff = end_angle - start_angle;
        if (diff < 0) diff += 360.0f;
        float step = diff / 11.0f;

        for (int i = 0; i < 12; i++) {
            float current_angle = start_angle + step * i;
            if (current_angle >= 360.0f) current_angle -= 360.0f;

            // BẮT BUỘC ĐẢO CHIỀU GÓC (Vì LD14 quay cùng chiều kim đồng hồ)
            current_angle = 360.0f - current_angle;
            if (current_angle >= 360.0f) current_angle -= 360.0f;

            // Làm tròn góc để đưa vào mảng 360 phần tử
            int index = std::round(current_angle);
            if (index >= 360) index = 0;

            int data_idx = 6 + i * 3;
            uint16_t dist_mm = static_cast<uint16_t>(data[data_idx + 1] << 8 | data[data_idx]);
            uint8_t intensity = data[data_idx + 2];

            // Lọc các điểm rác gần thân xe (< 15cm)
            if (dist_mm > 150) {
                ranges_buffer_[index] = dist_mm / 1000.0f; // Đổi sang mét
                intensities_buffer_[index] = static_cast<float>(intensity);
            }
        }
    }

    void publish_scan() {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = "laser_frame";

        // CẤU HÌNH HEADER CHUẨN (359 ĐỘ)
        scan_msg.angle_min = 0.0;
        scan_msg.angle_max = (2.0 * M_PI) * (359.0 / 360.0);
        scan_msg.angle_increment = (2.0 * M_PI) / 360.0;
        scan_msg.scan_time = 0.1; // Tương đương 10Hz
        scan_msg.time_increment = scan_msg.scan_time / 360.0;
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 8.0;

        scan_msg.ranges = ranges_buffer_;
        scan_msg.intensities = intensities_buffer_;

        scan_pub_->publish(scan_msg);

        // Đóng gói xong thì reset mảng về 0.0f
	// Reset khoảng cách về vô cực (Chuẩn của ROS2 đối với các tia không có tín hiệu phản hồi thường đặt là giá trị vô cực.
	std::fill(ranges_buffer_.begin(), ranges_buffer_.end(), std::numeric_limits<float>::infinity());
        std::fill(intensities_buffer_.begin(), intensities_buffer_.end(), 0.0f);
    }

    serial::Serial ser_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<uint8_t> buffer_;
    std::vector<float> ranges_buffer_;
    std::vector<float> intensities_buffer_;
    float last_start_angle_ = -1.0f;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ld14Node>());
    rclcpp::shutdown();
    return 0;
}
