#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= CẤU HÌNH CÔNG TẮC =================
// Đổi thành false nếu muốn thử nghiệm SLAM không có IMU
bool USE_IMU = true; 

// ================= CẤU HÌNH CHÂN (Theo sơ đồ tay) =================
// 1. Encoder
const int ENC_R_B = 18; 
const int ENC_R_A = 19; 
const int ENC_L_B = 32; 
const int ENC_L_A = 33; 

//const int ENC_L_A = 18; 
//const int ENC_L_B = 19; 
//const int ENC_R_A = 32; 
//const int ENC_R_B = 33; 

// 2. Motor Driver TB6612
const int PWMA = 13;
const int AIN1 = 12;
const int AIN2 = 14;

const int PWMB = 25;
const int BIN1 = 26;
const int BIN2 = 27;

// ================= BIẾN TOÀN CỤC =================
Adafruit_MPU6050 mpu;
float gyro_z_offset = 0.0; // Biến lưu sai số góc yaw tĩnh
float accel_x_offset = 0.0; // Biến lưu sai số gia tốc x tĩnh
volatile long pulse_left = 0;
volatile long pulse_right = 0;

unsigned long last_time = 0;
const int PUBLISH_HZ = 20; // 20Hz = 50ms

// ================= HÀM NGẮT ENCODER =================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) pulse_left++;
  else pulse_left--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) pulse_right--;
  else pulse_right++;
}

// ================= HÀM ĐIỀU KHIỂN MOTOR =================
// Hàm này nhận giá trị từ -255 đến 255
void setMotors(int pwm_L, int pwm_R) {
  // --- Điều khiển bánh PHẢI (Motor A) ---
  if (pwm_R > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm_R);
  } else if (pwm_R < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm_R); // analogWrite chỉ nhận số dương
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }

  // --- Điều khiển bánh TRÁI (Motor B) ---
  if (pwm_L > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, pwm_L);
  } else if (pwm_L < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -pwm_L);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // DÒNG NÀY ĐỂ CHỐNG LAG ESP32
  // 1. Setup Chân Encoder
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  // Kích hoạt Ngắt
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);

  // 2. Setup Chân Motor
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  // Đảm bảo xe đứng yên lúc mới khởi động
  setMotors(0, 0);

  // 3. Setup IMU
  if (USE_IMU) {
    Wire.begin();
    if (!mpu.begin()) {
      Serial.println("E: Khong tim thay MPU6050!");
      // Vẫn cho code chạy tiếp dù lỗi IMU để test Motor
    } else {
      // Cấu hình IMU cho robot di chuyển chậm (Lọc nhiễu tốt)
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      // --- BẮT ĐẦU HIỆU CHỈNH (CALIBRATION) ---
      // Lấy trung bình 500 mẫu trong 2.5 giây đầu tiên
      long sum = 0;
      for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyro_z_offset += g.gyro.z;
        accel_x_offset += a.acceleration.x;
        delay(5);
      }
      gyro_z_offset = gyro_z_offset / 500.0;
      accel_x_offset = accel_x_offset / 500.0;
    }
  }
}

// ================= LOOP =================
void loop() {
  unsigned long current_time = millis();
  
  // --- 1. NHẬN LỆNH ĐIỀU KHIỂN TỪ PC ---
  // Cú pháp PC gửi xuống: "M,150,150\n" (Tiến tới) hoặc "M,-100,100\n" (Xoay tại chỗ)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("M,")) {
      int firstComma = command.indexOf(',');
      int secondComma = command.indexOf(',', firstComma + 1);
      
      if (firstComma != -1 && secondComma != -1) {
        int pwmL = command.substring(firstComma + 1, secondComma).toInt();
        int pwmR = command.substring(secondComma + 1).toInt();
        
        // Khống chế dải PWM an toàn (-255 đến 255)
        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);
        
        // ĐẢO CHIỀU BÁNH TRÁI ---
        pwmL = -pwmL;
        
        setMotors(pwmL, pwmR);
      }
    }
  }

  // --- 2. GỬI DỮ LIỆU SENSOR LÊN PC ---
  if (current_time - last_time >= (1000 / PUBLISH_HZ)) {
    last_time = current_time;

    float gyro_z = 0.0;
    float accel_x = 0.0;
    
    // Đọc MPU6050
    if (USE_IMU) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      // Trừ đi sai số tĩnh đã đo được lúc khởi động
      gyro_z = g.gyro.z - gyro_z_offset;
      // DEADZONE: Nếu vận tốc quay quá nhỏ (< 0.02 rad/s), coi như đứng yên (0.0)
      if (abs(gyro_z) < 0.02) {
        gyro_z = 0.0;
      }
      // Lấy gia tốc trục X (Tịnh tiến thẳng)
      // (Có thể trừ đi offset tĩnh nếu lúc đứng yên nó không bằng 0)
      accel_x = a.acceleration.x - accel_x_offset;
    }

    // Vô hiệu hóa ngắt tạm thời để copy số xung (tránh lỗi đồng bộ hóa)
    noInterrupts();
    long current_left = pulse_left;
    long current_right = pulse_right;
    interrupts();

    // FORMAT GỬI DỮ LIỆU CHUẨN LÊN ROS 2:
    // Cú pháp: "D,XungTrai,XungPhai,GyroZ,AccelX"
    // (GyroZ là vận tốc góc quay quanh trục Z - Yaw)
    Serial.print("D,");
    Serial.print(current_left);
    Serial.print(",");
    Serial.print(current_right);
    Serial.print(",");
    Serial.print(gyro_z, 4); // Gửi vận tốc góc Z với 4 số thập phân
    Serial.print(",");
    Serial.println(accel_x, 4); // Gửi gia tốc trục X với 4 số thập phân
  }
}
