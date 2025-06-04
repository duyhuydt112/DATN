#include <SimpleFOC.h>

// Định nghĩa chân kết nối driver
#define PAN_IN1_PIN 14
#define PAN_IN2_PIN 27
#define PAN_IN3_PIN 26
#define PAN_EN_PIN 25

// Khởi tạo motor GM4108 (11 cặp cực từ)
BLDCMotor motor = BLDCMotor(11);

// Khởi tạo driver 3 PWM có chân EN
BLDCDriver3PWM driver = BLDCDriver3PWM(PAN_IN1_PIN, PAN_IN2_PIN, PAN_IN3_PIN, PAN_EN_PIN);

void setup() {
  Serial.begin(115200);
  delay(500);  // Đợi Serial sẵn sàng

  // Cấu hình driver
  driver.voltage_power_supply = 12;  // Điện áp cấp cho motor
  driver.init();

  // Liên kết driver với motor
  motor.linkDriver(&driver);

  // Chế độ điều khiển vòng hở theo vận tốc
  motor.controller = MotionControlType::velocity_openloop;

  // Giới hạn điện áp (tránh đẩy motor lên quá mạnh)
  motor.voltage_limit = 6;  // Bạn có thể thử từ 6~8V

  // Khởi tạo motor
  motor.init();

  // Hiển thị trạng thái
  Serial.println("Motor ready.");
}

void loop() {
  // Di chuyển motor với vận tốc không cảm biến (rad/s)
  motor.move(4.0);  // Tăng lên 3.0~5.0 nếu thấy yếu

  // Có thể dùng delay ngắn hoặc yield() để tránh WDT reset trên ESP32
  delay(5);  // hoặc yield();
}
