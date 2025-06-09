#include <SimpleFOC.h>

// Khởi tạo động cơ với 11 cặp cực
BLDCMotor motor = BLDCMotor(11);

// Khởi tạo driver 3PWM
BLDCDriver3PWM driver = BLDCDriver3PWM(14, 27, 26, 25);

// Khởi tạo cảm biến từ AS5048A (SPI)
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5);

// Biến lưu góc mục tiêu
float target_angle = 0;

// commander interface
Commander command = Commander(Serial);

// Hàm xử lý lệnh từ SimpleFOCStudio
void onTarget(char* cmd) {
  float deg = atof(cmd); // Lấy số từ chuỗi
  motor.target = deg * _PI / 180.0;  // Chuyển sang radian
}

unsigned long lastMove = 0;

void setup() {
  Serial.begin(115200);

  // Khởi tạo SPI cho ESP32
  SPI.begin(18, 23, 19);  // SCK, MISO, MOSI
  sensor.init();
  motor.linkSensor(&sensor);

  // Thiết lập điện áp nguồn cấp
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000; // Giảm tiếng ồn và tiết kiệm điện
  driver.init();
  motor.linkDriver(&driver);

  // Chọn chế độ điều khiển góc
  motor.controller = MotionControlType::angle;

  // PID điều chỉnh để tránh rung giật và quá nhiệt
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 4.0;
  motor.P_angle.P = 2.0;

  // Giới hạn điện áp và tốc độ
  motor.voltage_limit = 4.0;
  motor.velocity_limit = 3.0;
  
  // Bộ lọc tốc độ để giảm nhiễu
  motor.LPF_velocity.Tf = 0.05;

  // Thiết lập giám sát biến và lệnh
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL;
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 10;

  command.add('M', onTarget, "target angle in degrees");
  Serial.println(F("Motor commands sketch | Initial motion control > angle."));

  // Khởi tạo FOC
  motor.init();
  Serial.println("Running FOC init...");
  motor.initFOC();
  Serial.println("Motor ready.");
}

void loop() {
  motor.loopFOC();

  // Di chuyển đến góc mục tiêu định kỳ

  motor.move(motor.target);

  // Gửi dữ liệu theo định dạng của SimpleFOCStudio
  motor.monitor();

  // Nhận lệnh từ giao diện Studio
  command.run();

  delay(10);  // Giảm delay để cập nhật nhanh hơn
}
