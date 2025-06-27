#include <SimpleFOC.h>

// ------------------- Khai báo phần cứng -------------------
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 22, 21, 25);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 17); // CS2

// ------------------- Biến PID tùy chỉnh -------------------
float target_angle = 0;          // Góc mục tiêu (rad)
float error = 0;
float error_integral = 0;
float error_derivative = 0;
float torque_cmd = 0;
float angle_offset = 0.0;
float P = 1.5;
float I = 0.1;
float D = 0.02;

unsigned long t1 = 0;
const int Dt = 10;         // thời gian cập nhật PID (ms)

// ------------------- Giao tiếp Serial -------------------
Commander command = Commander(Serial);
void onTarget(char* cmd) {
  target_angle = atof(cmd) * _PI / 180.0; // nhập bằng độ, chuyển sang rad
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Khởi tạo SPI cho ESP32: SCK, MISO, MOSI
  SPI.begin(18, 23, 19);
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver cấu hình
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;         // giới hạn torque
  driver.pwm_frequency = 20000;
  driver.init();
  motor.linkDriver(&driver);

  // Dùng chế độ điều khiển torque
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  // Giới hạn tốc độ không cần thiết, nhưng vẫn đặt
  motor.velocity_limit = 100;

  // Bộ lọc tốc độ nếu cần dùng D hoặc velocity PID
  motor.LPF_velocity.Tf = 0.05;

  // Monitoring
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL;
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 10;

  command.add('T', onTarget, "target angle in degrees");

  // Khởi tạo FOC
  motor.init();
  motor.initFOC();
  angle_offset = sensor.getAngle();
  target_angle = 0;
  Serial.println("Motor ready.");
  Serial.println("Send command T{angle_deg} e.g. T90");

  t1 = millis();
}

void loop() {
  // PID position loop thủ công
  if ((millis() - t1) >= Dt) {
    float current_angle = motor.shaft_angle - angle_offset;
    error = target_angle - current_angle;

    // Deadzone nếu sai số nhỏ
    if (abs(error) < 0.01) error = 0;

    // PID terms
    error_integral += error * (Dt / 1000.0);
    error_integral = constrain(error_integral, -1.0, 1.0); // anti-windup

    // Khâu D dùng tốc độ đã lọc
    error_derivative = -motor.shaftVelocity();  // ✅ Đúng
    error_derivative = constrain(error_derivative, -1.0, 1.0); // anti

    // PID output = Torque
    torque_cmd = P * error + I * error_integral + D * error_derivative;
    torque_cmd = constrain(torque_cmd, -motor.voltage_limit, motor.voltage_limit);

    t1 = millis();

    // In debug
    Serial.print("Err: "); Serial.print(error, 4);
    Serial.print(" | Torque: "); Serial.print(torque_cmd, 4);
    Serial.print(" | Angle: "); Serial.println(current_angle, 4);
  }

  motor.loopFOC();            // Cập nhật từ cảm biến
  motor.move(torque_cmd);     // Gửi lệnh torque
  motor.monitor();
  command.run();              // Đọc lệnh từ Serial
  delay(10);
}
