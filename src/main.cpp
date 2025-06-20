#include <SimpleFOC.h>

// ------------------- Khai báo phần cứng -------------------
// SỬA: GM3506 có 7 cặp cực
BLDCMotor motor = BLDCMotor(7);  // ĐÚNG cặp cực của GM3506
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 13, 16, 17);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 15); // CS2

// ------------------- Biến PID tùy chỉnh -------------------
float target_angle = 0;          
float error = 0;
float error_prev = 0;
float error_integral = 0;
float error_derivative = 0;
float torque_cmd = 0;

// SỬA: PID phù hợp hơn
float P = 0.3;
float I = 0.1;
float D = 0.01;

unsigned long t1 = 0;
const int Dt = 10;        

bool has_target = false;  // SỬA: chỉ chạy PID khi có lệnh

// ------------------- Giao tiếp Serial -------------------
Commander command = Commander(Serial);
void onTarget(char* cmd) {
  target_angle = atof(cmd) * _PI / 180.0;
  has_target = true;  // đánh dấu đã có lệnh
}

void setup() {
  Serial.begin(115200);
  delay(500);

  SPI.begin(18, 23, 19);      // SPI cho ESP32
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver cấu hình
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;         
  driver.pwm_frequency = 20000;
  driver.init();
  motor.linkDriver(&driver);

  // Điều khiển torque bằng điện áp
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.velocity_limit = 100;

  // Bộ lọc tốc độ
  motor.LPF_velocity.Tf = 0.15;

  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL;
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 10;

  command.add('T', onTarget, "target angle in degrees");

  motor.init();
  motor.initFOC();

  // SỬA: tránh phản ứng ngay từ đầu
  float init_angle = sensor.getAngle();
  target_angle = init_angle;
  error_prev = 0;
  error_integral = 0;

  Serial.print("Initial angle: ");
  Serial.println(init_angle, 4);

  Serial.println("Motor ready.");
  Serial.println("Send command T{angle_deg} e.g. T90");

  t1 = millis();
}

void loop() {
  if ((millis() - t1) >= Dt && has_target) {
    float current_angle = sensor.getAngle();     
    error = target_angle - current_angle;

    if (abs(error) < 0.01) error = 0;

    error_integral += error * (Dt / 1000.0);
    error_integral = constrain(error_integral, -1.0, 1.0); 
    error_derivative = (error - error_prev) / (Dt / 1000.0);

    torque_cmd = P * error + I * error_integral + D * error_derivative;
    torque_cmd = constrain(torque_cmd, -motor.voltage_limit, motor.voltage_limit);

    error_prev = error;
    t1 = millis();

    Serial.print("Err: "); Serial.print(error, 4);
    Serial.print(" | Torque: "); Serial.print(torque_cmd, 4);
    Serial.print(" | Angle: "); Serial.println(current_angle, 4);
  }

  motor.loopFOC();            
  motor.move(torque_cmd);     
  motor.monitor();
  command.run();              
  delay(10);
}
