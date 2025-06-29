#include <SimpleFOC.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Motor_Control.h>

// ------------------- Dữ liệu từ ESP1 -------------------
struct IMUData {
  float yaw;
  float pitch;
  float roll;
};

IMUData imuData;
float pitch = 0.0;
float filtered_pitch = 0.0;
const float alpha = 0.15;  // low-pass filter

// ------------------- PID vị trí → torque -------------------
float target_angle = 0.0;
float torque_cmd = 0.0;
float angle_error = 0, prev_error = 0, integral_error = 0;
float derivative = 0;
const float Dt = 0.002;

// ------------------- PID giữ thăng bằng theo pitch -------------------
float balance_target = 0.0;
float balance_error = 0.0;
float balance_integral = 0.0;
float balance_derivative = 0.0;
float balance_prev = 0.0;

float Kp_balance = 7.0;
float Ki_balance = 0.01;
float Kd_balance = 2.0;

// ------------------- PID vị trí ngoài -------------------
float angle_P = 7.0;
float angle_I = 0.3;
float angle_D = 0.1;

// ------------------- Trạng thái -------------------
bool reached_target = false;
bool enable_balance = false;
float bno_offset = 0.0;

// ------------------- Cấu hình phần cứng -------------------
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(TILT_IN1_PIN, TILT_IN2_PIN, TILT_IN3_PIN, TILT_EN_PIN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, TILT_CS_PIN);
InlineCurrentSense current_sense = InlineCurrentSense(TILT_R_SHUNT, TILT_INA_GAIN, TILT_CURRENT_SENSOR_A0, TILT_CURRENT_SENSOR_A1);

// ------------------- Commander -------------------
Commander command = Commander(Serial);
void onAngle(char* buf) {
  target_angle = atof(buf) * _PI / 180.0;
  reached_target = false;
  enable_balance = false;
  Serial.println("Quay tới góc đích...");
}

// ------------------- Nhận dữ liệu từ ESP1 -------------------
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(IMUData)) {
    memcpy(&imuData, incomingData, sizeof(IMUData));
    pitch = (imuData.pitch - bno_offset) * _PI / 180.0;
    filtered_pitch = alpha * pitch + (1 - alpha) * filtered_pitch;
  }
}

void setup() {
  Serial.begin(115200);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1);
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW sẵn sàng");

  // SPI + Motor
  SPI.begin(18, 23, 19);
  sensor.init(); motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 4.5;
  driver.pwm_frequency = 20000;
  driver.init(); motor.linkDriver(&driver);

  if (!current_sense.init()) Serial.println("INA240 init FAILED!");
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  current_sense.skip_align = true;
  current_sense.gain_b *= -1;

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.PID_current_q.P = 9.0;
  motor.PID_current_q.I = 7.0;
  motor.LPF_current_q.Tf = 0.1;

  motor.PID_current_d.P = 2.0;
  motor.PID_current_d.I = 1.0;
  motor.LPF_current_d.Tf = 0.06;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = 3.0;

  motor.monitor_variables = _MON_TARGET | _MON_CURR_Q | _MON_ANGLE | _MON_VEL;
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 10;

  motor.init();
  motor.initFOC();

  command.add('A', onAngle, "target angle in degrees");
  Serial.println("Gõ A 90 để quay tới 90 độ rồi giữ cân bằng.");
}

void loop() {
  motor.loopFOC();

  float current_angle = motor.shaft_angle;

  // PID góc → torque
  angle_error = target_angle - current_angle;
  angle_error = fmod(angle_error + _PI, _2PI);
  if (angle_error < 0) angle_error += _2PI;
  angle_error -= _PI;

  integral_error += angle_error * Dt;
  integral_error = constrain(integral_error, -0.5, 0.5);

  float raw_derivative = (angle_error - prev_error) / Dt;
  derivative = 0.95 * derivative + 0.05 * raw_derivative;
  prev_error = angle_error;

  torque_cmd = angle_P * angle_error + angle_I * integral_error + angle_D * derivative;
  torque_cmd = constrain(torque_cmd, -motor.voltage_limit, motor.voltage_limit);

  // Nếu đã đến góc đích → bật giữ thăng bằng
  if (!reached_target && abs(angle_error) < 0.02) {
    reached_target = true;
    enable_balance = true;
    bno_offset = imuData.pitch;
    Serial.println("Đã đến đích. Bắt đầu giữ thăng bằng.");
  }

  float balance_torque = 0;
  if (enable_balance) {
    balance_error = balance_target - filtered_pitch;
    balance_integral += balance_error * Dt;
    balance_integral = constrain(balance_integral, -0.5, 0.5);
    balance_derivative = (balance_error - balance_prev) / Dt;
    balance_prev = balance_error;

    balance_torque = Kp_balance * balance_error + Ki_balance * balance_integral + Kd_balance * balance_derivative;
  }

  float total_torque = torque_cmd + balance_torque;
  total_torque = constrain(total_torque, -motor.voltage_limit, motor.voltage_limit);

  // Debug
  Serial.print("Err: "); Serial.print(angle_error, 4);
  Serial.print(" | Pitch: "); Serial.print(filtered_pitch, 4);
  Serial.print(" | Raw pitch: "); Serial.print(imuData.pitch, 2);
  Serial.print(" | Torque: "); Serial.println(total_torque, 4);

  motor.move(total_torque);
  command.run();
  motor.monitor();
  delay(1);
}
