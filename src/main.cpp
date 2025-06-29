#include <SimpleFOC.h>
#include <SPI.h>
#include <Motor_Control.h>

// ------------------- Cấu hình phần cứng -------------------
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(TILT_IN1_PIN, TILT_IN2_PIN, TILT_IN3_PIN, TILT_EN_PIN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, TILT_CS_PIN);

// INA240A2 → Gain = 50V/V, Shunt = 0.01Ω → đo 2 pha
InlineCurrentSense current_sense = InlineCurrentSense(TILT_R_SHUNT, TILT_INA_GAIN, TILT_CURRENT_SENSOR_A0, TILT_CURRENT_SENSOR_A1);

// ------------------- Biến điều khiển vị trí bằng torque -------------------
float target_angle = 0.0;      // rad
float torque_cmd = 0.0;

// PID vị trí ngoài (angle → torque)
float angle_P = 6.0;
float angle_I = 0.6;
float angle_D = 0.1;
float angle_error = 0, prev_error = 0, integral_error = 0;
float derivative = 0;  // giữ giá trị lọc đạo hàm giữa các vòng lặp
const float Dt = 0.002;  // thời gian mẫu (2ms)

// ------------------- Commander -------------------
Commander command = Commander(Serial);
void onAngle(char* buf){ 
  target_angle = atof(buf) * _PI / 180.0;  // nhập độ → rad
}

void setup(){
  Serial.begin(115200);
  SPI.begin(18, 23, 19);
  sensor.init(); motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 4.5;  // Giảm để ổn định torque
  driver.pwm_frequency = 20000;
  driver.init(); motor.linkDriver(&driver);

  if (!current_sense.init()) Serial.println("⚠️ INA240 init FAILED!");
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  current_sense.skip_align = true;
  current_sense.gain_b *= -1;

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // PID dòng Q (torque)
  motor.PID_current_q.P = 7.0;
  motor.PID_current_q.I = 7.0;
  motor.PID_current_q.D = 0.0;
  motor.LPF_current_q.Tf = 0.1;  // lọc tốt hơn

  // PID dòng D
  motor.PID_current_d.P = 5.0;
  motor.PID_current_d.I = 1.0;
  motor.PID_current_d.D = 0.0;
  motor.LPF_current_d.Tf = 0.06;

  // FOC modulation mượt hơn
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = 3.0;

  // Theo dõi thông số
  motor.monitor_variables = _MON_TARGET | _MON_CURR_Q | _MON_ANGLE | _MON_VEL;
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  command.add('A', onAngle, "target angle in degrees");
  Serial.println("✅ Điều khiển vị trí bằng torque sẵn sàng!");
  Serial.println("➡ Gõ A 90 để quay đến góc 90 độ");
}

void loop(){
  motor.loopFOC();

  // ==== PID góc → torque ====
  float deadzone = 0.015;  // rad ≈ 0.86 độ

  // 1. Tính sai số góc [-PI, PI]
  float angle_error = target_angle - motor.shaft_angle;
  angle_error = fmod(angle_error + _PI, _2PI);
  if (angle_error < 0) angle_error += _2PI;
  angle_error -= _PI;

  // 2. Deadzone - ngắt điều khiển nếu rất gần
  if (abs(angle_error) < deadzone) {
    torque_cmd = 0;
    motor.move(torque_cmd);
    command.run();
    motor.monitor();
    delay(1);
    return;
  }

  // 3. Tích phân (nếu không trong deadzone)
  integral_error += angle_error * Dt;
  integral_error = constrain(integral_error, -0.5, 0.5);  // chống windup

  // 4. Đạo hàm có lọc mạnh hơn
  float raw_derivative = (angle_error - prev_error) / Dt;
  derivative = 0.95 * derivative + 0.05 * raw_derivative;
  prev_error = angle_error;

  // 5. PID coefficients - tự động giảm khi gần đích
  angle_P = (abs(angle_error) > 0.05) ? 6.0 : 0.1;
  angle_I = (abs(angle_error) > 0.05) ? 0.6 : 0.03;
  angle_D = (abs(angle_error) > 0.05) ? 0.1 : 0.005;

  // 6. PID output
  torque_cmd = angle_P * angle_error + angle_I * integral_error + angle_D * derivative;
  torque_cmd = constrain(torque_cmd, -motor.voltage_limit, motor.voltage_limit);

  // Debug in ra
  Serial.print("Angle_Err: ");
  Serial.println(angle_error, 4);

  motor.move(torque_cmd);
  command.run();
  motor.monitor();
  delay(1);
}
