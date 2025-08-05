/*********************************************************************
 * Angle-openloop giữ vị trí – chống rung nhạy nhanh (đã tối ưu)
 *********************************************************************/
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

constexpr uint8_t PIN_UH_1 = 13, PIN_VH_1 = 12, PIN_WH_1 = 14, PIN_EN_1 = 27;
constexpr uint8_t PIN_UH_2 = 26, PIN_VH_2 = 25, PIN_WH_2 = 33, PIN_EN_2 = 32;
constexpr uint8_t PIN_UH_3 = 15, PIN_VH_3 = 2, PIN_WH_3 = 0, PIN_EN_3 = 4;
constexpr uint8_t POLE_PAIRS = 11;

BLDCMotor       motor1 (POLE_PAIRS);
BLDCDriver3PWM  driver1(PIN_UH_1, PIN_VH_1, PIN_WH_1, PIN_EN_1);
BLDCMotor       motor2 (POLE_PAIRS);
BLDCDriver3PWM  driver2(PIN_UH_2, PIN_VH_2, PIN_WH_2, PIN_EN_2);
BLDCMotor       motor3 (POLE_PAIRS);
BLDCDriver3PWM  driver3(PIN_UH_3, PIN_VH_3, PIN_WH_3, PIN_EN_3);

constexpr float SUPPLY_V      = 12.0;
constexpr float LOOP_HZ       = 500.0;
constexpr float LOOP_DT       = 1.0 / LOOP_HZ;

float offsetPitch   = 0.0f;
float offsetYaw  = 0.0f;


enum Mode : uint8_t {PREPOSITION, CAPTURE_ZERO, BALANCE};
Mode mode = PREPOSITION;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ===== Bộ lọc trung bình trượt cho output PID =====
#define MA_SIZE 5
float pid_buffer[MA_SIZE] = {0};
int pid_idx = 0;

float filterPID(float input) {
  pid_buffer[pid_idx] = input;
  pid_idx = (pid_idx + 1) % MA_SIZE;

  float sum = 0;
  for (int i = 0; i < MA_SIZE; i++) sum += pid_buffer[i];
  return sum / MA_SIZE;
}

// ===== Hàm hỗ trợ =====
float deg2Rad(float d){ return d * DEG_TO_RAD; }
float angleDiff(float target, float current) {
  float diff = fmod(target - current + PI, _2PI) - PI;
  return diff < -PI ? diff + _2PI : diff;
}
float angleDiffDeg(float target_deg, float current_deg) {
  float diff = fmod(target_deg - current_deg + 180.0f, 360.0f) - 180.0f;
  return diff < -180.0f ? diff + 360.0f : diff;
}
struct PIDState {
  float integral = 0.0f;
  float prevErr = 0.0f;
  float shaft_angle = 0.0f;
  unsigned long prev_time = 0;
};

struct PIDParams {
  float Kp;
  float Ki;
  float Kd;
};

PIDState pidPitch;
PIDState pidYaw;

PIDParams pidParamPitch = { 40.0f, 0.3f, 0.2f };
PIDParams pidParamYaw   = { 46.0f, 0.2f, 0.1f };  
// ===== PID tích hợp trong computeElectricalAngle =====
float computeElectricalAngle(float target_angle, float velocity_limit, int pole_pairs,
                             PIDState& state, const PIDParams& params) {
  unsigned long now = micros();
  float Ts = (now - state.prev_time) * 1e-6f;
  state.prev_time = now;
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  float error = angleDiff(target_angle, state.shaft_angle);
  state.integral += error * Ts;
  state.integral = constrain(state.integral, -10.0f, 10.0f);

  float derivative = (error - state.prevErr) / Ts;
  state.prevErr = error;

  float velocity = params.Kp * error + params.Ki * state.integral + params.Kd * derivative;
  velocity = filterPID(velocity);  // nếu bạn có lọc
  velocity = constrain(velocity, -velocity_limit, velocity_limit);

  state.shaft_angle += velocity * Ts;

  float shaft_angle_norm = fmod(state.shaft_angle, _2PI);
  if (shaft_angle_norm < 0) shaft_angle_norm += _2PI;

  float electrical_angle = shaft_angle_norm * pole_pairs;
  return electrical_angle;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("Không tìm thấy BNO055!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  //MOTOR 3
  driver1.voltage_power_supply = SUPPLY_V;
  driver1.pwm_frequency = 20000;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller        = MotionControlType::angle_openloop;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.init();
  motor1.voltage_limit = 4.0f;        // Hạn chế điện áp đầu ra
  motor1.velocity_limit = 5.0f;       // Giới hạn vận tốc góc (rad/s)
  
  //MOTOR 3
  driver2.voltage_power_supply = SUPPLY_V;
  driver2.pwm_frequency = 20000;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller        = MotionControlType::angle_openloop;
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.init();
  motor2.voltage_limit = 7.0f;        // Hạn chế điện áp đầu ra
  motor2.velocity_limit = 4.0f;       // Giới hạn vận tốc góc (rad/s)
  
  //MOTOR 3
  driver3.voltage_power_supply = SUPPLY_V;
  driver3.pwm_frequency = 20000;
  driver3.init();
  motor3.linkDriver(&driver3);
  motor3.controller        = MotionControlType::angle_openloop;
  motor3.torque_controller = TorqueControlType::voltage;
  motor3.init();
  motor3.voltage_limit = 4.0f;        // Hạn chế điện áp đầu ra
  motor3.velocity_limit = 4.0f;       // Giới hạn vận tốc góc (rad/s)
  Mode mode = PREPOSITION;
}

void loop() {
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yawDeg    = e.x();
  float pitchDeg  = e.y();
  float rollDeg   = e.x();
  switch (mode) {
    case PREPOSITION: {
      // // float elec_angle_2 = computeElectricalAngle(0, motor2.velocity_limit, POLE_PAIRS);
      // // float elec_angle_3 = computeElectricalAngle(_PI_2, motor3.velocity_limit, POLE_PAIRS);
      // // motor2.setPhaseVoltage(motor2.voltage_limit, 0, elec_angle_2); 
      // // motor3.setPhaseVoltage(motor3.voltage_limit, 0, elec_angle_3);  
      // motor2.move(PI);
      // motor3.move(PI);
      // motor2.loopFOC();
      // motor3.loopFOC();
      // delay(100);
      // mode = CAPTURE_ZERO;
      static unsigned long start = millis();
      motor2.loopFOC();
      motor3.loopFOC();
      //motor2.move(_PI_2);
      motor3.move(_PI_2);

      if (millis() - start > 1000) {   // sau 1 giây thì chuyển sang CAPTURE_ZERO
        mode = CAPTURE_ZERO;
      }
      break;
    }

    case CAPTURE_ZERO: {
      offsetYaw = yawDeg;
      offsetPitch = pitchDeg;
      pidPitch.shaft_angle = 0;
      pidYaw.shaft_angle = 0;
      mode = BALANCE;
      Serial.printf("Offset = %.2f° | Start BALANCE\n", offsetYaw);
      break;
    }

    case BALANCE: {
      float rel_Yaw = yawDeg - offsetYaw;
      float rel_Pitch = pitchDeg - offsetPitch;
      if (rel_Yaw > 180.0f) rel_Yaw -= 360.0f;
      if (rel_Yaw < -180.0f) rel_Yaw += 360.0f;
      // if (rel_Pitch > 180.0f) rel_Pitch -= 360.0f;
      // if (rel_Pitch < -180.0f) rel_Pitch += 360.0f;
      float target_Yaw = -deg2Rad(rel_Yaw);
      float target_Pitch = -deg2Rad(rel_Pitch);
      //float target_rad = -rel;
      static unsigned long stableStart = 0;
      static bool isStable = false;

      if (fabs(rel_Yaw) < 5.0f) {
        if (!isStable && (millis() - stableStart > 500)) {
          isStable = true;
          Serial.println("🟢 Gimbal stabilized.");
        }
      } else {
        stableStart = millis();
        isStable = false;
      }

      if (isStable) {
        motor3.setPhaseVoltage(0, 0, 0);
        return;
      }

      // === Gọi hàm điều khiển PID + update góc điện ===
      float elec_anglePitch = computeElectricalAngle(target_Pitch, 7.0f, POLE_PAIRS, pidPitch, pidParamPitch);
      float elec_angleYaw   = computeElectricalAngle(target_Yaw,   7.0f, POLE_PAIRS, pidYaw,   pidParamYaw);
      //motor2.setPhaseVoltage(motor2.voltage_limit, 0, elec_anglePitch);
      //motor3.setPhaseVoltage(motor3.voltage_limit, 0, elec_angleYaw);
      

      break;
    }
  }
  Serial.printf("%.2f,%.2f\n",yawDeg, pitchDeg);
  delay(1);
}
