/*********************************************************************
 * Angle‑openloop giữ vị trí – KHÔNG giật, KHÔNG nóng + PID có lọc D
 *********************************************************************/
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

constexpr uint8_t PIN_UH_3 = 15, PIN_VH_3 = 2, PIN_WH_3 = 0, PIN_EN_3 = 4;

constexpr uint8_t POLE_PAIRS = 11;
BLDCMotor       motor3 (POLE_PAIRS);
BLDCDriver3PWM  driver3(PIN_UH_3, PIN_VH_3, PIN_WH_3, PIN_EN_3);

constexpr float SUPPLY_V      = 12.0;
constexpr float VOLT_HOLD_3   = 3.0;
constexpr float VOLT_BOOST_3  = 5.0;

constexpr float LOOP_HZ       = 1000.0;
constexpr float LOOP_DT       = 1.0 / LOOP_HZ;
constexpr float RAMP_STEP     = 0.5;
constexpr float DEAD_BAND     = 0.1f;
constexpr float BOOST_TH      = 1.0;

float integral      = 0.0f;
float prevErr       = 0.0f;
float prevD         = 0.0f;
float offsetPitch   = 0.0f;
constexpr float ELECT_PER_DEG = POLE_PAIRS * DEG_TO_RAD;

float target_req = 0, target_deg = 0;

enum Mode : uint8_t {PREPOSITION, CAPTURE_ZERO, BALANCE};
Mode mode = PREPOSITION;

float deg2Rad(float d){ return d * DEG_TO_RAD; }

// PID
float Kp = 1.0f;
float Ki = 0.00f;
float Kd = 0.08f;

float integral_limit = 50.0f; // giới hạn tích phân

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("Không tìm thấy BNO055!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  driver3.voltage_power_supply = SUPPLY_V;
  driver3.pwm_frequency = 20000;
  driver3.init();

  motor3.linkDriver(&driver3);
  motor3.controller        = MotionControlType::angle_openloop;
  motor3.torque_controller = TorqueControlType::voltage;
  motor3.init();
  motor3.velocity_limit = 5.0;

  target_req = 100.0;
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // ⚠️ Dùng orientation.y nếu bạn muốn điều khiển theo Pitch
  float pitchDeg = orientationData.orientation.z;

  switch (mode) {
    case PREPOSITION: {
      float err  = target_req - target_deg;
      float step = (fabs(err) > RAMP_STEP) ? RAMP_STEP * ((err > 0) ? 1 : -1) : err;
      target_deg += step;

      float vlim_3 = (fabs(err) > BOOST_TH) ? VOLT_BOOST_3 : VOLT_HOLD_3;
      motor3.voltage_limit  = vlim_3;
      driver3.voltage_limit = vlim_3;

      if (fabs(err) < DEAD_BAND) {
        motor3.setPhaseVoltage(0, 0, 0);
        Serial.println("DEAD BAND -----------------------------------------");
      } else {
        motor3.move(deg2Rad(target_deg));
        Serial.println("OUT OF DEAD BAND -----------------------------------------");
      }

      if (target_deg >= 100.0f) {
        mode = CAPTURE_ZERO;
        Serial.println("Reached 100°, capturing offset");
      }
      break;
    }

    case CAPTURE_ZERO: {
      offsetPitch = pitchDeg;
      integral = prevErr = prevD = 0.0f;
      mode = BALANCE;
      Serial.printf("Offset = %.2f° | Start BALANCE\n", offsetPitch);
      break;
    }

    case BALANCE: {
      float deadband_thresh = 0.1f; // vùng chết ±0.5 độ
      float rel = pitchDeg - offsetPitch;
      float err = -rel;

      // Deadband: nếu sai số nhỏ hơn ngưỡng, bỏ qua
      if (fabs(err) < deadband_thresh) {
          err = 0.0f;
      }

      // PID - tích phân (anti-windup)
      integral += err * LOOP_DT;
      integral = constrain(integral, -integral_limit, integral_limit);

      // PID - đạo hàm có lọc nhiễu
      float dRaw = (err - prevErr) / LOOP_DT;
      float dFiltered = 0.9f * prevD + 0.05f * dRaw;
      prevD = dFiltered;
      prevErr = err;

      // Tính PID
      float pid = Kp * err + Ki * integral + Kd * dFiltered;
      pid = constrain(pid, -5.0f, 5.0f);

      // Điều khiển điện tử
      float elecAngle = deg2Rad(target_deg) + (pid * ELECT_PER_DEG);
      elecAngle = constrain(elecAngle, 0, 2*PI);  // hoặc -PI..PI nếu cần
      motor3.move(elecAngle);

      // Debug in ra serial
      Serial.printf("Pitch: %.2f | Err: %.2f | P: %.2f | I: %.2f | D: %.2f | PID: %.2f | Elec: %.2f\n",
        pitchDeg, err, Kp*err, Ki*integral, Kd*dFiltered, pid, elecAngle);
      break;
    }
  }

  motor3.loopFOC();
  delayMicroseconds(2000); // tăng tần số vòng lặp lên 500 Hz
}
