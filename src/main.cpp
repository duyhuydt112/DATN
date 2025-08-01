/*********************************************************************
 * Angle-openloop giữ vị trí – chống rung nhạy nhanh
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
constexpr float VOLT_BOOST_3  = 7.0;

constexpr float LOOP_HZ       = 1000.0;
constexpr float LOOP_DT       = 1.0 / LOOP_HZ;

float integral      = 0.0f;
float prevErr       = 0.0f;
float prevD         = 0.0f;
float offsetPitch   = 0.0f;
constexpr float ELECT_PER_DEG = POLE_PAIRS * DEG_TO_RAD;

float target_req = 100.0;
float target_deg = 0.0;

enum Mode : uint8_t {PREPOSITION, CAPTURE_ZERO, BALANCE};
Mode mode = PREPOSITION;

float deg2Rad(float d){ return d * DEG_TO_RAD; }

// PID
float Kp = 0.2f;
float Ki = 1.0f;
float Kd = 0.5f;
float integral_limit = 50.0f;

// Deadband và Hysteresis
constexpr float DEAD_BAND = 0.5f;
constexpr float HYSTERESIS = 0.2f;

// Giữ torque khi giữ yên đủ lâu
unsigned long stableStart = 0;
bool isStable = false;
constexpr uint16_t STABLE_TIME_MS = 500;

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
  // ======= LẤY GÓC BNO055 =======
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yawDeg = e.x();

  switch (mode) {
    case PREPOSITION: {
      motor3.move(0);
      mode = CAPTURE_ZERO;
      break;
    }

    case CAPTURE_ZERO: {
      offsetPitch = yawDeg;
      integral = prevErr = prevD = 0.0f;
      stableStart = millis();
      mode = BALANCE;
      Serial.printf("Offset = %.2f° | Start BALANCE\n", offsetPitch);
      break;
    }

    case BALANCE: {
      float rel = yawDeg - offsetPitch;
      float err = -rel;

      static bool shouldReturn = false;
      static unsigned long returnTimer = 0;

      if (fabs(rel) > 5.0f) {
        if (!shouldReturn) {
          shouldReturn = true;
          returnTimer = millis();
          Serial.println("⏳ Lệch khỏi góc ban đầu >5°, bắt đầu đếm thời gian...");
        }
      }

      if (shouldReturn) {
        if (fabs(rel) > 5.0f) {
          returnTimer = millis();
        } else if (millis() - returnTimer > 5000) {
          target_deg = offsetPitch;
          offsetPitch = yawDeg;
          shouldReturn = false;
          Serial.println("🔁 Quay về góc đã ghi nhận tại CAPTURE_ZERO");
        }
      }

      float rel2 = yawDeg - offsetPitch;
      if (rel2 > 180.0f) rel2 -= 360.0f;
      if (rel2 < -180.0f) rel2 += 360.0f;
      float err2 = -rel2;

      static bool insideDeadZone = false;
      if (fabs(err2) < (DEAD_BAND - HYSTERESIS)) {
        err2 = 0.0f;
        insideDeadZone = true;
      } else if (fabs(err2) > (DEAD_BAND + HYSTERESIS)) {
        insideDeadZone = false;
      } else if (insideDeadZone) {
        err2 = 0.0f;
      }

      if (fabs(rel2) < 0.3f) {
        if (!isStable && (millis() - stableStart > STABLE_TIME_MS)) {
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

      if (fabs(err2) < 0.1f) integral = 0;
      integral += err2 * LOOP_DT;
      integral = constrain(integral, -integral_limit, integral_limit);
      float dRaw = (err2 - prevErr) / LOOP_DT;
      float dFiltered = 0.5f * prevD + 0.05f * dRaw;
      prevD = dFiltered;
      prevErr = err2;
      float pid = Kp * err2 + Ki * integral + Kd * dFiltered;
      pid = constrain(pid, -4.5f, 4.5f);

      // Bỏ giới hạn slope để đáp ứng nhanh
      if (fabs(err2) < 11.0f) {
        pid = roundf(pid * 10.0f) / 10.0f;
        //pid = roundf(pid);
      }

      motor3.move(pid);

      Serial.printf("%.2f,%.2f,%.2f,%.2f\n", rel, err2, pid, yawDeg);
      break;
    }
  }

  motor3.loopFOC();
  delay(1); // Tăng tốc phản hồi
}
