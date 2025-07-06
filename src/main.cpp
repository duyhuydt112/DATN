/*********************************************************************
 * Angle‑openloop giữ vị trí – KHÔNG giật, KHÔNG nóng
 *********************************************************************/
#include <SimpleFOC.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BNO055.h>
#include <esp_now.h>

constexpr uint8_t PIN_UH_1 = 13, PIN_VH_1 = 12, PIN_WH_1 = 14, PIN_EN_1 = 27;
constexpr uint8_t PIN_UH_2 = 26, PIN_VH_2 = 22, PIN_WH_2 = 21, PIN_EN_2 = 25;
constexpr uint8_t PIN_UH_3 = 15, PIN_VH_3 = 2, PIN_WH_3 = 0, PIN_EN_3 = 4;

constexpr uint8_t POLE_PAIRS = 11;
BLDCMotor       motor2 (POLE_PAIRS);
BLDCDriver3PWM  driver2(PIN_UH_2, PIN_VH_2, PIN_WH_2, PIN_EN_2);
BLDCMotor       motor3 (POLE_PAIRS);
BLDCDriver3PWM  driver3(PIN_UH_3, PIN_VH_3, PIN_WH_3, PIN_EN_3);
// Nguồn & giới hạn
constexpr float SUPPLY_V      = 12.0;
constexpr float VOLT_HOLD_2     = 7.0;   // điện áp giữ
constexpr float VOLT_BOOST_2    = 7.0;   // điện áp di chuyển
constexpr float VOLT_HOLD_3     = 3.0;   // điện áp giữ
constexpr float VOLT_BOOST_3    = 5.0;   // điện áp di chuyển
// Ramp & dead‑band
constexpr float LOOP_HZ   = 1000.0;
constexpr float RAMP_STEP = 0.5;   // °/chu kỳ
constexpr float DEAD_BAND = 0.1;   // ±°
constexpr float BOOST_TH  = 15.0;  // |err| > 15° → boost
float integral = 0.0f;
float prevErr  = 0.0f;
float offsetPitch = 0.0f;

float target_req = 0, target_deg = 0;
enum Mode : uint8_t {PREPOSITION, CAPTURE_ZERO, BALANCE};
Mode mode = PREPOSITION;

inline float deg2Rad(float d){ return d*DEG_TO_RAD; }
float baselinePitchDeg = 0.0f;

struct __attribute__((packed)) QuatMsg {
  float w, x, y, z;
};
volatile QuatMsg latestQuat = {1, 0, 0, 0};
volatile bool quatValid = false;

void onDataRecv(const uint8_t*, const uint8_t* data, int len){
  if (len == sizeof(QuatMsg)) {
    memcpy((void*)&latestQuat, data, sizeof(QuatMsg));
    quatValid = true;
  }
}

float quatToPitchDeg(const QuatMsg& q){
  float sinp = 2.0f * (q.w * q.y- q.z * q.x);
  if (fabsf(sinp) >= 1.0f) return copysignf(90.0f, sinp);
  return asinf(sinp) * 180.0f / PI;
}

void setupEspNow(){
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK){
    Serial.println("[ERR] ESP-NOW init Failed");
    while (true) delay(10);
  }
  esp_now_register_recv_cb(onDataRecv);
}
void setup() {
  Serial.begin(115200);
  setupEspNow();
 //MOTOR 2
  driver2.voltage_power_supply = SUPPLY_V;
  driver2.pwm_frequency = 20000;
  driver2.init();

  motor2.linkDriver(&driver2);
  motor2.controller        = MotionControlType::angle_openloop;
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.init();
  target_req = 100;

   //MOTOR 3
  driver3.voltage_power_supply = SUPPLY_V;
  driver3.pwm_frequency = 25000;
  driver3.init();

  motor3.linkDriver(&driver3);
  motor3.controller        = MotionControlType::angle_openloop;
  motor3.torque_controller = TorqueControlType::voltage;
  motor3.init();
}

void loop() {
  QuatMsg q; memcpy(&q, (void*)&latestQuat, sizeof(QuatMsg));
  float pitchDeg = quatToPitchDeg(q);
  /* 4) Luôn giữ cùng một góc – KHÔNG đặt 0 rad */
  switch (mode) {
    case PREPOSITION: {
        /* 2) Ramping */
        float err  = target_req - target_deg;
        float step = (fabs(err) > RAMP_STEP) ? RAMP_STEP*((err>0)?1:-1) : err;
        target_deg += step;

        /* 3) Chọn điện áp theo sai số */
        float vlim_2 = (fabs(err) > BOOST_TH) ? VOLT_BOOST_2 : VOLT_HOLD_2;
        //MOTOR2
        motor2.voltage_limit  = vlim_2;
        driver2.voltage_limit = vlim_2;
        motor2.move(deg2Rad(target_deg));
        //MOTOR3
        float vlim_3 = (fabs(err) > BOOST_TH) ? VOLT_BOOST_3 : VOLT_HOLD_3;
        motor3.voltage_limit  = vlim_3;
        driver3.voltage_limit = vlim_3;
        // sau khi tính err
        if (fabs(err) < DEAD_BAND) {
          motor3.setPhaseVoltage(0, 0, 0);    // hoàn toàn tắt PWM
        } else {
          motor3.move(deg2Rad(target_deg));
        }
        if (target_deg >= 100.0f) {
          delay(1000);
          mode = CAPTURE_ZERO;
          Serial.println("Reached 90°, capturing offset");
        }
        break;
    }
    case CAPTURE_ZERO: {
      offsetPitch = pitchDeg;
      integral = prevErr = 0.0f;
      mode = BALANCE;
      Serial.print("Offset = "); Serial.print(offsetPitch, 2); Serial.println("°\nBalancing...");
      break;
    }
  }
  motor2.loopFOC();
  motor3.loopFOC();

  delayMicroseconds(10000);          // 1 kHz
}
