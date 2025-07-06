/*********************************************************************
 * Angle‑openloop giữ vị trí – KHÔNG giật, KHÔNG nóng
 *********************************************************************/
#include <SimpleFOC.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BNO055.h>
#include <esp_now.h>
constexpr uint8_t PIN_UH = 26, PIN_VH = 22, PIN_WH = 21, PIN_EN = 25;
constexpr uint8_t POLE_PAIRS = 11;
BLDCMotor       motor (POLE_PAIRS);
BLDCDriver3PWM  driver(PIN_UH, PIN_VH, PIN_WH, PIN_EN);

// Nguồn & giới hạn
constexpr float SUPPLY_V      = 12.0;
constexpr float VOLT_HOLD     = 7.0;   // điện áp giữ
constexpr float VOLT_BOOST    = 7.0;   // điện áp di chuyển

// Ramp & dead‑band
constexpr float LOOP_HZ   = 1000.0;
constexpr float RAMP_STEP = 0.5;   // °/chu kỳ
constexpr float DEAD_BAND = 0.3;   // ±°
constexpr float BOOST_TH  = 15.0;  // |err| > 15° → boost

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

float quatToPitchDeg (const QuatMsg& q){
  float sinp = 2.0f * (q.w * q.y- q.z * q.x);
  if (fabsf(sinp) >= 1.0f) return copysignf(90.0f, sinp);
  return asinf(sinp)*180.0f/PI;
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

  driver.voltage_power_supply = SUPPLY_V;
  driver.pwm_frequency = 25000;
  driver.init();

  motor.linkDriver(&driver);
  motor.controller        = MotionControlType::angle_openloop;
  motor.torque_controller = TorqueControlType::voltage;
  motor.init();

  Serial.println(F("Ready nhập góc (°)"));
}

void loop() {
  /* 1) Đọc lệnh */
  if (Serial.available()) {
    target_req = Serial.parseFloat();
    Serial.printf("Target %.1f°\n", target_req);
  }
  /* 4) Luôn giữ cùng một góc – KHÔNG đặt 0 rad */
  switch (mode) {
    case PREPOSITION: {
        /* 2) Ramping */
        float err  = target_req - target_deg;
        float step = (fabs(err) > RAMP_STEP) ? RAMP_STEP*((err>0)?1:-1) : err;
        target_deg += step;

        /* 3) Chọn điện áp theo sai số */
        float vlim = (fabs(err) > BOOST_TH) ? VOLT_BOOST : VOLT_HOLD;
        motor.voltage_limit  = vlim;
        driver.voltage_limit = vlim;
        motor.move(deg2Rad(target_deg));
        if (target_deg >= 90.0f) {
          delay(1000);
          mode = CAPTURE_ZERO;
          Serial.println("Reached 90°, capturing offset");
        }
        break;
    }

  }
  motor.loopFOC();
  delayMicroseconds(10000);          // 1 kHz
}
