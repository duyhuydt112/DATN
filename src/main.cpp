//--------------------------------------------------------------
//  ESP‑NOW 2‑ESP SETUP · MOTOR SIDE (ESP2)
//  -----------------------------------------------------------
//  • ESP1  : BNO055 + quaternion (w,x,y,z) broadcast
//  • ESP2  : receives quaternion → converts to pitch →
//            1) open‑loop ramp motor to 90 °
//            2) capture offset
//            3) PID balance using IMU data (still no encoder)
//
//  Hardware (ESP2)
//    • ESP32 DevKit v1
//    • BLDC gimbal motor 11 pole‑pairs (GBM2804‑x)
//    • DRV8313 / L6234 (UH‑VH‑WH) 3‑PWM driver
//--------------------------------------------------------------

#include <WiFi.h>
#include <esp_now.h>
#include <SimpleFOC.h>

//--------------------------------------------------------------
//  Motor pins & params
//--------------------------------------------------------------
constexpr int PIN_IN1 = 26;
constexpr int PIN_IN2 = 22;
constexpr int PIN_IN3 = 21;
constexpr int PIN_EN = 25;
constexpr uint8_t POLE_PAIRS = 11;

BLDCMotor      motor(POLE_PAIRS);
BLDCDriver3PWM driver(PIN_IN1, PIN_IN2, PIN_IN3, PIN_EN);

//--------------------------------------------------------------
//  ESP‑NOW quaternion packet
//--------------------------------------------------------------
struct __attribute__((packed)) QuatMsg {
  float w, x, y, z;
};
volatile QuatMsg latestQuat = {1,0,0,0};
volatile bool    quatValid  = false;

void onDataRecv(const uint8_t*, const uint8_t* data, int len) {
  if (len == sizeof(QuatMsg)) {
    memcpy((void*)&latestQuat, data, sizeof(QuatMsg));
    quatValid = true;
  }
}

//--------------------------------------------------------------
//  Quaternion → pitch (deg)
//--------------------------------------------------------------
float quatToPitchDeg(const QuatMsg& q) {
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (fabsf(sinp) >= 1.0f) return copysignf(90.0f, sinp);
  return asinf(sinp) * 180.0f / PI;
}

//--------------------------------------------------------------
//  Control constants
//--------------------------------------------------------------
const float INIT_MECH_DEG = 90.0f;                // pre‑rotate
const float ELECT_PER_DEG = POLE_PAIRS * DEG_TO_RAD;
const float TARGET_ELEC   = INIT_MECH_DEG * ELECT_PER_DEG;
const float INIT_TIME_S   = 1.5f;                 // ramp duration

// PID
float Kp = 1.2f;
float Ki = 0.0f;
float Kd = 0.03f;
float integral = 0.0f;
float prevErr  = 0.0f;
constexpr float LOOP_DT = 0.002f; // 500 Hz
uint32_t tLast = 0;

//--------------------------------------------------------------
//  Runtime vars
//--------------------------------------------------------------
enum Mode : uint8_t { PREPOSITION, CAPTURE_ZERO, BALANCE };
Mode        mode = PREPOSITION;
uint32_t    tStart;
float       offsetPitch = 0.0f;
float       elecAngle   = 0.0f;

//--------------------------------------------------------------
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP‑NOW init failed");
    while (true) delay(10);
  }
  esp_now_register_recv_cb(onDataRecv);
  // Broadcast mode ⇒ no need to add peers.
}

//--------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP2 · Gimbal Motor (no encoder) ===\n");

  setupEspNow();

  // driver
  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit        = 10.0f;    // more torque
  driver.pwm_frequency        = 25000;
  driver.init();

  // motor
  motor.linkDriver(&driver);
  motor.foc_modulation  = FOCModulationType::SinePWM;
  motor.voltage_limit   = driver.voltage_limit;
  motor.controller      = MotionControlType::angle_openloop;
  motor.init();

  tStart = micros();
  Serial.println("Waiting quaternion & pre‑rotate 90°...\n");
}

//--------------------------------------------------------------
void loop() {
  if (micros() - tLast < LOOP_DT * 1e6) return;
  tLast = micros();

  if (!quatValid) return;   // no IMU data yet

  QuatMsg q; memcpy(&q, (void*)&latestQuat, sizeof(QuatMsg));
  float pitchDeg = quatToPitchDeg(q);

  switch (mode) {
    //---------------- PRE‑ROTATE -----------------------------
    case PREPOSITION: {
      float prog = min((micros() - tStart) / 1e6f / INIT_TIME_S, 1.0f);
      elecAngle = prog * TARGET_ELEC;               // no wrap
      motor.move(elecAngle);
      if (prog >= 1.0f) {
        mode = CAPTURE_ZERO;
        Serial.println("Reached 90°, capturing offset");
      }
      break;
    }

    //---------------- CAPTURE ZERO --------------------------
    case CAPTURE_ZERO: {
      offsetPitch = pitchDeg;
      integral = prevErr = 0.0f;
      mode = BALANCE;
      Serial.print("Offset = "); Serial.print(offsetPitch, 2); Serial.println("°\nBalancing...");
      break;
    }

    //---------------- BALANCE -------------------------------
    case BALANCE: {
      float rel = pitchDeg - offsetPitch;
      float err = -rel;
      integral += err * LOOP_DT;
      float d   = (err - prevErr) / LOOP_DT;
      prevErr   = err;
      float pid = Kp * err + Ki * integral + Kd * d; // deg
      elecAngle += pid * ELECT_PER_DEG;
      motor.move(elecAngle);

      static uint16_t div=0; if(++div>=25){div=0;
        Serial.print("Rel ");Serial.print(rel,2);
        Serial.print("° | Err ");Serial.print(err,2);
        Serial.print(" | Elec(rad) ");Serial.println(elecAngle,3);}      
      break;
    }
  }
}
//--------------------------------------------------------------
