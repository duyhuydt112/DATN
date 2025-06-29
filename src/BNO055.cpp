#include "BNO055.h"

Adafruit_BNO055 BNO055 = Adafruit_BNO055(55, 0x28);
KalmanFilter kalPan, kalTilt, kalRoll;

imu::Quaternion q_base;
bool baseSet = false;

void BNO055_Setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Dang ket noi den BNO055...............");
  if (!BNO055.begin(OPERATION_MODE_NDOF)) {
    Serial.println("Khong ket noi duoc BNO055!");
    while (1);
  }
  Serial.println("Da ket noi thanh cong BNO055!!");
  BNO055.setExtCrystalUse(true);
  Serial.println("---------------------------------------");
  delay(1000);
}

imu::Quaternion quatInverse(imu::Quaternion q) {
  return imu::Quaternion(q.w(), -q.x(), -q.y(), -q.z());
}

imu::Quaternion quatMultiply(imu::Quaternion q1, imu::Quaternion q2) {
  float w1 = q1.w(), x1 = q1.x(), y1 = q1.y(), z1 = q1.z();
  float w2 = q2.w(), x2 = q2.x(), y2 = q2.y(), z2 = q2.z();

  return imu::Quaternion(
    w1*w2 - x1*x2 - y1*y2 - z1*z2,
    w1*x2 + x1*w2 + y1*z2 - z1*y2,
    w1*y2 - x1*z2 + y1*w2 + z1*x2,
    w1*z2 + x1*y2 - y1*x2 + z1*w2
  );
}

float kalmanUpdate(KalmanFilter &kf, float measurement) {
  kf.P += kf.Q;
  float K = kf.P / (kf.P + kf.R);
  kf.est += K * (measurement - kf.est);
  kf.P *= (1 - K);
  return kf.est;
}

std::array<float, 3> Read_PanTiltRoll() {
  imu::Quaternion q_now = BNO055.getQuat();
  if (!baseSet) {
    q_base = q_now;
    baseSet = true;
    return {0.0, 0.0, 0.0};
  }

  imu::Quaternion q_rel = quatMultiply(q_now, quatInverse(q_base));
  float w = q_rel.w(), x = q_rel.x(), y = q_rel.y(), z = q_rel.z();

  float pan = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)) * RAD_TO_DEG;
  float tilt = asin(constrain(2 * (w*y - z*x), -1.0, 1.0)) * RAD_TO_DEG;
  float roll = atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)) * RAD_TO_DEG;

  return {
    kalmanUpdate(kalPan, pan),
    kalmanUpdate(kalTilt, tilt),
    kalmanUpdate(kalRoll, roll)
  };
}
