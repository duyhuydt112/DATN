#ifndef BNO055_H
#define BNO055_H
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define SCL_PIN 22
#define SDA_PIN 21
extern Adafruit_BNO055 BNO055;
extern imu::Quaternion q_base;
extern bool baseSet;

struct KalmanFilter {
  float est = 0;
  float P = 1.0;
  float Q = 0.01;
  float R = 1.0;
};

extern KalmanFilter kalPan, kalTilt, kalRoll;

void BNO055_Setup();
imu::Quaternion quatInverse(imu::Quaternion q);
imu::Quaternion quatMultiply(imu::Quaternion q1, imu::Quaternion q2);
std::array<float, 3> Read_PanTiltRoll();
float kalmanUpdate(KalmanFilter &kf, float measurement);
#endif