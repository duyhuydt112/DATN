#ifndef BNO055_HPP
#define BNO055_HPP

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/* Define Pin for BNO055 */
#define BNO055_SCL_PIN 22
#define BNO055_SDA_PIN 21

extern Adafruit_BNO055 BNO055;
std::array<float, 3> Get_BNO055_BasePoint();std::array<float, 3> Read_Angle_ThreeAxes_FromQuat(float &Pan, float &Tilt, float &Roll, std::array<float, 3> Base_Point);
void BNO055_Setup();
float degtorad(float degree);
float normalizeRoll(float roll_deg);
float normalizePan(float pan_deg);
void LimitedAngle(float &Pan, float &Tilt, float &Roll, float Threshold, float limitedPan, float limitedTilt, float limitedRoll);
void PrintCalibrationStatus();
void SetBaseForwardVector();
//int GetPitchDirectionFromQuat();
#endif