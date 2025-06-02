#ifndef BNO055_HPP
#define BNO055_HPP

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/* Define Pin for BNO055 */
#define BNO055_SCL_PIN 22
#define BNO055_SDA_PIN 21

extern Adafruit_BNO055 BNO055;
std::array<float, 3> Get_BNO055_BasePoint();
std::array<float, 3> Read_Angle_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point);
void BNO055_Setup();

#endif