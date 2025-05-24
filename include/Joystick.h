#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <Arduino.h>
#include <Gimbal_Position_Control.h>
#include <iostream>
#include <cmath>
#include <Motor_Control.h>

/* Axis Rotation */
extern float Pan_Degree;
extern float Tilt_Degree;
extern float Roll_Degree;

/* Define Rotation Range*/
const float MIN_PAN = -180;
const float MAX_PAN = 180;
const float MIN_TILT = -90;
const float MAX_TILT = 90;
const float MIN_ROLL = -180;
const float MAX_ROLL = 180;
const float JOY_CENTER = 2048; 
const int DEADZONE = 200;

/* Define Pin for Joystick*/
const int VRX_PIN = 25;
const int VRY_PIN = 33;
const int SW_PIN = 32;

/* Define Angular Velocity Per_Second */
const float SPEED_DEG_PER_SECOND = 45.0f; // quay 90 độ mỗi giây khi gạt max
const float SPEED_RAD_PER_SECOND = SPEED_DEG_PER_SECOND * PI / 180.0f;

/* Time Variable */
extern unsigned long prevMillis;

/* Azimuth Angle && Radius */
extern float Azimuth_Angle;
extern float Elevation_Angle;
extern float Radius;
extern float Roll_Angle;

/* Define Joystick Press State */
enum Press_State {
    PAN_STATE,
    TILT_STATE,
    ROLL_STATE
};

extern bool Last_Joystick_Pressed;
extern int Current_State;

/* Define Control Function*/
void Joystick_Init();
std::array<float, 2> Joystick_Input_Processing();
void Joystick_Run();
void Button_State();
#endif
