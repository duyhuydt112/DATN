#ifndef GIMBAL_POSITION_CONTROL_HPP
#define GIMBAL_POSITION_CONTROL_HPP
#include <vector> 
#include <array>
// float MPU_GetAngle();
// std::vector<std::array<float, 3>> Spline_Interpolation(const std::array<float, 3>& Start_Point, std::array<float, 3>& End_Point);
std::array<float, 3> Invert_Kinematic(const std::array<float, 4> Target_Postion);
// void Move2_Target_Point(float X, float Y, float Z, float Tilt_Angle);


#endif