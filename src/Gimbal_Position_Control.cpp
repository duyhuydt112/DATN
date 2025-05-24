#include <Gimbal_Position_Control.h>
#include <iostream>
#include <array>
#include <vector>
#include <math.h>

// float MPU_GetAngle(){
//     return 10;
// }

std::array<float, 3> Invert_Kinematic(const std::array<float, 4> Target_Postion){
    float q1, q2, q3;
    std::array<float, 3> Target_Angle;
    Target_Angle[0] = -atan2(Target_Postion[0], Target_Postion[1]); //q1
    Target_Angle[1] = -atan2(Target_Postion[2], sqrt(Target_Postion[0]*Target_Postion[0] + Target_Postion[1]*Target_Postion[1]));   //q2
    Target_Angle[2] = Target_Postion[3];     //q3
    // Target_Angle[3] = MPU_GetAngle();
    return Target_Angle;
}

// void Move2_Target_Point(float X, float Y, float Z, float Tilt_Angle){

//     return 0;
// }