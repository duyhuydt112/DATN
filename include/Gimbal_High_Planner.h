#ifndef GIMBAL_HIGH_PLANNER_HPP
#define GIMBAL_HIGH_PLANNER_HPP

#include <vector> 
#include <iostream>
#include <array>
#include <vector>
#include <math.h>
#include <Arduino.h>
#include <BNO055.h>

/* Number of Sample*/
extern double T_Planner;
extern double Ts_Planner;

/* Output */
extern const int MAX_SAMPLES; 
extern double Planner_OutPut[3][100];
extern int Number_Samples ;

// float MPU_GetAngle();
// std::vector<std::array<float, 3>> Spline_Interpolation(const std::array<float, 3>& Start_Point, std::array<float, 3>& End_Point);
void High_Planner(double P[3], double Tilt, double T, double Ts, double q[3][100], int &N_out);
double evaluatePolynomial(const double coeff[6], double t);
void solveLinearSystem(double A[6][6], double b[6], double x[6]);
// void Move2_Target_Point(float X, float Y, float Z, float Tilt_Angle);


#endif