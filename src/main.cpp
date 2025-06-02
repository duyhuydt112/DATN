#include <Joystick.h>
#include <Gimbal_High_Planner.h>
#include <Motor_Control.h>

MotorConfig Roll_Config = {
    0.0,   // initial_target
    0.2,   // PID_P
    20,    // PID_I
    0,     // PID_D
    0.01,  // LPF_Tf
    20,    // P_angle_P
    50,    // velocity_limit
    10,    // voltage_limit
    2,      // current_limit
    10000  // output_ramp
};

void setup() {
  Serial.begin(115200);
  Joystick_Init();
  Motor_Setup(Roll_Config, Roll_Config, Roll_Config);
}

void loop() {
  Joystick_Run();
  FOC_Run();
  Motor_Monitor_Run();
  Roll_Move(PI);
}
