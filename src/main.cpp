#include <SimpleFOC.h>
#include <Motor_Control.h>

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------CONFIGURE-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

MotorConfig Pan_Config = {
    0.0f,   // initial_target
    5.0f,   // PID_P
    20.0f,    // PID_I
    0.0f,     // PID_D
    100.0f,    // velocity_limit
    _MON_TARGET | _MON_ANGLE | _MON_VEL,
    10,    // downsample
    10.0f
};

MotorConfig Tilt_Config = {
    0.0f,   // initial_target
    5.0f,   // PID_P
    20.0f,    // PID_I
    0.0f,     // PID_D
    100.0f,    // velocity_limit
    _MON_TARGET | _MON_ANGLE | _MON_VEL,
    10,    // downsample
    10.0f
};

MotorConfig Roll_Config = {
    0.0f,   // initial_target
    5.0f,   // PID_P
    20.0f,    // PID_I
    0.0f,     // PID_D
    100.0f,    // velocity_limit
    _MON_TARGET | _MON_ANGLE | _MON_VEL,
    10,    // downsample
    10.0f
};

DriverConfig Driver_Config;

PID_Calculate Pan_PID;
PID_Calculate Tilt_PID;
PID_Calculate Roll_PID;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------SETUP-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(500);
  SPI.begin(CLK, MISO, MOSI);
  Motor_Enable = {true, false, false}; //PAN, PITCH, ROLL
  Controller_Setup(Pan_Config, Tilt_Config, Roll_Config);
}

void loop() {
  PID_Run(Pan_Config, Pan_Encoder, Pan_PID, Has_Pan_Angle_Target, Pan_Target_Angle);
  PID_Run(Tilt_Config, Tilt_Encoder, Tilt_PID, Has_Tilt_Angle_Target, Tilt_Target_Angle);
  PID_Run(Roll_Config, Roll_Encoder, Roll_PID, Has_Roll_Angle_Target, Roll_Target_Angle);
  FOC_Run();           
  Motor_Move(Pan_PID.Torque_cmd, Tilt_PID.Torque_cmd, Roll_PID.Torque_cmd);
  Motor_Monitor_Run();
  Commander_Run();              
  delay(10);
}
