#include <SimpleFOC.h>
#include <Motor_Control.h>

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------CONFIGURE-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

MotorConfig Pan_Config = {
    0.0f,   // initial_target
    6.0f,   // PID_P
    0.2f,    // PID_I
    0.01f,     // PID_D
    10.0f,    // velocity_limit
    _MON_TARGET | _MON_ANGLE | _MON_VEL,
    10,    // downsample
    10.0f,
    0.05f
};

MotorConfig Tilt_Config = {
    0.0f,   // initial_target
    5.0f,   // PID_P
    0.2f,    // PID_I
    0.01f,     // PID_D
    10.0f,    // velocity_limit
    _MON_TARGET | _MON_ANGLE | _MON_VEL,
    10,    // downsample
    8.0f,
    0.15f
};

MotorConfig Roll_Config = {
    0.0f,   // initial_target
    2.0f,   // PID_P
    0.2f,    // PID_I
    0.01f,     // PID_D
    10.0f,    // velocity_limit
    _MON_TARGET | _MON_ANGLE | _MON_VEL,
    10,    // downsample
    10.0f,
    0.15f
};

DriverConfig Driver_Config;

PID_Calculate Pan_PID;
PID_Calculate Tilt_PID;
PID_Calculate Roll_PID;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*                                                                    SETUP                                                                       */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(500);
  SPI.begin(CLK, MISO, MOSI);
  Motor_Enable = {true, false, false}; //PAN, PITCH, ROLL
  Controller_Setup(Pan_Config, Tilt_Config, Roll_Config, Driver_Config);
  Serial.println("Motor ready.");
  Serial.println("Send command T{angle_deg} e.g. T90");
  Pan_Target_Angle = 0.0;
  Tilt_Target_Angle = 0.0;
  Roll_Target_Angle = 0.0;
}

void loop() {
  PID_Run(Pan_Config, Pan_Encoder, Pan_PID, Has_Pan_Angle_Target, Pan_Target_Angle, Pan_Motor);
  PID_Run(Tilt_Config, Tilt_Encoder, Tilt_PID, Has_Tilt_Angle_Target, Tilt_Target_Angle, Tilt_Motor);
  PID_Run(Roll_Config, Roll_Encoder, Roll_PID, Has_Roll_Angle_Target, Roll_Target_Angle, Roll_Motor);
  FOC_Run();           
  Motor_Move(Pan_PID.Torque_cmd, Tilt_PID.Torque_cmd, Roll_PID.Torque_cmd);
  Motor_Monitor_Run();
  Commander_Run();              
  delay(5);
}
