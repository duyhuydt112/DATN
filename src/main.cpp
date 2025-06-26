#include <SimpleFOC.h>
#include <Motor_Control.h>

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------CONFIGURE-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

MotorConfig Pan_Config = {
    0.0f,   // initial_target
    0.0f,   // PID_P
    0.0f,    // PID_I
    0.0f,     // PID_D
    _MON_TARGET | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE ,
    10,    // downsample
    10.0f, // voltage_limit
    4.0f   // current_limit
};

MotorConfig Tilt_Config = {
    0.0f,   // initial_target
    0.0f,   // PID_P
    0.0f,    // PID_I
    0.0f,     // PID_D
    _MON_TARGET | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE ,
    10,    // downsample
    10.0f, // voltage_limit
    4.0f   // current_limit
};

MotorConfig Roll_Config = {
    0.0f,   // initial_target
    0.0f,   // PID_P
    0.0f,    // PID_I
    0.0f,     // PID_D
    _MON_TARGET | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE ,
    10,    // downsample
    10.0f, // voltage_limit
    4.0f   // current_limit
};

DriverConfig Driver_Config;

CurrentPID Pan_Current_PID = {
    3.0f,  // q_P
    1.0f,  // q_I
    0.0f,  // q_D
    10.0f, // q_Output_Ramp
    0.02f, // q_tf

    2.0f,  // d_P
    1.0f,  // d_I
    0.0f,  // d_D
    10.0f, // d_Output_Ramp
    0.02f  // d_tf
};


CurrentPID Tilt_Current_PID = {
    5.0f,   // q_P
    5.0f,   // q_I
    0.0f,   // q_D
    10.0f,  // q_Output_Ramp
    0.01f,  // q_tf

    5.0f,   // d_P
    5.0f,   // d_I
    0.0f,   // d_D
    10.0f,  // d_Output_Ramp
    0.01f,  // d_tf
};


CurrentPID Roll_Current_PID = {
    5.0f,   // q_P
    5.0f,   // q_I
    0.0f,   // q_D
    10.0f,  // q_Output_Ramp
    0.01f,  // q_tf

    5.0f,   // d_P
    5.0f,   // d_I
    0.0f,   // d_D
    10.0f,  // d_Output_Ramp
    0.01f,  // d_tf
};


/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*                                                                    SETUP                                                                       */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

bool Roll_Initialized = false;
bool Tilt_Initialized = false;
bool Pan_Initialized  = false;

// ------------------------- TÁCH RIÊNG SETUP TỪNG MOTOR (TORQUE/VOLTAGE CONTROL) -------------------------


void setup() {
  Serial.begin(115200);
  delay(500);
  SPI.begin(CLK, MISO, MOSI);
  Motor_Enable = {true, false, false};
  Controller_Setup(Pan_Config, Tilt_Config, Roll_Config, Driver_Config ,Pan_Current_PID, Tilt_Current_PID, Roll_Current_PID);
}

void loop() {
    FOC_Run();
    Motor_Move(Pan_Target_Current, Tilt_Target_Current, Roll_Target_Current);
    Motor_Monitor_Run();
    Commander_Run();
    delay(1);
}