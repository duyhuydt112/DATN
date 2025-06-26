#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <PD_Fuzzy.h>

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------PIN_INIT-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

// SPI cho các driver
#define PAN_CS_PIN   5
#define TILT_CS_PIN  17
#define ROLL_CS_PIN  16
#define MOSI         19
#define MISO         23
#define CLK          18

/* Power For Driver */
#define PAN_EN_PIN  27
#define TILT_EN_PIN 25
#define ROLL_EN_PIN 4
#define GND_PIN     0

/* Driver Pan */
#define PAN_IN1_PIN  13
#define PAN_IN2_PIN  12
#define PAN_IN3_PIN  14

/* Driver Tilt */
#define TILT_IN1_PIN 26
#define TILT_IN2_PIN 22
#define TILT_IN3_PIN 21

/* Driver ROLL */
#define ROLL_IN1_PIN 15
#define ROLL_IN2_PIN 2
#define ROLL_IN3_PIN 0

/* Current Sense */
    /* ROLL */
#define ROLL_R_SHUNT 0.1f
#define ROLL_INA_GAIN 100.0f
#define ROLL_CURRENT_SENSOR_A0 36
#define ROLL_CURRENT_SENSOR_A1 39
    /* TILT */
#define TILT_R_SHUNT 0.02f
#define TILT_INA_GAIN 50.0f
#define TILT_CURRENT_SENSOR_A0 34
#define TILT_CURRENT_SENSOR_A1 35
    /* PAN */
#define PAN_R_SHUNT 0.1f
#define PAN_INA_GAIN 50.0f
#define PAN_CURRENT_SENSOR_A0 33
#define PAN_CURRENT_SENSOR_A1 32

/* Define Parameter For Motor */
    /* GM3506 */
#define GM3506_PAIR_POLES 11
#define GM3506_KV_RATING 190

    /* GM4108 */
#define GM4108_PAIR_POLES 11
#define GM4108_KV_RATING 45



/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------VARIABLES-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

/* Encoder Object Init */
extern MagneticSensorSPI Pan_Encoder;
extern MagneticSensorSPI Roll_Encoder;
extern MagneticSensorSPI Tilt_Encoder;

/* Driver Object Init */
extern BLDCDriver3PWM Pan_Driver;
extern BLDCDriver3PWM Tilt_Driver;
extern BLDCDriver3PWM Roll_Driver;

/* BLDC Object Init */
extern BLDCMotor Pan_Motor;
extern BLDCMotor Tilt_Motor;
extern BLDCMotor Roll_Motor;

extern InlineCurrentSense Roll_Current_Sensor;
extern InlineCurrentSense Tilt_Current_Sensor;
extern InlineCurrentSense Pan_Current_Sensor;

/* Commander via Terminal */
extern Commander commander;
extern std::array<bool, 3> Motor_Enable;

/* Motor Motion Parameter Config */
struct MotorConfig {
  float Initial_target;
  float PID_P;
  float PID_I;
  float PID_D;
  int Monitor_Variables;
  int Monitor_Downsample;
  float Voltage_limit;
  float Current_limit;
  //float Velocity_limit;
  // float Velocity_LPF_Tf;
  // float P_angle_P;
  // float Output_Ramp;
};

struct PID_Calculate {
  const int Delta_Time = 10;
  unsigned int Last_Time = 0;
  float Target_angle = 0;          
  float Error = 0;
  float Error_Prev = 0;
  float Error_Integral = 0;
  float Error_Derivative = 0;
  float Torque_cmd = 0;
};

/* Driver Parameter Config */
struct DriverConfig {
  float Voltage_Power_Supply = 12;
  float Voltage_Limit = 6;
  float Pwm_Frequency = 20000;
};

/* Current Sense PID Config */
struct CurrentPID {
  float P_Current_q;
  float I_Current_q;
  float D_Current_q;
  float Output_Ramp_Current_q;
  float Tf_LPF_Current_q;

  float P_Current_d;
  float I_Current_d;
  float D_Current_d;
  float Output_Ramp_Current_d;
  float Tf_LPF_Current_d;
};


/* Global Variables */
extern bool Has_Pan_Angle_Target;
extern bool Has_Tilt_Angle_Target;
extern bool Has_Roll_Angle_Target;
extern float Pan_Target_Angle;
extern float Tilt_Target_Angle;
extern float Roll_Target_Angle;
extern float Pan_Target_Current;
extern float Tilt_Target_Current;
extern float Roll_Target_Current;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------Configure-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void Configure_Motor(BLDCMotor& motor, const MotorConfig& motorConfig, const DriverConfig& driverConfig);
void Configure_Driver(const DriverConfig& config, BLDCDriver3PWM& driver);
void Linking_With_Motor(BLDCDriver3PWM& Driver_Conf, MagneticSensorSPI& Encoder_Conf, BLDCMotor& Motor, InlineCurrentSense& Current_Sense);
void Controller_Setup(const MotorConfig& Pan_Conf, const MotorConfig& Tilt_Conf, const MotorConfig& Roll_Conf, const DriverConfig& Driver, const CurrentPID& Pan_Curr_Conf, const CurrentPID& Tilt_Curr_Conf, const CurrentPID& Roll_Curr_Conf);

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------Serial Commander-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void on_Pan_Angle_Target(char* cmd);
void on_Tilt_Angle_Target(char* cmd);
void on_Roll_Target(char* cmd);
void on_Pan_Current_Target(char* cmd);
void on_Tilt_Current_Target(char* cmd);
void on_Roll_Current_Target(char* cmd);

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------RUN FUNCTION-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void PID_Run(const MotorConfig& motorConfig, MagneticSensorSPI& sensor, PID_Calculate& PID, bool& Has_Angle_Target, float& Target_Angle, BLDCMotor& Motor);
void Motors_Move(float Pan_Move_Angle_Rad, float Tilt_Move_Angle_Rad, float Roll_Move_Angle_Rad);
void Motor_Monitor_Run();
void Commander_Run();
void Motor_Move(float Pan_Move_Angle_Degree, float Tilt_Move_Angle_Degree, float Roll_Move_Angle_Degree);
void FOC_Run();
#endif 
