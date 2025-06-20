#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

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
#define PAN_EN_PIN  26
#define TILT_EN_PIN 13
#define ROLL_EN_PIN 4
#define GND_PIN     00000000000000

/* Driver Pan */
#define PAN_IN1_PIN  12
#define PAN_IN2_PIN  14
#define PAN_IN3_PIN  27

/* Driver Tilt */
#define TILT_IN1_PIN 25
#define TILT_IN2_PIN 33
#define TILT_IN3_PIN 32

/* Driver ROLL */
#define ROLL_IN1_PIN 15
#define ROLL_IN2_PIN 2
#define ROLL_IN3_PIN 0

/* Define Parameter For Motor */
    /* GM3506 */
#define GM3506_PAIR_POLES 11
#define GM3506_KV_RATING 190

    /* GM4108 */
#define GM4108_PAIR_POLES 11
#define GM4108_KV_RATING 45

// /* Module Joystick */
// #define JOYSTICK_X_PIN    36  // ADC input only
// #define JOYSTICK_Y_PIN    39  // ADC input only
// #define JOYSTICK_BTN_PIN  33  // Digital input

// /* MPU9250 - I2C */
// #define MPU9250_SDA_PIN   32
// #define MPU9250_SCL_PIN   34  // Input only, vẫn dùng được I2C

// /* Nút bấm riêng */
// #define BUTTON1_PIN       35  // Input only digital
// #define BUTTON2_PIN       3   // RX0 - dùng nếu không dùng UART0

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

/* Commander via Terminal */
extern Commander commander;
extern std::array<bool, 3> Motor_Enable;

/* Motor Motion Parameter Config */
struct MotorConfig {
  float Initial_target;
  float PID_P;
  float PID_I;
  float PID_D;
  float Velocity_limit;
  int Monitor_Variables;
  int Monitor_Downsample;
  float Voltage_limit;
  // float Current_limit;
  // float LPF_Tf = 0.15;
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


/* Global Variables */
extern bool Has_Pan_Angle_Target = false;
extern bool Has_Tilt_Angle_Target = false;
extern bool Has_Roll_Angle_Target = false;
extern float Pan_Target_Angle = 0;
extern float Tilt_Target_Angle = 0;
extern float Roll_Target_Angle = 0;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------Configure-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void Configure_Motor(BLDCMotor& motor, const MotorConfig& config);
void Configure_Driver(const DriverConfig& config, BLDCDriver3PWM& driver);
void Linking_With_Motor(BLDCDriver3PWM& Driver_Conf, MagneticSensorSPI& Encoder_Conf, BLDCMotor& Motor);
void Controller_Setup(const MotorConfig& Pan_Conf, const MotorConfig& Tilt_Conf, const MotorConfig& Roll_Conf);

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------Serial Commander-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void on_Pan_Angle_Target(char* cmd);
void on_Tilt_Angle_Target(char* cmd);
void on_Roll_Target(char* cmd);

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------RUN FUNCTION-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

void PID_Run(const MotorConfig& motorConfig, MagneticSensorSPI& sensor, PID_Calculate& PID, bool& Has_Angle_Target, float& Target_Angle);
void Motors_Move(float Pan_Move_Angle_Rad, float Tilt_Move_Angle_Rad, float Roll_Move_Angle_Rad);
void Motor_Monitor_Run();
void Commander_Run();
void Motor_Move(float Pan_Move_Angle_Degree, float Tilt_Move_Angle_Degree, float Roll_Move_Angle_Degree);
void FOC_Run();
#endif 
