#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

// SPI cho các driver
#define PAN_CS_PIN   5
#define ROLL_CS_PIN  16
#define TILT_CS_PIN  17
#define MOSI         19
#define MISO         23
#define CLK          18

/* Power For Driver */
#define PAN_EN_PIN  25
#define TILT_EN_PIN 26
#define ROLL_EN_PIN 27
#define GND_PIN     0

/* Driver Pan */
#define PAN_IN1_PIN  0
#define PAN_IN2_PIN  2
#define PAN_IN3_PIN  4

/* Driver Tilt */
#define TILT_IN1_PIN 12
#define TILT_IN2_PIN 13
#define TILT_IN3_PIN 14

/* Driver ROLL */
#define ROLL_IN1_PIN 21
#define ROLL_IN2_PIN 22
#define ROLL_IN3_PIN 15

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
extern std::array<bool, 3> Motor_Commander_Enable;

/* Motor Motion Parameter Config */
struct MotorConfig {
  float initial_target;
  float PID_P;
  float PID_I;
  float PID_D;
  float LPF_Tf;
  float P_angle_P;
  float velocity_limit;
  float voltage_limit;
  float current_limit;
};
extern MotorConfig motorConfig;

/* Define Function*/
void Motor_Setup(const MotorConfig& Pan_Conf, const MotorConfig& Tilt_Conf, const MotorConfig& Roll_Conf);
void Motors_Move(float Pan_Move_Angle_Rad, float Tilt_Move_Angle_Rad, float Roll_Move_Angle_Rad);
void Do_Motor(std::array<bool, 3> Motor_Commander_Enable, char* cmd);
void Commander_Run();
void Motor_Monitor_Run();
void configureMotor(BLDCMotor& motor, const MotorConfig& config);
void Pan_Move(float Pan_Move_Angle_Rad);
void Tilt_Move(float Tilt_Move_Angle_Rad);
void Roll_Move(float Roll_Move_Angle_Rad);
void FOC_Run();
#endif 
