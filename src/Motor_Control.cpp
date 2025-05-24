#include "Motor_Control.h"

/* Encoder Object Init */
MagneticSensorSPI Pan_Encoder = MagneticSensorSPI(AS5048_SPI, PAN_CS_PIN);
MagneticSensorSPI Roll_Encoder = MagneticSensorSPI(AS5048_SPI, ROLL_CS_PIN);
MagneticSensorSPI Tilt_Encoder = MagneticSensorSPI(AS5048_SPI, TILT_CS_PIN);

/* Driver Object Init */
BLDCDriver3PWM Pan_Driver = BLDCDriver3PWM(PAN_IN1_PIN, PAN_IN2_PIN, PAN_IN2_PIN, PAN_EN_PIN);
BLDCDriver3PWM Tilt_Driver = BLDCDriver3PWM(TILT_IN1_PIN, TILT_IN2_PIN, TILT_IN3_PIN, TILT_EN_PIN);
BLDCDriver3PWM Roll_Driver = BLDCDriver3PWM(ROLL_IN1_PIN, ROLL_IN2_PIN, ROLL_IN3_PIN, ROLL_EN_PIN);

/* BLDC Object Init */
BLDCMotor Pan_Motor = BLDCMotor(GM4108_PAIR_POLES, GM4108_KV_RATING);
BLDCMotor Tilt_Motor = BLDCMotor(GM3506_PAIR_POLES, GM4108_KV_RATING);
BLDCMotor Roll_Motor = BLDCMotor(GM3506_PAIR_POLES, GM3506_KV_RATING);

/* Commander via Terminal */
Commander commander = Commander(Serial);
std::array<bool, 3> Motor_Commander_Enable = {true, false, false};



void configureMotor(BLDCMotor& motor, const MotorConfig& config) {

    /* Initial Value of Angle*/
    motor.target = config.initial_target; // E.x: initial_angle = 0.0 radian

    /* PID Paramter Config */
    motor.PID_velocity.P = config.PID_P;
    motor.PID_velocity.I = config.PID_I;
    motor.PID_velocity.D = config.PID_D;

    /* Low Pass Filter For Velocity */
    motor.LPF_velocity.Tf = config.LPF_Tf;

    /* Outer loop control (angle) */
    motor.P_angle.P = config.P_angle_P;

    /* Velocity limit when controlling by angle */
    motor.velocity_limit = config.velocity_limit;

    /* Set Limit */
    motor.voltage_limit = config.voltage_limit; // Volts - default driver.voltage_limit
    motor.current_limit = config.current_limit; // Amps - default 0.2Amps
    motor.P_angle.output_ramp = config.output_ramp; // Rad/S^2 - Accelaration 
}

void Do_Motor(char* cmd) { 
    if (Motor_Commander_Enable[0]) {
        commander.motor(&Pan_Motor, cmd);
    }
    if (Motor_Commander_Enable[1]) {
        commander.motor(&Tilt_Motor, cmd);
    }
    if (Motor_Commander_Enable[2]) {
        commander.motor(&Roll_Motor, cmd);
    }
}

void Motor_Setup(const MotorConfig& Pan_Conf, const MotorConfig& Tilt_Conf, const MotorConfig& Roll_Conf){

    /* Driver Attribute Config */
    Pan_Driver.pwm_frequency = 20000;
    Pan_Driver.voltage_power_supply = 12;
    Pan_Driver.voltage_limit = 12;

    Tilt_Driver.pwm_frequency = 20000;
    Tilt_Driver.voltage_power_supply = 12;
    Tilt_Driver.voltage_limit = 12;
    
    Roll_Driver.pwm_frequency = 20000;
    Roll_Driver.voltage_power_supply = 12;
    Roll_Driver.voltage_limit = 12;

    /* Driver Init */
    Roll_Driver.init();
    Tilt_Driver.init();
    Pan_Driver.init();

    /* Encoder Init */
    Pan_Encoder.init();
    Roll_Encoder.init();
    Tilt_Encoder.init();

    /* Link Sensor && Driver With Motor */
    Pan_Motor.linkDriver(&Pan_Driver);
    Pan_Motor.linkSensor(&Pan_Encoder);
    Pan_Motor.controller = MotionControlType::angle;

    Tilt_Motor.linkDriver(&Tilt_Driver);
    Tilt_Motor.linkSensor(&Tilt_Encoder);
    Tilt_Motor.controller = MotionControlType::angle;

    Roll_Motor.linkDriver(&Roll_Driver);
    Roll_Motor.linkSensor(&Roll_Encoder);
    Roll_Motor.controller = MotionControlType::angle;

    /* Motor Init */
    Pan_Motor.init();
    Tilt_Motor.init();
    Roll_Motor.init();

    /* FOC Init */
    Pan_Motor.initFOC();
    Tilt_Motor.initFOC();
    Roll_Motor.init();

    /* Motor Monitoring */
    Pan_Motor.useMonitoring(Serial);
    Tilt_Motor.useMonitoring(Serial);
    Roll_Motor.useMonitoring(Serial);

    /* Simple FOC Debug */
    SimpleFOCDebug::enable(&Serial);

    /* Commander Init */
    commander.add('M', Do_Motor, "Terminal Commander For Motor");  

    /* Config Motor */
    configureMotor(Pan_Motor, Pan_Conf);
    configureMotor(Tilt_Motor, Tilt_Conf);
    configureMotor(Roll_Motor, Roll_Conf);

}

void FOC_Run(){
    Pan_Motor.loopFOC();
    Tilt_Motor.loopFOC();
    Roll_Motor.loopFOC();
}

void Pan_Move(float Pan_Move_Angle_Rad){
    Roll_Motor.move(Pan_Move_Angle_Rad);
}

void Tilt_Move(float Tilt_Move_Angle_Rad){
    Roll_Motor.move(Tilt_Move_Angle_Rad);
}

void Roll_Move(float Roll_Move_Angle_Rad){
    Roll_Motor.move(Roll_Move_Angle_Rad);
}

void Motor_Monitor_Run(){
    Pan_Motor.monitor();
    Tilt_Motor.monitor();
    Roll_Motor.monitor();
}

void Commander_Run(){
    commander.run();
}


