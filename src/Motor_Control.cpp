#include "Motor_Control.h"

/* Encoder Object Init */
MagneticSensorSPI Pan_Encoder = MagneticSensorSPI(AS5048_SPI, PAN_CS_PIN);
MagneticSensorSPI Roll_Encoder = MagneticSensorSPI(AS5048_SPI, ROLL_CS_PIN);
MagneticSensorSPI Tilt_Encoder = MagneticSensorSPI(AS5048_SPI, TILT_CS_PIN);

/* Driver Object Init */
BLDCDriver3PWM Pan_Driver = BLDCDriver3PWM(PAN_IN1_PIN, PAN_IN2_PIN, PAN_IN3_PIN, PAN_EN_PIN);
BLDCDriver3PWM Tilt_Driver = BLDCDriver3PWM(TILT_IN1_PIN, TILT_IN2_PIN, TILT_IN3_PIN, TILT_EN_PIN);
BLDCDriver3PWM Roll_Driver = BLDCDriver3PWM(ROLL_IN1_PIN, ROLL_IN2_PIN, ROLL_IN3_PIN, ROLL_EN_PIN);

/* BLDC Object Init */
BLDCMotor Pan_Motor = BLDCMotor(GM4108_PAIR_POLES, GM4108_KV_RATING);
BLDCMotor Tilt_Motor = BLDCMotor(GM3506_PAIR_POLES, GM4108_KV_RATING);
BLDCMotor Roll_Motor = BLDCMotor(GM3506_PAIR_POLES, GM3506_KV_RATING);

/* Commander via Terminal */
Commander commander = Commander(Serial);
std::array<bool, 3> Motor_Commander_Enable = {false, false, true};

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

void Driver_Setup(BLDCDriver3PWM& Driver_Conf){
    Driver_Conf.pwm_frequency = 20000;
    Driver_Conf.voltage_power_supply = 12;
    Driver_Conf.voltage_limit = 12;
}

void Linking_With_Motor(BLDCDriver3PWM& Driver_Conf, MagneticSensorSPI& Encoder_Conf, BLDCMotor& Motor, MotionControlType Type, bool has_Encoder){
    Motor.linkDriver(&Driver_Conf);
    if (has_Encoder == true)
        Motor.linkSensor(&Encoder_Conf);
    Motor.controller = Type;
}

void Motor_Setup(const MotorConfig& Pan_Conf, const MotorConfig& Tilt_Conf, const MotorConfig& Roll_Conf){

    if (Motor_Commander_Enable[0]) {
        Driver_Setup(Pan_Driver);
        Pan_Driver.init();
        Pan_Encoder.init();
        Linking_With_Motor(Pan_Driver, Pan_Encoder, Pan_Motor, MotionControlType::angle, true);
        Pan_Motor.init();
        Pan_Motor.initFOC();
        Pan_Motor.useMonitoring(Serial);
        configureMotor(Pan_Motor, Pan_Conf);
    }
    if (Motor_Commander_Enable[1]) {
        Driver_Setup(Tilt_Driver);
        Tilt_Driver.init();
        Tilt_Encoder.init();
        Linking_With_Motor(Tilt_Driver, Tilt_Encoder, Tilt_Motor, MotionControlType::angle, true);
        Tilt_Motor.init();
        Tilt_Motor.initFOC();
        Tilt_Motor.useMonitoring(Serial);
        configureMotor(Tilt_Motor, Tilt_Conf);
    }
    if (Motor_Commander_Enable[2]) {
        Driver_Setup(Roll_Driver);
        Roll_Driver.init();
        Roll_Encoder.init();
        Linking_With_Motor(Roll_Driver, Roll_Encoder, Roll_Motor, MotionControlType::angle, false);
        Roll_Motor.init();
        Roll_Motor.initFOC();
        Roll_Motor.useMonitoring(Serial);
        configureMotor(Roll_Motor, Roll_Conf);
    }

    /* Simple FOC Debug */
    SimpleFOCDebug::enable(&Serial);

    /* Commander Init */
    commander.add('M', Do_Motor, "Terminal Commander For Motor");  

}

void FOC_Run(){
    if (Motor_Commander_Enable[0]) Pan_Motor.loopFOC();
    if (Motor_Commander_Enable[1]) Tilt_Motor.loopFOC();
    if (Motor_Commander_Enable[2]) Roll_Motor.loopFOC();
}

void Pan_Move(float Pan_Move_Angle_Rad){
    Pan_Motor.move(Pan_Move_Angle_Rad);
}

void Tilt_Move(float Tilt_Move_Angle_Rad){
    Tilt_Motor.move(Tilt_Move_Angle_Rad);
}

void Roll_Move(float Roll_Move_Angle_Rad){
    Roll_Motor.move(Roll_Move_Angle_Rad);
}

void Motor_Monitor_Run(){
    // Pan_Motor.monitor();
    // Tilt_Motor.monitor();
    Pan_Motor.monitor();
}

void Commander_Run(){
    commander.run();
}

void Pan_Motor_Control(){}



