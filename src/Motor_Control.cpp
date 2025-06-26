#include "Motor_Control.h"

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------VARIABLES-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

/* Encoder Object Init */
MagneticSensorSPI Pan_Encoder = MagneticSensorSPI(AS5048_SPI, PAN_CS_PIN);
MagneticSensorSPI Tilt_Encoder = MagneticSensorSPI(AS5048_SPI, TILT_CS_PIN);
MagneticSensorSPI Roll_Encoder = MagneticSensorSPI(AS5048_SPI, ROLL_CS_PIN);

/* Driver Object Init */
BLDCDriver3PWM Pan_Driver = BLDCDriver3PWM(PAN_IN1_PIN, PAN_IN2_PIN, PAN_IN3_PIN, PAN_EN_PIN);
BLDCDriver3PWM Tilt_Driver = BLDCDriver3PWM(TILT_IN1_PIN, TILT_IN2_PIN, TILT_IN3_PIN, TILT_EN_PIN);
BLDCDriver3PWM Roll_Driver = BLDCDriver3PWM(ROLL_IN1_PIN, ROLL_IN2_PIN, ROLL_IN3_PIN, ROLL_EN_PIN);

/* BLDC Object Init */
BLDCMotor Pan_Motor = BLDCMotor(GM4108_PAIR_POLES, GM4108_KV_RATING);
BLDCMotor Tilt_Motor = BLDCMotor(GM4108_PAIR_POLES, GM4108_KV_RATING);
BLDCMotor Roll_Motor = BLDCMotor(GM3506_PAIR_POLES, GM3506_KV_RATING);

/* Current Sensor */
InlineCurrentSense Pan_Current_Sensor = InlineCurrentSense(PAN_R_SHUNT, PAN_INA_GAIN, PAN_CURRENT_SENSOR_A0, PAN_CURRENT_SENSOR_A1);
InlineCurrentSense Tilt_Current_Sensor = InlineCurrentSense(TILT_R_SHUNT, TILT_INA_GAIN, TILT_CURRENT_SENSOR_A0, TILT_CURRENT_SENSOR_A1);
InlineCurrentSense Roll_Current_Sensor = InlineCurrentSense(ROLL_R_SHUNT, ROLL_INA_GAIN, ROLL_CURRENT_SENSOR_A0, ROLL_CURRENT_SENSOR_A1);

/* Commander via Terminal */
Commander commander = Commander(Serial);
std::array<bool, 3> Motor_Enable = {false, false, true};

/* Global Variables */
bool Has_Pan_Angle_Target = false;
bool Has_Tilt_Angle_Target = false;
bool Has_Roll_Angle_Target = false;
float Pan_Target_Angle = 0;
float Tilt_Target_Angle = 0;
float Roll_Target_Angle = 0;
float Pan_Target_Current = 0;
float Tilt_Target_Current = 0;
float Roll_Target_Current = 0;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------Serial Commander-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

/* Serial Commander Function */

void on_Pan_Current_Target(char* cmd) {
  Pan_Target_Current = atof(cmd);  // đơn vị: Ampe
}

void on_Tilt_Current_Target(char* cmd) {
  Tilt_Target_Current = atof(cmd);  // đơn vị: Ampe
}

void on_Roll_Current_Target(char* cmd) {
  Roll_Target_Current = atof(cmd);  // đơn vị: Ampe
}

void on_Pan_Angle_Target(char* cmd) {
  Pan_Target_Angle = atof(cmd) * _PI / 180.0;
  Has_Pan_Angle_Target = true;  // đánh dấu đã có lệnh
}

void on_Tilt_Angle_Target(char* cmd) {
  Tilt_Target_Angle = atof(cmd) * _PI / 180.0;
  Has_Tilt_Angle_Target = true;  // đánh dấu đã có lệnh
}

void on_Roll_Angle_Target(char* cmd) {
  Roll_Target_Angle = atof(cmd) * _PI / 180.0;
  Has_Roll_Angle_Target = true;  // đánh dấu đã có lệnh
}
/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------Configure-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

/* *********************** LƯU Ý ***********************/
/* *********************** Thứ tự gọi hàm Config: Driver -> Motor ************************/

void Configure_Driver(const DriverConfig& config, BLDCDriver3PWM& driver) {
  driver.voltage_power_supply = config.Voltage_Power_Supply;
  driver.voltage_limit = config.Voltage_Limit;         
  driver.pwm_frequency = config.Pwm_Frequency;
}

void Configure_Motor(BLDCMotor& motor, const MotorConfig& motorConfig, const DriverConfig& driverConfig, const CurrentPID& pid) {

    // motor.current_limit = config.Current_limit; // Amps - default 0.2Amps
    motor.voltage_limit = driverConfig.Voltage_Limit; // Volts - default driver.voltage_limit
    motor.current_limit = motorConfig.Current_limit;
    MAX_FUZZY_INPUT_ERROR = driverConfig.Voltage_Limit;
    MAX_FUZZY_OUTPUT_CONTROL_VOLTAGE = driverConfig.Voltage_Limit;

    // motor.LPF_velocity = motorConfig.Velocity_LPF_Tf;
    // motor.velocity_limit = motorConfig.Velocity_limit;
    motor.monitor_variables = motorConfig.Monitor_Variables;
    motor.monitor_downsample = motorConfig.Monitor_Downsample;
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.PID_current_q.P = pid.P_Current_q;
    motor.PID_current_q.I = pid.I_Current_q;
    motor.PID_current_q.D = pid.I_Current_q;
    motor.PID_current_q.output_ramp = pid.Output_Ramp_Current_q;
    motor.LPF_current_q.Tf = pid.Tf_LPF_Current_q;

    motor.PID_current_d.P = pid.P_Current_d;
    motor.PID_current_d.I = pid.P_Current_d;
    motor.PID_current_d.D = pid.P_Current_d;
    motor.PID_current_d.output_ramp = pid.Output_Ramp_Current_d;
    motor.LPF_current_d.Tf = pid.Tf_LPF_Current_d;
    motor.voltage_sensor_align = 2.5;
}


void Linking_With_Motor(BLDCDriver3PWM& Driver_Conf, MagneticSensorSPI& Encoder_Conf, BLDCMotor& Motor, InlineCurrentSense& Current_Sense){
    Motor.linkDriver(&Driver_Conf);
    Motor.linkSensor(&Encoder_Conf);
    Motor.linkCurrentSense(&Current_Sense);
}

void Controller_Setup(const MotorConfig& Pan_Conf, const MotorConfig& Tilt_Conf, const MotorConfig& Roll_Conf, const DriverConfig& Driver, const CurrentPID& Pan_Curr_Conf, const CurrentPID& Tilt_Curr_Conf, const CurrentPID& Roll_Curr_Conf){

if (Motor_Enable[0]) {
    Configure_Driver(Driver, Pan_Driver);
    Pan_Driver.init();
    Pan_Encoder.init();
    Pan_Current_Sensor.linkDriver(&Pan_Driver);
    // Pan_Current_Sensor.gain_b *=-1; // if b inverted
    // Pan_Current_Sensor.gain_a *=-1;
    // Pan_Current_Sensor.skip_align = true;
    Pan_Current_Sensor.init();
    // ---- In dòng align thử ngay sau khi init current sense ----
    Serial.println(">>> Start printing current during Align (Phase A active)");
    Pan_Driver.setPwm(1.0f, 0.0f, 0.0f); // bật pha A
    delay(1000);
    int align_result = Pan_Current_Sensor.driverAlign(Pan_Motor.voltage_sensor_align);
    Serial.print("Align result: ");
    Serial.println(align_result);
  
    Pan_Driver.setPwm(0, 0, 0);
    Serial.println(">>> Done printing Align currents");

    // ---- Tiếp tục khởi tạo như bình thường ----
    Linking_With_Motor(Pan_Driver, Pan_Encoder, Pan_Motor, Pan_Current_Sensor);
    Configure_Motor(Pan_Motor, Pan_Conf, Driver, Pan_Curr_Conf);
    Pan_Motor.useMonitoring(Serial);
    Pan_Motor.init();
    Pan_Motor.initFOC(0, Direction::CW);
    
    Serial.print("Pan Initial angle: ");
    Pan_Target_Angle = Pan_Encoder.getAngle();
    Serial.println(Pan_Target_Angle, 4);
}

    if (Motor_Enable[1]) {
        Configure_Driver(Driver, Tilt_Driver);
        Tilt_Driver.init();
        Tilt_Encoder.init();
        Tilt_Current_Sensor.linkDriver(&Tilt_Driver);
        Tilt_Current_Sensor.init();
        Linking_With_Motor(Tilt_Driver, Tilt_Encoder, Tilt_Motor, Tilt_Current_Sensor);
        Configure_Motor(Tilt_Motor, Tilt_Conf, Driver, Tilt_Curr_Conf);
        Tilt_Motor.useMonitoring(Serial);
        Tilt_Motor.init();
        Tilt_Motor.initFOC(0, Direction::CW);
        Serial.print("Tilt Initial angle: ");
        Tilt_Target_Angle = Tilt_Encoder.getAngle();
        Serial.println(Tilt_Target_Angle, 4);
    }
    if (Motor_Enable[2]) {
        Configure_Driver(Driver, Roll_Driver);
        Roll_Driver.init();
        Roll_Encoder.init();
        Roll_Current_Sensor.linkDriver(&Roll_Driver);
        Roll_Current_Sensor.init();
        Linking_With_Motor(Roll_Driver, Roll_Encoder, Roll_Motor, Roll_Current_Sensor);
        Configure_Motor(Roll_Motor, Roll_Conf, Driver, Roll_Curr_Conf);
        Roll_Motor.useMonitoring(Serial);
        Roll_Motor.init();
        Roll_Motor.initFOC(0, Direction::CW);
        Serial.print("Roll Initial angle: ");
        Roll_Target_Angle = Roll_Encoder.getAngle();
        Serial.println(Roll_Target_Angle, 4);
    }

    /* Simple FOC Debug */
    SimpleFOCDebug::enable(&Serial);

    /* Commander Init */
    commander.add('P', on_Pan_Current_Target, "Terminal Commander For Pan Motor");
    commander.add('T', on_Tilt_Current_Target, "Terminal Commander For Tilt Motor");  
    commander.add('R', on_Roll_Current_Target, "Terminal Commander For Roll Motor");    
    Serial.println("Motor ready.");
    Serial.println("Send command");


}

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------RUN FUNCTION-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */


void PID_Run(const MotorConfig& motorConfig, MagneticSensorSPI& sensor, PID_Calculate& PID, bool& Has_Angle_Target, float& Target_Angle, BLDCMotor& Motor){
    if ((millis() - PID.Last_Time) >= PID.Delta_Time && Has_Angle_Target) {
    float Current_angle = Motor.shaft_angle - sensor.getAngle();     
    PID.Error = Target_Angle - Current_angle;

    if (abs(PID.Error) < 0.01) PID.Error = 0;

    PID.Error_Integral += PID.Error * (PID.Delta_Time  / 1000.0);
    PID.Error_Integral = constrain(PID.Error_Integral, -1.0, 1.0); 
    PID.Error_Derivative = (PID.Error - PID.Error_Prev) / (PID.Delta_Time / 1000.0);
    
    PID.Torque_cmd = motorConfig.PID_P * PID.Error + motorConfig.PID_I * PID.Error_Integral + motorConfig.PID_D * PID.Error_Derivative;
    PID.Torque_cmd = constrain(PID.Torque_cmd, -motorConfig.Voltage_limit, motorConfig.Voltage_limit);

    PID.Error_Prev = PID.Error;
    PID.Last_Time = millis();

    Serial.print("Err: "); Serial.print(PID.Error, 4);
    Serial.print(" | Torque: "); Serial.print(PID.Torque_cmd, 4);
    Serial.print(" | Angle: "); Serial.println(Current_angle, 4);
  }
}

void FOC_Run(){
    if (Motor_Enable[0]){
        Pan_Motor.loopFOC();
    }

    if (Motor_Enable[1]){
        Tilt_Motor.loopFOC();
    }

    if (Motor_Enable[2]){
        Roll_Motor.loopFOC();
    }
}

void Motor_Move(float Pan_Move_Angle_Degree, float Tilt_Move_Angle_Degree, float Roll_Move_Angle_Degree){
    if (Motor_Enable[0]){
        Pan_Motor.move(Pan_Move_Angle_Degree);
        delay(10);
    }

    if (Motor_Enable[1]){
        Tilt_Motor.move(Tilt_Move_Angle_Degree);
        delay(10);
    }

    if (Motor_Enable[2]){
        Roll_Motor.move(Roll_Move_Angle_Degree);
        delay(10);
    }
}

void Motor_Monitor_Run(){
    if (Motor_Enable[0]){
        Pan_Motor.monitor();
    }

    if (Motor_Enable[1]){
        Tilt_Motor.monitor();
    }

    if (Motor_Enable[2]){
        Roll_Motor.monitor();
    }
}

void Commander_Run(){
    commander.run();
}