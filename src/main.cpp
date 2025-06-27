#include <SimpleFOC.h>
#include <SPI.h>
// Motor + driver
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(26,22,21,25);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 17);

// INA240-like qua InlineCurrentSense
InlineCurrentSense current_sense = InlineCurrentSense(0.05f, 50.0f, 36, 39);

// Torque điều khiển
float target_torque = 0.0;
Commander command = Commander(Serial);
void onTorque(char* buf){ target_torque = atof(buf); }

void setup(){
  Serial.begin(115200);
  SPI.begin(18,23,19);
  sensor.init(); motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 5.2;
  driver.pwm_frequency = 20000;
  driver.init(); motor.linkDriver(&driver);

  current_sense.init();
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_sensor_align = 3;
  motor.PID_current_q.P = 2.0;
  motor.PID_current_q.I = 10.0;
  motor.PID_current_q.D = 0.001;
  motor.LPF_current_q.Tf = 0.01;
  motor.PID_current_d.P = 1.0;
  motor.PID_current_d.I = 5.0;
  motor.PID_current_d.D = 0.001;
  motor.LPF_current_d.Tf = 0.01;
  int align_result = current_sense.driverAlign(motor.voltage_sensor_align);
  Serial.print("Align result: ");
  Serial.println(align_result);
  motor.monitor_variables = _MON_TARGET | _MON_CURR_Q | _MON_ANGLE;
  motor.useMonitoring(Serial); motor.monitor_downsample = 10;

  motor.init(); motor.initFOC();

  command.add('T', onTorque, "target torque (voltage)");
  Serial.println("Nhập T 0.5 để đặt torque");
}

void loop(){
  motor.loopFOC();
  motor.move(target_torque);
  motor.monitor();
  command.run();
}
