#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

/* Define Pin */
#define PAN_CS_PIN 5
#define ROLL_CS_PIN 16
#define TILT_CS_PIN 17
#define MOSI 19
#define MISO 23
#define CLK 18

/* Define Function*/
void Spi_Setup(int Clk_Pin, int Mosi_Pin, int Miso_Pin);
int Read_Motor_State();

#endif 
