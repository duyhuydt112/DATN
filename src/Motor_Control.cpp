#include "Motor_Control.h"
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

void Motor_Setup(){
    Serial.begin(11500);
    MagneticSensorSPI Pan_Encoder = MagneticSensorSPI(AS5048_SPI, PAN_CS_PIN);
    MagneticSensorSPI Roll_Encoder = MagneticSensorSPI(AS5048_SPI, ROLL_CS_PIN);
    MagneticSensorSPI Tilt_Encoder = MagneticSensorSPI(AS5048_SPI, TILT_CS_PIN);
    BLDCDriver3PWM Pan_Driver = BLDCDriver3PWM(PAN_IN1_PIN, PAN_IN2_PIN, PAN_IN2_PIN, PAN_EN_PIN);
    BLDCDriver3PWM Tilt_Driver = BLDCDriver3PWM(TILT_IN1_PIN, TILT_IN2_PIN, TILT_IN3_PIN, TILT_EN_PIN);
    BLDCDriver3PWM Roll_Driver = BLDCDriver3PWM(ROLL_IN1_PIN, ROLL_IN2_PIN, ROLL_IN3_PIN, ROLL_EN_PIN);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    Pan_Encoder.init();
    Roll_Encoder.init();
    Tilt_Encoder.init();

}

int Read_Motor_State(int CS_Pin){
    // Gửi lệnh đọc góc 0xFFFF (theo datasheet)
    uint16_t command = 0xFFFF;
    uint16_t result;

    digitalWrite(CS_Pin, LOW);
    delayMicroseconds(1);

    SPI.transfer16(command);  // Bỏ response đầu (dummy read)
    digitalWrite(CS_Pin, HIGH);
    delayMicroseconds(1);

    // Lần đọc thực sự
    digitalWrite(CS_Pin, LOW);
    delayMicroseconds(1);
    result = SPI.transfer16(0x0000); // Đọc giá trị góc
    digitalWrite(CS_Pin, HIGH);

    return result & 0x3FFF; // Chỉ lấy 14-bit thấp

}

