#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <array>

#define BNO055_SDA_PIN 21  // hoặc D2 nếu dùng ESP8266
#define BNO055_SCL_PIN 22  // hoặc D1 nếu dùng ESP8266

Adafruit_BNO055 BNO055 = Adafruit_BNO055(55, 0x28);
sensors_event_t orientationData , angVelocityData , linearAccelData;

void BNO055_Setup(){
    Wire.begin(BNO055_SDA_PIN, BNO055_SCL_PIN);
    
    if (!BNO055.begin()) {
        Serial.println("Không tìm thấy cảm biến BNO055!");
        while (1);
    }

    delay(1000);
    BNO055.setExtCrystalUse(true);
}

std::array<float, 3> Get_BNO055_BasePoint(){
    std::array<float, 3> Base_Point;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);
    Base_Point[0] = euler.z();  // Roll
    Base_Point[1] = euler.y();  // Pitch
    Base_Point[2] = euler.x();  // Yaw
    return Base_Point;
}

std::array<float, 3> Read_Angle_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point) {
    std::array<float, 3> Current_Rotation;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);
    Current_Rotation[2] = euler.x() - Base_Point[2]; // Yaw
    Current_Rotation[1] = euler.y() - Base_Point[1]; // Pitch
    Current_Rotation[0] = euler.z() - Base_Point[0]; // Roll

    Yaw = Current_Rotation[2];
    Pitch = Current_Rotation[1];
    Roll = Current_Rotation[0];

    Serial.print("YAW: "); Serial.print(Yaw); Serial.print(" | ");
    Serial.print("PITCH: "); Serial.print(Pitch); Serial.print(" | ");
    Serial.print("ROLL: "); Serial.println(Roll);

    return Current_Rotation;
}
