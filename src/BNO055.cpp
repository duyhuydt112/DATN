#include <BNO055.h>

Adafruit_BNO055 BNO055 = Adafruit_BNO055(55, 0x28);

void BNO055_Setup(){
    Wire.begin(BNO055_SDA_PIN, BNO055_SCL_PIN);
    BNO055.setExtCrystalUse(true);
}

std::array<float, 3> Get_BNO055_BasePoint(){
    std::array<float, 3> Base_Point;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);
    Base_Point[0] = euler.z();  //Roll
    Base_Point[1] = euler.y();  //Pitch
    Base_Point[2] = euler.x();  //Yaw
    return Base_Point;
}

std::array<float, 3> Read_Angle_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point) {
    std::array<float, 3> Current_Rotation;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);
    Current_Rotation[2] = euler.x() - Base_Point[0]; 
    Current_Rotation[1] = euler.y() - Base_Point[1]; 
    Current_Rotation[0] = euler.z() - Base_Point[2]; 

    Serial.print("YAW: "); Serial.println(Yaw);
    Serial.print("PITCH: "); Serial.println(Pitch);
    Serial.print("ROLL: "); Serial.println(Roll);
  return Current_Rotation;
}