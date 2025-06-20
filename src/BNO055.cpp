#include <BNO055.h>
#include <SimpleKalmanFilter.h>
#include <cmath>
//-------------------------------------------------------------------------------------------------------------------------//
//                                                                Chú thích:
//                 Chiều quay của các trục trên BNO055:
//                - Trục Z: Quay góc Roll
//                - Trục X: Góc quay Pan
//                - Trục Y: Góc quay Tilt
//-------------------------------------------------------------------------------------------------------------------------//
//     CẤU HÌNH PHẦN CỨNG      //
//-----------------------------//
#define BNO055_SDA_PIN 21
#define BNO055_SCL_PIN 22

Adafruit_BNO055 BNO055 = Adafruit_BNO055(55, 0x28);

//-----------------------------//
//     KALMAN CHO TỪNG TRỤC    //
//-----------------------------//
SimpleKalmanFilter kalmanPan(0.02, 0.1, 0.5);
SimpleKalmanFilter kalmanTilt(0.02, 0.1, 0.5);
SimpleKalmanFilter kalmanRoll(0.02, 0.1, 0.5);

//-----------------------------//
//        HÀM TIỆN ÍCH         //
//-----------------------------//

float degtorad(float degree) {
    return degree * PI / 180.0;
}

float normalizePan(float pan_deg) {
    if (pan_deg > 180.0f)
        return pan_deg - 360.0f;
    else
        return pan_deg;
}

float normalizeRoll(float roll_deg) {
    if (roll_deg > 180.0f)
        return roll_deg - 360.0f;
    else
        return roll_deg;
}

//-----------------------------//
//     KHỞI TẠO CẢM BIẾN       //
//-----------------------------//
void BNO055_Setup() {
    Wire.begin(BNO055_SDA_PIN, BNO055_SCL_PIN);

    if (!BNO055.begin()) {
        Serial.println("Failed to connect, please verify the connections!");
        while (1);
    }

    delay(1000);
    BNO055.setExtCrystalUse(true);
    Serial.println("BNO055 initialized successfully!");
}

//-----------------------------//
//      LẤY GÓC GỐC BAN ĐẦU    //
//-----------------------------//
std::array<float, 3> Get_BNO055_BasePoint() {
    std::array<float, 3> Base_Point;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);

    Base_Point[0] = normalizePan(euler.z());                        // Pan
    Base_Point[1] = euler.y();                     // Tilt
    Base_Point[2] = normalizeRoll(euler.x());                      // Roll
    return Base_Point;
}

//---------------------------------------------//
//      ĐỌC GÓC ĐỘ SAU KALMAN + SO VỚI GỐC GỐC //
//---------------------------------------------//
std::array<float, 3> Read_Angle_ThreeAxes_FromQuat(float &Pan, float &Tilt, float &Roll, std::array<float, 3> Base_Point) {
    std::array<float, 3> Current_Rotation;
    imu::Quaternion quat = BNO055.getQuat();
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);

    float w = quat.w();
    float x = quat.x();
    float y = quat.y();
    float z = quat.z();
      // Tính toán các thành phần
    float gx = 2.0 * (x * z - w * y);
    float gy = 2.0 * (y * z + w * x);
    float gz = w * w - x * x - y * y + z * z;
    
    // Tính toán góc Pan, Tilt, Roll từ quaternion
    float rawTilt = asin(constrain(w * w - x * x - y * y + z * z, -1.0, 1.0)) * 180.0 / PI;
    float rawRoll = normalizeRoll(euler.z());
    float rawPan  = normalizePan(euler.x());
    // Điều chỉnh góc so với Base_Point
    rawPan  -= Base_Point[0];   // Pan
    rawTilt -= Base_Point[1];   // Tilt
    rawRoll -= Base_Point[2];   // Roll
    // Cập nhật giá trị góc sau khi lọc Kalman
    Pan   = rawPan;
    Tilt  = rawTilt;
    Roll  = rawRoll;
    // Lưu vào mảng kết quả
    Current_Rotation[0] = Pan;
    Current_Rotation[1] = Tilt;
    Current_Rotation[2] = Roll;
    // In ra kết quả
    //Serial.print("PAN: ");   Serial.print(Pan, 2);  Serial.println(" °");
    //Serial.print("TILT: ");  Serial.print(Tilt, 2); Serial.println(" °");
    //Serial.print("ROLL: ");  Serial.print(Roll, 2); Serial.println(" °");
    //Serial.println("---------------------------------------------------------------------");
    // Trả về mảng chứa góc Pan, Tilt, Roll
    return Current_Rotation;
}

//----------------------------------------------//
//       GIỚI HẠN GÓC & XÁC ĐỊNH CHIỀU QUAY      //
//----------------------------------------------//
void LimitedAngle(float &Pan, float &Tilt, float &Roll, float Threshold, float limitedPan, float limitedTilt, float limitedRoll) {
    if (abs(Pan) > limitedPan)
    {
        Serial.println("PAN out of range!");
    }
    if (abs(Tilt) > limitedTilt)
    {
        Serial.println("TILT out of range!");
    }
    if (abs(Roll) > limitedRoll)
    {
        Serial.println("ROLL out of range!");
    }
    if (Pan > (Pan + Threshold)) {
        Serial.println("PAN is rotated CCW");
    } 
    else if (Pan < (Pan - Threshold)) {
        Serial.println("PAN is rotated CW");
    }
    else
    {
        Serial.println("PAN is not rotated");
    }
    if (Roll > (Roll + Threshold)) {
        Serial.println("ROLL is rotated CCW");
    } 
    else if (Roll < (Roll - Threshold)) {
        Serial.println("ROLL is rotated CW");
    }
    else
    {
        Serial.println("ROLL is not rotated");
    }
    if (Tilt > (Tilt + Threshold)) {
        Serial.println("TILT is rotated CCW");
    } 
    else if (Tilt < (Tilt - Threshold)) {
        Serial.println("TILT is rotated CW");
    }
    else
    {
        Serial.println("TILT is not rotated");
    }
    Serial.println("---------------------------------------------------------------------");
}

//----------------------------------------------//
//     HIỂN THỊ TRẠNG THÁI HIỆU CHUẨN BNO055    //
//----------------------------------------------//
void PrintCalibrationStatus() {
    uint8_t sys, gyro, accel, mag;
    BNO055.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("Calibration >> SYS:");
    Serial.print(sys);
    Serial.print(" GYRO:");
    Serial.print(gyro);
    Serial.print(" ACC:");
    Serial.print(accel); 
    Serial.print(" MAG:");
    Serial.println(mag);
}
// Biến toàn cục lưu vector hướng chuẩn
float fwd_base[3] = {0.0f, 0.0f, 0.0f};

// Gọi hàm này 1 lần sau khi hệ thống ổn định
void SetBaseForwardVector() {
    imu::Quaternion quat = BNO055.getQuat();
    float x = 2 * (quat.x() * quat.z() + quat.w() * quat.y());
    float y = 2 * (quat.y() * quat.z() - quat.w() * quat.x());
    float z = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());

    fwd_base[0] = x;
    fwd_base[1] = y;
    fwd_base[2] = z;

    Serial.println("Base forward vector set.");
}

