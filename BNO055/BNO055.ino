/* Note:
 *  Yaw: góc quay quanh trục Z (quay mặt bàn)
 *  Pitch: góc cúi/ngửa quanh trục Y
 *  Roll: góc nghiêng trái/phải quanh trục X
 */

// Định nghĩa chân I2C
#define SCL_PIN 22
#define SDA_PIN 21

// Thư viện
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Khởi tạo cảm biến
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Hàm kiểm tra kết nối cảm biến
void CheckBNO055() {
  if (!bno.begin()) {
    Serial.println("Không tìm thấy BNO055, kiểm tra lại kết nối!");
    while (1);
  } else {
    Serial.println("BNO055 kết nối hoàn tất!");
  }
  delay(1000);
}

// Hàm đọc góc (sửa: truyền tham chiếu)
void ReadAngleThreeAxes(float &yaw, float &pitch, float &roll) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yaw   = euler.x();  // Yaw (Z)
  pitch = euler.y();  // Pitch (Y)
  roll  = euler.z();  // Roll (X)

  Serial.print("YAW: "); Serial.println(yaw);
  Serial.print("PITCH: "); Serial.println(pitch);
  Serial.print("ROLL: "); Serial.println(roll);
}

// Hàm phản hồi AI
void AI_Talk(int a, int b, int c, int d) {
  if (a == 1) Serial.println("BNO055 đang mất cân bằng");
  else        Serial.println("BNO055 đang cân bằng");

  // Ưu tiên khi hoàn toàn thẳng (mọi góc đều 0)
  if ((b == 0) && (c == 0) && (d == 0)) {
    Serial.println("BNO055 đang ở trạng thái thẳng");
  } else {
    // Yaw (b)
    if (b == 1)      Serial.println("BNO055 đang quay trái (Yaw < 180)");
    else if (b == 2) Serial.println("BNO055 đang quay phải (Yaw > 180)");

    // Pitch (c)
    if (c == 1)      Serial.println("BNO055 đang cúi xuống (Pitch < 0)");
    else if (c == 2) Serial.println("BNO055 đang ngửa lên (Pitch > 0)");

    // Roll (d)
    if (d == 1)      Serial.println("BNO055 đang nghiêng trái (Roll < 0)");
    else if (d == 2) Serial.println("BNO055 đang nghiêng phải (Roll > 0)");
  }
}


// Hàm nhận diện hướng và phản hồi
void ReHuongAI(float yaw, float pitch, float roll) {
  int tmp = 1;
  int cnt = 0;
  int cnr = 0;
  int cnp = 0;
  if ((yaw == 180) && (pitch == 0) && (roll == 0)) {
    tmp = 0;
  }

  if (yaw < 180)      cnt = 1;
  else if (yaw > 180) cnt = 2;
  else                cnt = 0;

  if (pitch < 0)      cnr = 1;
  else if (pitch > 0) cnr = 2;
  else                cnr = 0;

  if (roll < 0)       cnp = 1;
  else if (roll > 0)  cnp = 2;
  else                cnp = 0;
  AI_Talk(tmp, cnt, cnr, cnp);
  Serial.print("TMP: "); Serial.print(tmp);
  Serial.print("  CNT: "); Serial.println(cnt);
  Serial.print("  CNR: "); Serial.print(cnr);
  Serial.print("  CNP: "); Serial.println(cnp);
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(1000);

  CheckBNO055();
  bno.setExtCrystalUse(true);
}

void loop() {
  float yaw = 0.0, pitch = 0.0, roll = 0.0;
  ReadAngleThreeAxes(yaw, pitch, roll);
  ReHuongAI(yaw, pitch, roll);
  delay(500); // Đọc mỗi 500ms
}
