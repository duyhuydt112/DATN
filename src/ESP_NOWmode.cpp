#include <ESP_NOWmode.h>
#include <Joystick.h>
#include <Gimbal_High_Planner.h>
#include <SimpleFOC.h>

/* Mac Address of other ESP32*/
uint8_t sendingAddress[6]   = {0x14, 0x33, 0x5c, 0x48, 0x0b, 0x70};
uint8_t receivingAddress[6] = {0x34, 0x5F, 0x45, 0xAA, 0x8E, 0xCC};

/* Struct instance declaration*/
struct_send_message myData;
struct_receive_message receivedData;
esp_now_peer_info_t peerInfo;

/* Number of Data's Deadzone and Filter*/
float filterOffsetX = 0.0f;
float filterOffsetY = 0.0f;

/* Button State Mode*/
int current_Button = 0;
bool last_Button_Pressed = HIGH;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("----------------------------------------------");
}

// Function of ESP Slave receive from ESP master
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.printf("ESP_NOW_READ_DATA_JOYSTICK: x = %.2f, y = %.2f \n", receivedData.Xval, receivedData.Yval);
  Serial.print("Button: "); Serial.println(receivedData.Button ? "PRESSED" : "RELEASED");
  Serial.println("----------------------------------------------");

  if (receivedData.Button && !last_Button_Pressed)
  {
    current_Button = (current_Button + 1) % 5;
    if      (current_Button == 0) Serial.println("Motor Control (Pan Axis): ");
    else if (current_Button == 1) Serial.println("Motor Control (Tilt Axis): ");
    else if (current_Button == 2) Serial.println("Motor Control (Roll Axis): ");
    else if (current_Button == 3) Serial.println("Turn Off Manual Mode, Transfer to Auto Catching Mode");
    else if (current_Button == 4) Serial.println("Turn Off Auto Catching Mode, Turn Off System");
    delay (50);
  }
  last_Button_Pressed = receivedData.Button;
  Serial.printf("ESP_NOW_READ_DATA_BNO055: PAN = %.2f, TILT = %.2f, ROLL = %.2f \n", receivedData.Pan, receivedData.Tilt, receivedData.Roll);
  Serial.println("---------------------------------------------------------------");
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void Init_ESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println("ESP-NOW mode initialized!!!");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  Serial.println("----------------------------------------------");

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, receivingAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW Ready to send data!");
  Serial.println("----------------------------------------------");
}

float applyDeadzoneAndFilter(float input, float &filtered) {
    if (abs(input) < deadzone) input = 0;
    filtered = alpha * input + (1.0 - alpha) * filtered;
    return filtered;
  }

void Operation_Set(float &maxVelocity1, float &maxVelocity2, float &maxVelocity3, float velocityScale, float  &atuoPan, float &autoTilt, float &autoRoll)
{
  float filterX = applyDeadzoneAndFilter(receivedData.Xval, filterOffsetX);
  float filterY = applyDeadzoneAndFilter(receivedData.Yval, filterOffsetY);
  if (current_Button == 0)
  {
    float vel = filterY * maxVelocity1 * velocityScale;
    Motor_Move(vel, 0, 0);
  }
  else if (current_Button == 1)
  {
    float vel = filterY * maxVelocity2 * velocityScale;
    Motor_Move(0, vel, 0);
  }
  else if (current_Button == 2)
  {
    float vel = filterY * maxVelocity3 * velocityScale;
    Motor_Move(0, 0, vel);
  }
  else if (current_Button == 3)
  {
    Motor_Move(0, 0, 0);
    delay(1000);
    Motor_Move(atuoPan, autoTilt, autoRoll);
  }
  Motor_Monitor_Run();
  Commander_Run();
}
