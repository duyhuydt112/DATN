
#include <ESP_NOW_MODE.h>

uint8_t sendingAddress[]   = {0xEC, 0xE3, 0x34, 0x7B, 0x70, 0x28};
uint8_t receivingAddress[] = {0x14, 0x33, 0x5C, 0x48, 0x07, 0x6C};
// uint8_t receivingAddress[] = {0xEC, 0xE3, 0x34, 0x7B, 0x70, 0x28};
struct_message myData;
esp_now_peer_info_t peerInfo;
struct_message receivedData;


// Ham xac dinh du lieu
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("----------------------------------------------");
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
  Serial.println("Dang ket noi che do ESP_NOW..........................");
  WiFi.mode(WIFI_STA); // Chế độ STA
  WiFi.disconnect();   // Ngắt kết nối WiFi internet
  Serial.println("Da ket noi thanh cong ESP_NOW!!!");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Ket noi that bai, vui long kiem tra thiet bi ket noi!");
    return;
  }
  Serial.println("----------------------------------------------");
  esp_now_register_send_cb(OnDataSent); // Đăng ký callback gửi dữ liệu
  esp_now_register_recv_cb(OnDataRecv); // ✅ Thêm dòng này để đăng ký callback nhận
  memcpy(peerInfo.peer_addr, receivingAddress, 6);
  peerInfo.channel = 0;     // Sử dụng kênh hiện tại
  peerInfo.encrypt = false; // Không mã hóa dữ liệu

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW san sang truyen tai du lieu!!!");
  Serial.println("----------------------------------------------");
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Pan: "); Serial.println(receivedData.Pan);
}