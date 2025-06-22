#ifndef ESP_NOWMODE_H
#define ESP_NOWMODE_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* Địa chỉ MAC của ESP */
extern uint8_t sendingAddress[6];
extern uint8_t receivingAddress[6];

/* Number of Data's Deadzone and Filter*/
extern float filterOffsetX;
extern float filterOffsetY;

/* Cấu trúc dữ liệu gửi */
typedef struct struct_message {
    float Xval;
    float Yval;
    bool Button;
    float Pan;
    float Tilt;
    float Roll;
} struct_send_message;

/* Cấu trúc dữ liệu nhận */
typedef struct __attribute__((packed)) {
    float Xval;
    float Yval;
    bool Button;
    float Pan;
    float Tilt;
    float Roll;
} struct_receive_message;

extern struct_send_message myData;
extern struct_receive_message receivedData;
extern esp_now_peer_info_t peerInfo;

/* Các hàm khởi tạo và callback */
void Init_ESPNow();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void readMacAddress();
void Operation_Set(float &maxVelocity1, float &maxVelocity2, float &maxVelocity3, float velocityScale, float  &atuoPan, float &autoTilt, float &autoRoll);
#endif
