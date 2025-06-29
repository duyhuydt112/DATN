#ifndef ESP_NOW_MODE_H
#define ESP_NOW_MODE_H

#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>
#include <esp_wifi.h>

extern uint8_t sendingAddress[6];    // Địa chỉ MAC của ESP gửi
extern uint8_t receivingAddress[6];  // Địa chỉ MAC của ESP nhận

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void readMacAddress();
void Init_ESPNow();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

struct struct_message {
  float Azimuth_Joystick;
  float Elevation_Joystick;
  float Roll_Joystick;
  float Pan;
  float Tilt;
  float Roll;
};

extern struct_message myData;         // dữ liệu gửi
extern struct_message receivedData;   // dữ liệu nhận
extern struct_message incomingData;

#endif