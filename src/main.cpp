#include <Joystick.h>
#include <Gimbal_Position_Control.h>

void setup() {
  Serial.begin(115200);
  Joystick_Init();
}

void loop() {
  Joystick_Run();
}
