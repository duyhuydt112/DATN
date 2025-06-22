#include <Joystick.h>
#include <SimpleFOC.h>
#include <Gimbal_High_Planner.h>

/* Axis Rotation */
float Pan_Degree = 0;
float Tilt_Degree = 0;
float Roll_Degree = 0;

/* Deadzone and Filter Setup*/
const float deadzone = 0.15;
const float alpha    = 0.05;

/* Azimuth Angle && Radius */
float Azimuth_Angle = 0.0f;
float Elevation_Angle = 0.0f;
float Radius = 1.0f;
float Roll_Angle = 0.0f;

/* Time Variables */
unsigned long prevMillis = 0;

/* Joystick Setup Function*/
void Joystick_Init(){
    prevMillis = millis();
    pinMode(SW_PIN, INPUT_PULLUP);
}

/* Button Pressed Variables */
bool Last_Joystick_Pressed = HIGH;
int Current_State = 0;

/* Move Gimbal Axis Manual */
extern bool Manual_Mode_Flag = false;

std::array<float, 2> Joystick_Input_Processing(){
    std::array<float, 2> Axis_Value = {0.0f, 0.0f};
    int VRX_Value = analogRead(VRX_PIN);
    int VRY_Value = analogRead(VRY_PIN);
    Axis_Value[0] = (VRX_Value - JOY_CENTER) / JOY_CENTER;
    Axis_Value[1]=  (VRY_Value - JOY_CENTER) / JOY_CENTER;
    return Axis_Value;
    /*
        Value Target: VrX of Joystick -> Axis_Value[0]
                      VrY of Joystick -> Axis_Value[1]
                      JOY_CENTER is 2048.0f
    */
}

void Turn_Off_Manual_Mode(){
    Manual_Mode_Flag = false;
}

void Button_State(){
     bool Joystick_Pressed = digitalRead(SW_PIN) == LOW;
    if(Joystick_Pressed && !Last_Joystick_Pressed){
        Current_State = (Current_State + 1) % 5;
    }
    Last_Joystick_Pressed = Joystick_Pressed;
    /*
    Mode Set: Current_State = 0 -> Pan  Control
              Current_State = 1 -> Tilt Control
              Current_State = 2 -> Roll Control
              Current_State = 3 -> Turn Off Manual Set, Turn On Auto Catching Face Set
              Current_State = 4 -> Turn Off Auto Catching Face Set, Power Off -> Stop Machine
    */
}

float applyDeadzoneAndFilter(float input, float &filtered) {
  if (abs(input) < deadzone) input = 0;
  filtered = alpha * input + (1.0 - alpha) * filtered;
  return filtered;
}