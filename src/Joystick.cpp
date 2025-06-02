#include <Joystick.h>

/* Axis Rotation */
float Pan_Degree = 0;
float Tilt_Degree = 0;
float Roll_Degree = 0;

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
    Axis_Value[0] = VRX_Value - JOY_CENTER;
    Axis_Value[1]= VRY_Value - JOY_CENTER;
    return Axis_Value;
}

void Turn_Off_Manual_Mode(){
    Manual_Mode_Flag = false;
}

void Button_State(){
     bool Joystick_Pressed = digitalRead(SW_PIN) == LOW;
    if(Joystick_Pressed && !Last_Joystick_Pressed){
        Current_State = (Current_State + 1) % 3;
    }
    Last_Joystick_Pressed = Joystick_Pressed;
}

void Joystick_Run(){

    /* Init Variables */
    unsigned long currentMillis = millis();
    float deltaTime = (currentMillis - prevMillis) / 1000.0f; 
    prevMillis = currentMillis;
    std::array<float, 2> Axis_Value = Joystick_Input_Processing();

    /* Check Joystick Button Value */
    Button_State();

    /* Check Joystick Pan_Axis Move*/
    if (abs(Axis_Value[0]) > DEADZONE && Current_State == Press_State::PAN_STATE) {
        float direction = (Axis_Value[0] > 0) ? 1.0f : -1.0f;
        float deltaAngle = direction * SPEED_RAD_PER_SECOND * deltaTime;
        float nextAngle = Azimuth_Angle + deltaAngle;

        // Range in [-π, π]
        if (nextAngle < -PI * 0.999999f) {
            Azimuth_Angle = -PI * 0.999999f;
        } 
        else if (nextAngle > PI * 0.999999f) {
            Azimuth_Angle = PI * 0.999999f;
        } 
        else {
            Azimuth_Angle = nextAngle;
        }
        Pan_Move(Azimuth_Angle);
        Manual_Mode_Flag = true;
    }


    /* Check Joystick Tilt_Axis Move*/
    if (abs(Axis_Value[1]) > DEADZONE && Current_State == Press_State::TILT_STATE){
        float direction = (Axis_Value[1] > 0) ? 1.0f : -1.0f;
        float deltaAngle = direction * SPEED_RAD_PER_SECOND * deltaTime;
        Elevation_Angle += deltaAngle;
        Tilt_Move(Elevation_Angle);
        Manual_Mode_Flag = true;
    }

    if (abs(Axis_Value[0]) > DEADZONE && Current_State == Press_State::ROLL_STATE){
        float direction = (Axis_Value[0] > 0) ? 1.0f : -1.0f;
        float deltaAngle = direction * SPEED_RAD_PER_SECOND * deltaTime;
        Roll_Angle += deltaAngle;
        Roll_Move(Roll_Angle);
        Manual_Mode_Flag = true;
    }
}

