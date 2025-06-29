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
//SimpleKalmanFilter VrX(2.0f, 2.0f, 0.01f); // Kalman filter for X-axis
//SimpleKalmanFilter VrY(2.0f, 2.0f, 0.01f); // Kalman filter for Y-axis
/* Joystick Setup Function*/
void Joystick_Init(){
    Serial.println("Dang ket noi den Joystick.........................");
    prevMillis = millis();
    pinMode(SW_PIN, INPUT_PULLUP);
    delay(1000);
    Serial.println("Joystick ket noi thanh cong!!!!");
    Serial.println("-------------------------");
}

/* Button Pressed Variables */
bool Last_Joystick_Pressed = HIGH;
int Current_State = 0;

/* Move Gimbal Axis Manual */
extern bool Manual_Mode_Flag = false;

std::array<float, 2> Joystick_Input_Processing(){
    std::array<float, 2> Axis_Value = {0.0f, 0.0f};
    int VRX_Value = analogRead(VRX_PIN); // 0 - 4098
    int VRY_Value = analogRead(VRY_PIN); // 0 - 4098
   // Serial.println(VRX_Value);
    Axis_Value[0] = (VRX_Value - JOY_CENTER) / JOY_CENTER; // Calib ve -1 va 1
    Axis_Value[1] = (VRY_Value - JOY_CENTER) / JOY_CENTER; // Calib ve -1 va 1
    return Axis_Value;
}

void Turn_Off_Manual_Mode(){
    Manual_Mode_Flag = false;
}
// Function to handle button state changes
void Button_State(){
     bool Joystick_Pressed = digitalRead(SW_PIN) == LOW;
    if(Joystick_Pressed && !Last_Joystick_Pressed){
        Current_State = (Current_State + 1) % 3;
    }
    Last_Joystick_Pressed = Joystick_Pressed;
}
// 

std::array<float, 3> Joystick_Run() {
    std::array<float, 3> Angle_Value = {0.0f, 0.0f, 0.0f};
    unsigned long currentMillis = millis();
    float deltaTime = (currentMillis - prevMillis) / 1000.0f;
    prevMillis = currentMillis;

    std::array<float, 2> Axis_Value = Joystick_Input_Processing();
    Serial.print("VRX: "); Serial.println(Axis_Value[0], 2);
    Serial.print("VRY: "); Serial.println(Axis_Value[1], 2);
    Serial.print("Button State: "); Serial.println(Current_State);
    float xVal = Axis_Value[0], yVal = Axis_Value[1];
    Button_State();

    float direction, deltaAngle, nextAngle, limit;

    if (Current_State == Press_State::PAN_STATE && abs(xVal) > DEADZONE) {
        direction = (xVal > 0) ? 1.0f : -1.0f;
        deltaAngle = direction * SPEED_RAD_PER_SECOND * deltaTime;
        nextAngle = Azimuth_Angle + deltaAngle;
        limit = PI;
        Azimuth_Angle = constrain(nextAngle, -limit, limit);
        Manual_Mode_Flag = true;
    }

    if (Current_State == Press_State::TILT_STATE && abs(yVal) > DEADZONE) {
        direction = (yVal > 0) ? 1.0f : -1.0f;
        deltaAngle = direction * SPEED_RAD_PER_SECOND * deltaTime;
        nextAngle = Elevation_Angle + deltaAngle;
        limit = PI / 2;
        Elevation_Angle = constrain(nextAngle, -limit, limit);
        Manual_Mode_Flag = true;
    }

    if (Current_State == Press_State::ROLL_STATE && abs(xVal) > DEADZONE) {
        direction = (xVal > 0) ? 1.0f : -1.0f;
        deltaAngle = direction * SPEED_RAD_PER_SECOND * deltaTime;
        nextAngle = Roll_Angle + deltaAngle;
        limit = PI;
        Roll_Angle = constrain(nextAngle, -limit, limit);
        Manual_Mode_Flag = true;
    }
    Angle_Value[0] = degrees(Azimuth_Angle); // Convert radians to degrees
    Angle_Value[1] = degrees(Elevation_Angle);
    Angle_Value[2] = degrees(Roll_Angle);
    return Angle_Value;
/*
    Serial.print("Azimuth: "); Serial.println(degrees(Azimuth_Angle), 1);
    Serial.print("Elevation: "); Serial.println(degrees(Elevation_Angle), 1);
    Serial.print("Roll: "); Serial.println(degrees(Roll_Angle), 1);
    Serial.print("State: "); Serial.println(Current_State);*/
}

