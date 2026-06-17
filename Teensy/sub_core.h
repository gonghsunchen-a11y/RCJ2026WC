#ifndef SUB_CORE_H
#define SUB_CORE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

//Motor1
#define DIR_1 37     // 方向控制腳1
#define pwmPin1 4    // PWM 控制腳

//Motor2
#define DIR_2 11    // 方向控制腳2
#define pwmPin2 6   // PWM 控制腳

//Motor3
#define DIR_3 10    // 方向控制腳3
#define pwmPin3 5   // PWM 控制腳

//Motor4
#define DIR_4 36    // 方向控制腳4
#define pwmPin4 3   // PWM 控制腳

#define SLP1 23    
#define SLP2 12

// --- Multiplexer Pins ---
#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define M1 A0
#define M2 A1

struct LineData {
    uint32_t state = 0;
    bool active = false;
};

struct GyroData {
    float heading = 0.0f;
    float pitch = 0.0f;
    bool exist = false;
};


// --- Global Variable Declarations (Externs) ---
extern LineData lineData;           
extern MainCoreCommand mainCommand; // Added for MainCoreCommand usage  
extern GyroData gyroData;       
extern RobotStatus robot;        // Added to match Vector_Motion usage
extern uint16_t avg_ls[34];
extern BallData ballData;
extern Position RobotPos;
extern GoalData goalData;
extern USSensor usData;


// --- Core Function Prototypes ---
void sub_core_init();
int  readMux(int ch, int sig);
void update_line_sensor();
void fast_update_line_sensor();
void update_gyro_sensor();
void line_calibrate();

// --- Actuators & IK Prototypes ---
void SetMotorSpeed(uint8_t port, float speed);
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, float rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
#endif


