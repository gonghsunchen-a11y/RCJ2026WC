#ifndef MAIN_CORE_H
#define MAIN_CORE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Dual_Core_Config.h"

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Button Pins ---
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26

// --- Ultrasonic ---
#define TRIG_F 2
#define ECHO_F 6

#define TRIG_R 3
#define ECHO_R 8

#define TRIG_B 4
#define ECHO_B 9

#define TRIG_L 5
#define ECHO_L 10

#define US_COUNT 4
#define US_INVALID_DISTANCE 999.0f
#define US_FILTER_ALPHA 0.35f
#define US_INVALID_LIMIT 3

// --- 1. Blueprints (Struct Definitions) ---
// We define these so every file knows the "shape" of the data.
struct CamData {
    uint16_t ball_x = 65535; uint16_t ball_y = 65535;
    uint16_t ball_w = 65535; uint16_t ball_h = 65535;
    bool ball_valid = false;
    uint16_t goal_x = 65535; uint16_t goal_y = 65535;
    uint16_t goal_w = 65535; uint16_t goal_h = 65535;
    bool goal_valid = false;
};

struct BallData {
    uint16_t dist = 65535; uint16_t angle = 65535;
    bool exist = false;
};

struct USSensor {
    uint16_t dist_b = 0; uint16_t dist_l = 0;
    uint16_t dist_r = 0; uint16_t dist_f = 0;
};

struct RobotMovement {
    float vx = 0.0f; float vy = 0.0f; float rot_v = 0.0f;
    uint16_t heading = 0; // Target heading in degrees
};

struct BT_PacketData {
    float posX;
    float posY;
    float ballDistance;
    float ballAngle;
    bool hasPossession;
    int currentRole; // Sent so the teammate knows what you picked
};


// --- 3. External Variables ---
// These tell the compiler "The actual memory for these is in main.cpp"
extern CamData camData;
extern BallData ballData;
extern USSensor usData;
extern RobotMovement robotMovement;
extern Adafruit_SSD1306 display;


// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void kicker_control(bool kick);
bool UI_Interface();
void sendMotor(float vx, float vy, float rot_v, int heading);
void sendMotorAndGetSensors(float vx, float vy, float rot_v, int heading);
void get_ball_sensor();
void ATTACK_send_data_to_esp();
void DEFENSE_read_data_from_esp();
void DEFENSE_send_role_switch();
#endif