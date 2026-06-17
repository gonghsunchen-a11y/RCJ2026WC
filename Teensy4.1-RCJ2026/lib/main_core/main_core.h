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
#define front_us A15
#define left_us A14
#define back_us A16
#define right_us A17
#define alpha 0.15

// --- Goal Localization Thresholds ---
#define Y_LOCALIZE_THRESHOLD_L 20
#define Y_LOCALIZE_THRESHOLD_H 70
#define X_LOCALIZE_THRESHOLD_L 0   
#define X_LOCALIZE_THRESHOLD_H 320

#define GOAL_LOCALIZATION_C1 3335.0


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
    uint16_t dist = 255; uint16_t angle = 255;
    uint16_t possession = 255; bool valid = false;
    float Vx; float Vy;
};

struct USSensor {
    uint16_t dist_b = 0; uint16_t dist_l = 0;
    uint16_t dist_r = 0; uint16_t dist_f = 0;
};

struct RobotMovement {
    float vx = 0.0f; float vy = 0.0f; float rot_v = 0.0f;
    uint16_t heading = 0; // Target heading in degrees
};


struct SubCoreData {
    uint32_t lineState = 0x0000; // 16 sensors, 1 bit each
    int16_t gyroHeading = 0; // 0-359 degrees
};



// --- 3. External Variables ---
// These tell the compiler "The actual memory for these is in main.cpp"
extern CamData camData;
extern BallData ballData;
extern USSensor usData;
extern SubCoreData subCoreData;
extern RobotMovement robotMovement;
extern Adafruit_SSD1306 display;


// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void readBallCam();
void readFrontCam();
void readussensor();
void localization();
void kicker_control(bool kick);
bool UI_Interface();
void readGyroAndLineFromSubCore();
void sendMotor(float vx, float vy, float rot_v, int heading);
void sendMotorAndGetSensors(float vx, float vy, float rot_v, int heading);
void localizeRobot();
bool move_to_position(int pos_x, int pos_y);
void send_cam_and_pos_data();
void update_all_sensor();
void defense_localizeRobot();
void sendPacket();
#endif