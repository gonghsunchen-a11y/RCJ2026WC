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
// Ultrasonic pins
#define TRIG_F 2  
#define ECHO_F 6
#define TRIG_R 3  
#define ECHO_R 8
#define TRIG_B 4  
#define ECHO_B 9
#define TRIG_L 5  
#define ECHO_L 10

#define US_COUNT         4
#define US_INVALID_DIST  999.0f
#define US_FILTER_ALPHA  0.35f
#define US_INVALID_LIMIT 3

#define alpha 0.15

#define COM_O1_PIN 37
#define COM_O2_PIN 36

// --- Kicker Constants ---
#define Charge_Pin 33
#define Kicker_Pin 32


struct TopMaixPosData {
    int16_t x = 0;
    int16_t y = 0;
    uint8_t status = 0;
    bool valid = false;
    bool ball_found = false;
    uint16_t ball_angle = 0xFFFF;
    uint8_t ball_dist = 0;
};

// main_core.h
typedef struct {
  float    pos_x_f;
  float    pos_y_f;
  float    dist_cm[US_COUNT];
  uint8_t  invalid_count[US_COUNT];
  float    coord_x;
  float    coord_y;
  uint8_t  current;
  uint32_t last_trigger_time;
  uint32_t last_display_time;
} USData;



enum class RobotState : uint8_t { STATE_READY, STATE_CALIBRATING, STATE_SAVING };
// 2. Robot state structure
struct RobotMonitor {
    RobotState currentState = RobotState::STATE_READY;
    int8_t pos_x = 0;
    int8_t pos_y = 0;
    float vx = 0.0f; 
    float vy = 0.0f; 
    float rot_v = 0.0f;
    uint16_t heading = 0; // Target heading in degrees
    uint16_t ball_dist = 0; // 0-15 (4 bits)
    uint16_t ball_angle = 0; // 0-255 (8 bits)
    bool ball_valid = false;
    bool has_possession = false;
    bool strategy = false; // 0 for defense, 1 for offense
    uint8_t role = 0; // Uses the custom enum type with a default value
};

// --- 3. External Variables ---
// These tell the compiler "The actual memory for these is in main.cpp"
extern USData usData;
extern RobotMonitor robotMonitor;
extern RobotMonitor teammateMonitor;
extern Adafruit_SSD1306 display;
extern TopMaixPosData topmaixPosData;
extern USData usData;

// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void kicker_control(bool kick);
void readTopMaix();
void roleSwitchControl();
void get_ball_sensor();
void parseSerial6Stream();
void listenToEspCommands();
void sendRoleSwitchCommand();
void getRoleSwitchCommand();
void update_all_sensor();
void sendMotor(float vx, float vy, float rot_v, int heading);
void stopMotors();
bool UI_Interface();
void triggerUS(uint8_t i);
void updateUS();
void ballsensor();
void sendMaincoreData();
void readTopMaix();
void sensor_fusion();
void updateUS();
void triggerUS(uint8_t i);
void updateReading(uint8_t i, uint32_t duration);
void updateUSCoordinate();


enum USIndex   { US_FRONT = 0, US_RIGHT = 1, US_BACK = 2, US_LEFT = 3 };

// main_core.h
extern volatile uint32_t echo_start[US_COUNT];
extern volatile uint32_t echo_duration[US_COUNT];
extern volatile bool     echo_done[US_COUNT];
const uint8_t trigPins[US_COUNT] = { TRIG_F, TRIG_R, TRIG_B, TRIG_L };
const uint8_t echoPins[US_COUNT] = { ECHO_F, ECHO_R, ECHO_B, ECHO_L };
inline bool isValidUS(float d) { return d < US_INVALID_DIST; }

// 用 template 取代 4 個幾乎一樣的 ISR wrapper
template<uint8_t i>
void echoISR() {
  if (digitalRead(echoPins[i]) == HIGH) {
    echo_start[i] = micros();
  } else {
    echo_duration[i] = micros() - echo_start[i];
    echo_done[i] = true;
  }
}

#endif