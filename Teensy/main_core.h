#ifndef MAIN_CORE_H
#define MAIN_CORE_H

struct BallData{uint16_t angle = 255; uint16_t possession = 255; bool valid = false; float Vx; float Vy;} ballData;
struct TopMaixData {int16_t x = 0;int16_t y = 0;uint8_t status = 0;bool valid = false;bool ball_found = false;uint16_t ball_angle = 0xFFFF;uint8_t ball_dist = 0;};

extern BallData ballData;
extern TopMaixData topmaixData;

// MATH Constants
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f

// Buttons
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26
int _page = 0;      // 0: 主選單, 1: 掃描頁面
int _cursor = 0;    // 選單游標位置
unsigned long _lastPress = 0; 
unsigned long _lastUpdate = 0;

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Communication Module
#define COM_Pin1 37
#define COM_Pin2 36
// Kicker
#define Charge_Pin 33 //FET1
#define Kicker_Pin 32 //FET2

#endif
