#include <Wire.h>
#include <Arduino.h>
#include <main_core.h>

// ---- 腳位設定 ----
#define TRIG_F 2
#define ECHO_F 6
#define TRIG_R 3
#define ECHO_R 8
#define TRIG_B 4
#define ECHO_B 9
#define TRIG_L 5
#define ECHO_L 10

//US Sensor
#define front_us A15
#define left_us A16
#define back_us A17
#define right_us A14
#define alpha 0.15
float pos_x_f = 0.0;
float pos_y_f = 0.0;

#define US_COUNT         4
#define US_INVALID_DIST  999.0f
#define US_FILTER_ALPHA  0.35f
#define US_INVALID_LIMIT 3

volatile uint32_t echo_start[US_COUNT]    = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool     echo_done[US_COUNT]     = {false};

float   us_dist_cm[US_COUNT]       = { US_INVALID_DIST, US_INVALID_DIST, US_INVALID_DIST, US_INVALID_DIST };
uint8_t us_invalid_count[US_COUNT] = {0};

float    coord_x_cm = US_INVALID_DIST;
float    coord_y_cm = US_INVALID_DIST;
uint8_t  current_us = US_COUNT - 1;
uint32_t last_trigger_time = 0;
uint32_t last_display_time = 0;

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

void triggerUS(uint8_t i) {
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
}

// 合併 updateFilteredUS + markInvalidUS
void updateReading(uint8_t i, uint32_t duration) {
  if (duration > 100 && duration < 15000) {
    float raw = duration * 0.0343f / 2.0f;
    us_dist_cm[i] = isValidUS(us_dist_cm[i])
                      ? us_dist_cm[i] * (1.0f - US_FILTER_ALPHA) + raw * US_FILTER_ALPHA
                      : raw;
    us_invalid_count[i] = 0;
  } else if (++us_invalid_count[i] >= US_INVALID_LIMIT) {
    us_dist_cm[i] = US_INVALID_DIST;
  }
}

void updateUSCoordinate() {
  coord_x_cm = (isValidUS(us_dist_cm[US_LEFT]) && isValidUS(us_dist_cm[US_RIGHT]))
                 ? (us_dist_cm[US_LEFT] - us_dist_cm[US_RIGHT]) / 2.0f : US_INVALID_DIST;

  coord_y_cm = (isValidUS(us_dist_cm[US_BACK]) && isValidUS(us_dist_cm[US_FRONT]))
                 ? (us_dist_cm[US_BACK] - us_dist_cm[US_FRONT]) / 2.0f : US_INVALID_DIST;
}

void updateUS() {
  if (millis() - last_trigger_time >= 50) {
    last_trigger_time = millis();
    current_us = (current_us + 1) % US_COUNT;
    echo_done[current_us] = false;
    triggerUS(current_us);
  }

  for (uint8_t i = 0; i < US_COUNT; i++) {
    if (!echo_done[i]) continue;

    noInterrupts();
    uint32_t duration = echo_duration[i];
    echo_done[i] = false;
    interrupts();

    updateReading(i, duration);
    updateUSCoordinate();
  }
}

// ---- 顯示相關 ----
void printUSValue(float d) {
  if (!isValidUS(d)) {
    display.print(" ---.-");
    return;
  }
  char buf[8];
  snprintf(buf, sizeof(buf), "%5.1f", d);
  display.print(buf);
}

void printCoordValue(float c) {
  if (isValidUS(c)) display.print((int)round(c));
  else              display.print("---");
}

void showUSDistances() {
  if (millis() - last_display_time < 100) return;
  last_display_time = millis();

  static const char* labels[US_COUNT] = { "Front:", "Right:", "Back: ", "Left: " };
  static const uint8_t ys[US_COUNT]    = { 30, 38, 46, 54 };

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 22);
  display.print("X:"); printCoordValue(coord_x_cm);
  display.print(" Y:"); printCoordValue(coord_y_cm);

  for (uint8_t i = 0; i < US_COUNT; i++) {
    display.setCursor(0, ys[i]);
    display.print(labels[i]);
    printUSValue(us_dist_cm[i]);
    display.print(" cm");
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();

  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  attachInterrupt(digitalPinToInterrupt(ECHO_F), echoISR<US_FRONT>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_R), echoISR<US_RIGHT>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_B), echoISR<US_BACK>,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), echoISR<US_LEFT>,  CHANGE);
}

void loop() {
  updateUS();
  showUSDistances();
}