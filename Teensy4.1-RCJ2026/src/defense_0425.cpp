#include <sub_core.h>
uint8_t op_mode;

float lineVx = 0;
float lineVy = 0;


// ─── In sub_exe.cpp ───────────────────────────────────────────────────────────
// ── Zone masks (add these to sub_core.h or top of sub_exe.cpp) ──────────────
#define LS_MASK_FRONT  0x00000FE0UL
#define LS_MASK_RIGHT  0x001FF000UL
#define LS_MASK_BACK   0x0FE00000UL
#define LS_MASK_LEFT   0xF000001FUL

#define DEFENSE_SPEED     40.0f
#define DEFENSE_Y_SPEED   30.0f
#define DEFENSE_Y_RAMP    0.08f   // speed added per ms

// ── Helper: check if any sensor in a mask zone is triggered ─────────────────
// lineData.state: bit = 0 means ON LINE, 1 means clear
static inline bool zone_triggered(uint32_t mask) {
    return (lineData.state & mask) != mask; // any bit in mask is 0 → triggered
}


void defense_mode() {
    float vx = 0.0f;
    float vy = 0.0f;

    bool on_front = zone_triggered(LS_MASK_FRONT);
    bool on_back  = zone_triggered(LS_MASK_BACK);
    bool on_left  = zone_triggered(LS_MASK_LEFT);
    bool on_right = zone_triggered(LS_MASK_RIGHT);

    // ── Y-axis: stay ON the line ─────────────────────────────────────────────
    if (on_front) vy =  DEFENSE_SPEED;   // front touched → push forward
    if (on_back)  vy = -DEFENSE_SPEED;   // back touched  → push backward

    // ── Sideways: escape if going out of bounds ──────────────────────────────

    // ── Ball tracking on X-axis (only when not hitting side lines) ───────────
    if (on_left && on_right && ballData.valid) {
        int angle = ballData.angle;

        bool ball_left  = (angle > 105 && angle < 270);
        bool ball_right = (angle <  85 || angle > 270);

        if (ball_left) {
            float t = 1.0f - (float)abs(180 - angle) / 90.0f;
            vx = -DEFENSE_SPEED * t;
        } else if (ball_right) {
            float dist = (angle <= 85) ? angle : (360 - angle);
            float t = 1.0f - dist / 90.0f;
            vx = DEFENSE_SPEED * t;
        }
        else{
            vx = 0;
        }
    }
    FC_Vector_Motion(vx, vy, 90);
}

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while (1) {
        read_cam_and_pos_data();
        fast_update_line_sensor();
        update_gyro_sensor();

        defense_mode();
    }
}

void setup(){
  sub_core_init();
  while(1){
  Serial.println("Waiting for MainCore...");
    if(Serial8.available()){
      op_mode = Serial8.read();
      Serial.printf("Received mode: 0x%X\n", op_mode);
      if(op_mode == T_MODE_HEADER || op_mode == C_MODE_HEADER){
        Serial8.write(PROTOCAL_ACT); // Acknowledge receipt
        break;
      }
    }
  }
}

void loop(){
  while(1){
    update_gyro_sensor();
    update_line_sensor();
    Serial.printf("Gyro Heading: %f\n", gyroData.heading);
    for(uint8_t i = 0; i < 32; i++){
      Serial.printf("%d", (lineData.state >> i) & 1);
    }
    Serial.println();
    if (Serial8.available()) {
      uint8_t cmd = Serial8.read();
      //Serial.print(cmd);
      if (cmd == LS_CAL_START) {
          uint16_t max_ls[32], min_ls[32];
          uint16_t front_max = 0, front_min = 4095;
          uint16_t mid_max = 0, mid_min = 4095;
          for (int i = 0; i < 32; i++) { 
            max_ls[i] = 0; 
            min_ls[i] = 4095; 
          }
          int timer = 0;
          while (1) {
            if(micros() - timer > 100000) { // 每 100ms 更新一次
              digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED for visual feedback
              timer = micros();
            }
            if(Serial8.available()){
              uint8_t cmd = Serial8.read();
              if(cmd == LS_CAL_END){ // End calibration command
                break;
              }
            }
            for (int i = 0; i < LS_count; i++) {
              int r = readMux(i % 16, (i < 16) ? 1 : 2);
              if (r > max_ls[i]) max_ls[i] = r; 
              if (r < min_ls[i]) min_ls[i] = r;
            }
            uint16_t reading = analogRead(Front_LS);
            if(reading > front_max) front_max = reading;
            if(reading < front_min) front_min = reading;
            reading = analogRead(Mid_LS);
            if(reading > mid_max) mid_max = reading;
            if(reading < mid_min) mid_min = reading;

          }

          for (int i = 0; i < LS_count; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          for (int i = 0; i < LS_count; i++) {
            Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
          }
          avg_ls[32] = (front_max + front_min) / 2;
          avg_ls[33] = (mid_max + mid_min) / 2;
          EEPROM.put(0, avg_ls);
          delay(1000); // Ensure EEPROM write completes
          Serial8.write(LS_CAL_ACK); // Send end calibration acknowledgment
        } 
      else if (cmd == MOVE_CMD) {// When BTN_UP is pressed, send a move command to the main core
        Serial8.write(PROTOCAL_ACT);
        break;
      }
    }
  }
  digitalWrite(LED_BUILTIN, HIGH); // Toggle LED for visual feedback
  if(op_mode == C_MODE_HEADER){
    c_mode_main_function();
  }
  else if(op_mode == T_MODE_HEADER){
    //t_mode_main_function();
    ;
  }
}
