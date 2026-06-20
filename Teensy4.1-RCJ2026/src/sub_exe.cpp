#include "sub_core.h"
#include "sub_core.h"

#define DtoR_const 0.0174529f

uint8_t role = 0; // 0: default, 1: defense, 2: offense
void setup() {
    sub_core_init();
    digitalWrite(LED_BUILTIN, HIGH); // Indicate setup incomplete
    while(1){
        Serial.println("Waiting for MainCore...");
        if(Serial8.available()){
            uint8_t temp = Serial8.read();
            if(temp == 0x0A){
                role = 1; // offense
                Serial8.write(PROTOCAL_ACT);
                break;
            }
            else if(temp == 0x0D){
                role = 2; // defense
                Serial8.write(PROTOCAL_ACT);
                break;
            }
            else{
                Serial.println("Invalid handshake, retrying...");
            }
        }
        else{
            Serial.println("Invalid handshake, retrying...");
        }
    }
    digitalWrite(LED_BUILTIN, LOW); // Indicate setup complete
}
float get_line_move_deg(int8_t range, uint32_t line_state, uint8_t center) {
    float x_sum = 0, y_sum = 0;
    int cnt = 0;
    const float deg2rad = M_PI / 180.0f;
    for (int i = -range ; i <= range; i++) {
      uint8_t idx = (center + i + 32) % 32;

        if (!((lineData.state >> idx) & 1)) {
            float deg = idx * 11.25f;
            float rad = deg * deg2rad;
            x_sum += cosf(rad);
            y_sum += sinf(rad);
            cnt++;
        }
    }
    if (cnt > 0) {
        float move_deg = atan2(y_sum, x_sum) * 57.2958f;
        if (move_deg < 0) move_deg += 360.0f;
        return move_deg;
    }
    return -1; // No line detected
}
int count = 0;
void defense_mode() {
    update_gyro_sensor();
    update_line_sensor();
    readMaincoreData();
    /*
    for(uint8_t i = 0; i < 32; i++){
      Serial.printf("%d", (lineData.state >> i) & 1);
    }
    Serial.println("");
    */
    float right_deg = -1;
    float left_deg = -1;
    float right_vx = 0, right_vy = 0;
    float left_vx = 0, left_vy = 0;
    int count = 0;
    bool stopr = false;
    bool stopl = false;
    float move_deg = -1;
    bool side = false; // false: right, true: left
    Serial.printf("Ball valid: %d, angle: %d, dist: %d\n", subMonitor.ball_valid, subMonitor.ball_angle, subMonitor.ball_dist);
    for(uint8_t i = 0; i < 32; i++) {
      if (bitRead(lineData.state, i) == 0) {
        count++;
      }
    }
    if(count > 3){
      if(subMonitor.ball_valid && (subMonitor.ball_angle > 270 || subMonitor.ball_angle < 85)){
        move_deg = get_line_move_deg(7, lineData.state, 0);
      }
      else if(subMonitor.ball_valid && (subMonitor.ball_angle > 95 && subMonitor.ball_angle < 270)){
        move_deg = get_line_move_deg(7, lineData.state, 16);
      }
      else{
        move_deg = -1;
      }
    }
    if(move_deg < 315&& move_deg > 270&&move_deg != -1){
      stopr = true;
    }
    if(move_deg < 270 && move_deg > 225&&move_deg != -1){
      stopl = true;
    }
    float left_lock_deg = get_line_move_deg(7, lineData.state, 16);
    float right_lock_deg = get_line_move_deg(7, lineData.state, 0);
    float lock_deg = -1;
    float lockvx = 0, lockvy = 0;
    if(left_lock_deg != -1 || right_lock_deg != -1){
        if (left_lock_deg != -1) {
          float rad = left_lock_deg * M_PI / 180.0f;
          lockvx += cosf(rad);
          lockvy += sinf(rad);
        }
        if (right_lock_deg != -1) {
          float rad = right_lock_deg * M_PI / 180.0f;
          lockvx += cosf(rad);
          lockvy += sinf(rad);
        }
        lockvx=lockvx*40;
        lockvy=lockvy*40;
        lock_deg = atan2f(lockvy, lockvx) * 57.2958f;
        if (lock_deg < 0) lock_deg += 360.0f;
    }

    // 顯示左右角度和向量
    Serial.print("Move deg: ");
    Serial.print(move_deg);
    float vx = 0, vy = 0;
    if(move_deg!=-1){
      float temp_rad = move_deg * DtoR_const;
      vx = 40 * cosf(temp_rad);
      vy = 40 * sinf(temp_rad);
    }
    vx=vx*1.5+lockvx;
    vy=vy*0.5+lockvy*0.5;
    if(stopr&&vx > 0){
      vy = 0;
    }
    if(stopl&&vx < 0){
      vy = 0;
    }

    // 計算左右向量的和

    float res_deg = atan2(vy, vx) * 57.2958f;
    if (res_deg < 0) res_deg += 360.0f;
    Serial.print("Combined vector deg: ");
    Serial.print(res_deg);
    Serial.print(" | vx=");
    Serial.print(vx);
    Serial.print(", vy=");
    Serial.println(vy);

    Vector_Motion(vx, vy, 0);
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
      if(cmd == LS_CAL_START) {
        Serial.println("Starting line sensor calibration...");
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
          }
          for (int i = 0; i < LS_count; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          for (int i = 0; i < LS_count; i++) {
            Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
          }
          avg_ls[32] = (front_max + front_min) / 2;
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
  //digitalWrite(LED_BUILTIN, HIGH); // Toggle LED for visual feedback
  while(1){
    defense_mode();
  }
}   