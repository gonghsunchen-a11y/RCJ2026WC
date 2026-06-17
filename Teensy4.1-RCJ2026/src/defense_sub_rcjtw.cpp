#include <sub_core.h>
uint8_t op_mode;

float lineVx = 0;
float lineVy = 0;

#define DtoR_const 0.0174529f
#define SPD   35
#define SPD7  20  // SPD * 0.707

#define LOCK_ANGLE 45

#include <math.h>


/**
 * 使用向量求和計算法
 * @param line_state  32-bit 數據 (0代表壓線)
 * @param center      中心點 (0-31)
 * @param range       掃描半徑
 * @return float      絕對角度 (0~360)，沒壓線返回 -1.0
 */
float get_line_move_angle(uint32_t line_state, uint8_t center, int range) {
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    int count = 0;
    const float deg2rad = M_PI / 180.0f; // 角度轉弧度常數

    for (int i = -range; i <= range; i++) {
        uint8_t idx = (center + i + 32) % 32;

        // 檢查該位元是否為 0 (壓線)
        if (!((line_state >> idx) & 1)) {
            // 計算該感測器的絕對物理角度
            float angle_deg = idx * 11.25f;
            float angle_rad = angle_deg * deg2rad;

            // 向量累加
            x_sum += cosf(angle_rad);
            y_sum += sinf(angle_rad);
            count++;
        }
    }

    if (count > 0) {
        // 使用 atan2 算出弧度，並轉回角度
        float result_rad = atan2f(y_sum, x_sum);
        float result_deg = result_rad * (180.0f / M_PI);

        // 將結果標準化到 0~360 度
        if (result_deg < 0) result_deg += 360.0f;
        
        return result_deg;
    }

    return -1.0f;
}

// 計算兩個角度之間的最短夾角
float get_angle_diff(float a, float b) {
    float diff = fabs(a - b);
    if (diff > 180) diff = 360 - diff;
    return diff;
}

void defense_mode() {
    readMainPacket(); //要讀取超聲波　還有相機的參數
    update_line_sensor(); // Keep updating sensors!
    update_gyro_sensor();
    float back_vx = 0;
    float back_vy = 0;
    //看到球門，但超聲距離太遠或太近，要前進
    if(goalData.valid && (usData.dist_b > 40 || usData.dist_b < 30)){    
        //球門限制
        //右側 220
        //左側 100
        back_vx = 0;
        if(goalData.x > 200){
            back_vx = -30;
        }
        else if(goalData.x < 120){
            back_vx = 30;
        }
        back_vy = (35 - usData.dist_b) * 3;
        if(back_vy > 40) back_vy = 20;
        if(back_vy < -40) back_vy = -20;
        if(back_vy < 15 && back_vy > 0) back_vy = 15;
        if(back_vy > -15 && back_vy < 0) back_vy = -15;
    }
    if(!goalData.valid){//在角落 看不到相機，移動到50CM 處 看到相機
        back_vx = 0;
        back_vy = (50 - usData.dist_b) * 3;
        //最大速度和最小速度調整
        if(back_vy > 25) back_vy = 25;
        if(back_vy < -25) back_vy = -25;
        if(back_vy < 15 && back_vy > 0) back_vy = 15;
        if(back_vy > -15 && back_vy < 0) back_vy = -15;
    }
    Serial.printf("back_us %d back_vx %f, back_vy %f\n", usData.dist_b, back_vx, back_vy);
    if(back_vy || back_vx){
        Serial.printf("OUT\n");
        FC_Vector_Motion(back_vx, back_vy, 90);
    }
    float move_deg = -1;
    bool side = false;
    int count = 0;
    //計算有幾顆碰到線，避免誤觸
    for(int i = 0; i < LS_count; i++){
        if(bitRead(lineData.state, i) == 0){
            count++;
        }
    }
    if(count > 3 /*&&goalData.valid*/){//可以添加goalData.valid來確認是禁區的線
        if(ballData.valid && (ballData.angle > 270 || ballData.angle < 80)){
            move_deg = get_line_move_angle(lineData.state, 0, 7);
            //球在右邊以右半邊光感，以0號為中心循線
            //如果右側看到315代表在弧線區 要停止
            if(move_deg < 315 && move_deg > 270){
                side = true;
            }
        }
        else if(ballData.valid && (ballData.angle > 100 && ballData.angle < 270)){
            move_deg = get_line_move_angle(lineData.state, 16, 7);
            //球在左邊以左半邊光感，以16號為中心循線
            //如果左側看到225代表在弧線區 要停止(SIDE == TURE) 會鎖住VY<0
            if(move_deg > 225 && move_deg < 270){
                side = true;
            }
        }
        else{
            //如果不移動，需要透過計算左半邊的線角度和右半邊線角度，來鎖死在線上
            move_deg = -1;
            float left_lock_angle = get_line_move_angle(lineData.state, 16, 7);
            float right_lock_angle = get_line_move_angle(lineData.state, 0, 7);

            // 只有當至少有一個角度有效時才計算合向量
            if (left_lock_angle != -1 || right_lock_angle != -1) {
                float vx = 0, vy = 0;

                // 處理左側向量
                if (left_lock_angle != -1) {
                    float rad = left_lock_angle * (M_PI / 180.0f);
                    vx += cosf(rad);
                    vy += sinf(rad);
                }

                // 處理右側向量
                if (right_lock_angle != -1) {
                    float rad = right_lock_angle * (M_PI / 180.0f);
                    vx += cosf(rad);
                    vy += sinf(rad);
                }

                // 兩者的和向量轉回角度
                float res_rad = atan2f(vy, vx);
                move_deg = res_rad * (180.0f / M_PI);
                if (left_lock_angle != -1 && right_lock_angle != -1) {
                    float angle_dist = get_angle_diff(left_lock_angle, right_lock_angle);
                    
                    // 如果兩側感應到的線夾角大於 120 度 代表在線上 不要動 可以調整 角度越大越靈敏(應該)(各自偏離中心 60 度)
                    if (angle_dist > 120.0f) { 
                        move_deg = -1; // 衝突太大，視為無效指令
                    }
                }
            }
            if(get_angle_diff(left_lock_angle, 180) > LOCK_ANGLE || get_angle_diff(right_lock_angle, 0) > LOCK_ANGLE){
                side = true;
            }
        }
        Serial.printf("Deg %f\n", move_deg);
        float vx = 0;
        float vy = 0;
        if(move_deg != -1){
            float temp = move_deg * DtoR_const;
            vx = 40 * cos(temp);//防守循線vx速度
            vy = 40 * sin(temp);//防守循線vy速度
        }
        if(side && vy < 0){//鎖住VY < 0
            vy = 0;
        }
        FC_Vector_Motion(vx, vy, 90);
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
      if(cmd == LS_CAL_START) {
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
  digitalWrite(LED_BUILTIN, HIGH); // Toggle LED for visual feedback
  while(1){
    defense_mode();
  }
}