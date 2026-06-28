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
            Serial.printf("Received handshake: 0x%02X\n", temp);
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
    if(cnt > 0){
        float move_deg = atan2(y_sum, x_sum) * 57.2958f;
        if (move_deg < 0) move_deg += 360.0f;
        return move_deg;
    }
    return -1; // No line detected
}
void defense_mode() {
    update_gyro_sensor();
    update_line_sensor();
    readMaincoreData();

    int count = 0;
    for (uint8_t i = 0; i < 32; i++) {
        if (bitRead(lineData.state, i) == 0) count++;
    }

    float vx = 0, vy = 0;

    Serial.printf("Ball valid:%d angle:%d dist:%d | Line count:%d\n",subMonitor.ball_valid, subMonitor.ball_angle, subMonitor.ball_dist, count);

    if (count <= 3) {
        // ── No line: return to home (0, -90) ────────────────────────────
        float dx = 0.0f   - subMonitor.pos_x;
        float dy = -90.0f - subMonitor.pos_y;
        vx = constrain(dx * 0.15f, -40.0f, 40.0f);
        vy = constrain(dy * 0.40f, -40.0f, 40.0f);
        if (fabsf(vx) < 8.0f) vx = 0.0f;
        if (vy != 0.0f && fabsf(vy) < 12.0f) vy = (vy > 0.0f) ? 12.0f : -12.0f;
    } 
    else {
        // ── On line ──────────────────────────────────────────────────────
        // Sensor layout: 32 sensors, 0=right, 11.25° per sensor
        // right half: center=0,  range=13 → bits around 0°   (right)
        // left  half: center=16, range=13 → bits around 180° (left)
        float right_deg = get_line_move_deg(13, lineData.state, 0);
        float left_deg  = get_line_move_deg(13, lineData.state, 16);
        float top_deg = get_line_move_deg(13, lineData.state, 8);
        float down_deg  = get_line_move_deg(13, lineData.state, 24);
        Serial.printf("right_deg:%.1f left_deg:%.1f top_deg:%.1f down_deg:%.1f\n", right_deg, left_deg, top_deg, down_deg);
        // ── Ball side selection ──────────────────────────────────────────
        bool ball_on_right = subMonitor.ball_valid &&(subMonitor.ball_angle > 270 || subMonitor.ball_angle < 80);
        bool ball_on_left  = subMonitor.ball_valid &&(subMonitor.ball_angle > 100 && subMonitor.ball_angle < 270);

        float move_deg = -1.0f;
        if      (ball_on_right && right_deg != -1.0f) move_deg = right_deg;
        else if (ball_on_left  && left_deg  != -1.0f) move_deg = left_deg;

        // ── Symmetric corner lock ────────────────────────────────────────
        // Left  corner: left_deg  ∈ (180,270) AND right_deg ∈ ( 270,90) → block vx<0, vy<0
        // Right corner: right_deg ∈ (270,360) AND left_deg  ∈ ( 90,180) → block vx>0, vy<0
        /*
        bool lock_left  = (left_deg  != -1.0f && left_deg  > 225.0f && left_deg  < 270.0f) &&
                          (right_deg != -1.0f && right_deg <  90.0f  && right_deg > 0.0f);
        bool lock_right = (right_deg != -1.0f && right_deg > 315.0f && right_deg < 360.0f) &&
                          (left_deg  != -1.0f && left_deg  >  90.0f && left_deg  < 180.0f);
        */
        bool lock_left  = (down_deg != -1.0f && down_deg < 315.0f  && down_deg > 225.0f) && (top_deg  != -1.0f && top_deg  < 45.0f  && top_deg  >  0.0f);
        bool lock_right = (down_deg != -1.0f && down_deg < 315.0f  && down_deg > 225.0f) && (top_deg  != -1.0f && top_deg  >  135.0f && top_deg  < 180.0f);
        // ── Compute velocity ─────────────────────────────────────────────
        if (move_deg != -1.0f) {
          float rad = move_deg * DtoR_const;
          vx = 80.0f * cosf(rad);
          vy = 30.0f * sinf(rad);
        } 
        else if (left_deg != -1.0f || right_deg != -1.0f) {
            // No ball / unclear → vector sum of both to stay on line
            float svx = 0.0f, svy = 0.0f;
            if (left_deg  != -1.0f) { float r = left_deg  * DtoR_const; svx += cosf(r); svy += sinf(r); }
            if (right_deg != -1.0f) { float r = right_deg * DtoR_const; svx += cosf(r); svy += sinf(r); }
            float mag = sqrtf(svx * svx + svy * svy);
            if (mag > 0.001f) { vx = 80.0f * svx / mag; vy = 80.0f * svy / mag; }
            // mag ≈ 0 → sensors perfectly cancel → robot centred → stay still
        }
        // Both -1 and no ball → vx=vy=0, hold position

        // ── Apply corner locks ───────────────────────────────────────────
        if (lock_left)  { if (vx < 0.0f) vx = 0.0f; if (vy < 0.0f) vy = 0.0f; }
        if (lock_right) { if (vx > 0.0f) vx = 0.0f; if (vy < 0.0f) vy = 0.0f; }

        Serial.printf("right_deg:%.1f left_deg:%.1f move_deg:%.1f | lock_L:%d lock_R:%d\n", right_deg, left_deg, move_deg, lock_left, lock_right);
    }

    float res_deg = atan2f(vy, vx) * 57.2958f;
    if (res_deg < 0.0f) res_deg += 360.0f;
    Serial.printf("Output → deg:%.1f vx:%.1f vy:%.1f\n", res_deg, vx, vy);


    //Missing vector sum to stay on line when no ball detected
    // TODO: pass vx, vy to motor driver
    Vector_Motion(vx, vy, 0);
}

void defense_mode2(){
    update_gyro_sensor();
    update_line_sensor();
    readMaincoreData();

    int count = 0;
    float x_sum = 0, y_sum = 0;
    const float deg2rad = M_PI / 180.0f;
    float vx = 0, vy = 0;
    for (int i = 0 ; i < 32; i++) {
        if (!((lineData.state >> i) & 1)) {
            float rad = (i * 11.25f) * deg2rad;
            x_sum += cosf(rad);
            y_sum += sinf(rad);
            count++;
        }
    }

    Serial.printf("Ball valid:%d angle:%d dist:%d | Line count:%d\n",subMonitor.ball_valid, subMonitor.ball_angle, subMonitor.ball_dist, count);

    if (count <= 3) {
      //back to panelty area
      ;
    }
    else{
      float left_deg = get_line_move_deg(9, lineData.state, 16);
      float right_deg = get_line_move_deg(9, lineData.state, 0);
      float move_vx = 0;
      if(subMonitor.ball_valid){
        if(subMonitor.ball_angle > 270 || subMonitor.ball_angle < 80){
          move_vx = 60;
        }
        else if (subMonitor.ball_angle > 100 && subMonitor.ball_angle < 270){
          move_vx = -60;
        }
      }
      else{
        ;// back to standby position
      
      }
      float mag = 0;
      float line_deg = atan2(y_sum, x_sum) * 57.2958f;
      //static float decelerate = 1.0;
      if (line_deg < 0) line_deg += 360.0f;
      mag = sqrt(y_sum * y_sum + x_sum * x_sum);//index of checking how far the robot have shifted from the line
      float line_rad = line_deg * DtoR_const;
      if(abs(x_sum) > 1.0f && vy < 0.0f){
        move_vx *= 0.5;
      }
      if(abs(y_sum) < 1.75f){
        vx = move_vx;
        vy = 40.0f * (abs(y_sum) / 1.75f) * sinf(line_rad);
        //vy = 0;
      }
      else{
        vx = 60.0f * cosf(line_rad);
        vy = 60.0f * sinf(line_rad);
      }
      if(abs(x_sum) > 1.0f && vy < 0.0f){
        vy = vy < 0 ? 0 : vy;
        Serial.println("Corner lock activated");
      }
      if(abs(270 - left_deg) < 22.5 && (right_deg < 45 || abs(right_deg - 360) < 45)){
        vy = vy < 0 ? 0 : vy;
        vx = vx < 0 ? 0 : vx * 3;
        Serial.println("left lock activated");
      }
      if(abs(270 - right_deg) < 22.5 && abs(left_deg - 180) < 45){
        vy = vy < 0 ? 0 : vy;
        vx = vx > 0 ? 0 : vx * 3;
        Serial.println("right lock activated");
      }
      //Serial.printf("line_deg:%.1f, mag:%f, move_vx:%f, vx: %f, vy %f, x_sum:%f, y_sum:%f \n", line_deg, mag, move_vx, vx, vy, x_sum, y_sum);
      Serial.printf("left:%f, right:%f, line_deg:%f, mag:%f  \n", left_deg, right_deg, line_deg, mag);
    
    }
    Serial.printf("vx: %f, vy %f\n", vx, vy);
    Vector_Motion(vx, vy, 0);
}
void defense_mode3(){
  update_gyro_sensor();
  update_line_sensor();
  readMaincoreData();

  int count = 0;
  float x_sum = 0, y_sum = 0;
  const float deg2rad = M_PI / 180.0f;
  float vx = 0, vy = 0;
  for (int i = 0 ; i < 32; i++) {
      if (!((lineData.state >> i) & 1)) {
          float rad = (i * 11.25f) * deg2rad;
          x_sum += cosf(rad);
          y_sum += sinf(rad);
          count++;
      }
  }
  Serial.printf("Ball valid:%d angle:%d dist:%d | Line count:%d\n",subMonitor.ball_valid, subMonitor.ball_angle, subMonitor.ball_dist, count);

  static int avoid_timer = 0;
  static float avoid_vx = 0.0f, avoid_vy = 0.0f;

  if (count <= 3) {
    Serial.printf("subMonitor.pos_x %f subMonitor.pos_y %f\n", subMonitor.pos_x, subMonitor.pos_y);
    float dx = 0.0f   - subMonitor.pos_x;
    float dy = -90.0f - subMonitor.pos_y;
    vx = constrain(dx * 0.15f, -40.0f, 40.0f);
    vy = constrain(dy * 0.40f, -40.0f, 40.0f);
    if (fabsf(vx) < 8.0f) vx = 0.0f;
    if (vy != 0.0f && fabsf(vy) < 20.0f) vy = (vy > 0.0f) ? 20.0f : -20.0f;
    

    if (subMonitor.ball_valid && subMonitor.ball_dist > 0) {
      float pdx = dx;
      float pdy = dy;
      float pathLen = sqrtf(pdx * pdx + pdy * pdy);
      if (pathLen > 5.0f) {
        float ballRad = (gyroData.heading + subMonitor.ball_angle) * M_PI / 180.0f;
        float bx = subMonitor.pos_x + subMonitor.ball_dist * cosf(ballRad);
        float by = subMonitor.pos_y + subMonitor.ball_dist * sinf(ballRad);
        float relx = bx - subMonitor.pos_x;
        float rely = by - subMonitor.pos_y;
        float proj = (relx * pdx + rely * pdy) / pathLen;
        float perp_x = relx - proj * (pdx / pathLen);
        float perp_y = rely - proj * (pdy / pathLen);
        float perpDist = sqrtf(perp_x * perp_x + perp_y * perp_y);
        const float SAFETY_DIST = 30.0f;
        if (proj > 0.0f && proj < pathLen && perpDist < SAFETY_DIST) {
          float avoid_deg = gyroData.heading + subMonitor.ball_angle - 30.0f;
          float rad = avoid_deg * M_PI / 180.0f;
          avoid_vx = 40.0f * cosf(rad);
          avoid_vy = 40.0f * sinf(rad);
          avoid_timer = 25;
        }
      }
    }

    if (avoid_timer > 0) {
      vx = avoid_vx;
      vy = avoid_vy;
      avoid_timer--;
    }
  }
  else{
    float right_deg = get_line_move_deg(11, lineData.state, 0);
    float top_deg = get_line_move_deg(5, lineData.state, 8);
    float left_deg = get_line_move_deg(11, lineData.state, 16);
    float down_deg = get_line_move_deg(5, lineData.state, 24);
    bool ball_on_right = (subMonitor.ball_valid) && (subMonitor.ball_angle > 270 || subMonitor.ball_angle < 80);
    bool ball_on_left = (subMonitor.ball_valid) && (subMonitor.ball_angle > 100 && subMonitor.ball_angle < 270);
    
    float move_deg = -1;
    float lock_deg = atan2(y_sum, x_sum) * 57.2958f;
    if (lock_deg < 0) lock_deg += 360.0f;
    float mag = sqrt(y_sum * y_sum + x_sum * x_sum);//index of checking how far the robot have shifted from the line
    if(ball_on_right){
      move_deg = right_deg;
    }
    else if (ball_on_left){
      move_deg = left_deg;   
    }
    if(move_deg != -1){
      float move_rad = move_deg * DtoR_const;
      vx = 40.0f * cosf(move_rad);
      vy = 40.0f * sinf(move_rad);
    }
    else if(lock_deg != -1){
      float lock_rad = lock_deg * DtoR_const;;
      vx = mag * 15 * cosf(lock_rad);
      vy = mag * 15 * sinf(lock_rad);
    }
    if(top_deg != -1 && down_deg != -1 && abs(down_deg - 270) < 45){
      Serial.println("side");
      float escape_deg = get_line_move_deg(9, lineData.state, 8);
      Serial.printf("escape%f\n", escape_deg);
      bool robot_on_right = (escape_deg > 90 && escape_deg < 180);
      bool robot_on_left = (escape_deg < 90 && escape_deg > 0);
      move_deg = -1;
      if(ball_on_right){
        Serial.println("ball right lock");
        if(robot_on_left){
          move_deg = escape_deg;
        }
      }
      else if (ball_on_left){
        Serial.println("ball left2 lock");
        if(robot_on_right){
          move_deg = escape_deg;
        }
      }
      if(move_deg != -1){
        float move_rad = move_deg * DtoR_const;
        vx = 40.0f * cosf(move_rad);
        vy = 40.0f * sinf(move_rad);
      }
      else{
        vx = 0;
        if(vy < 0){
          vy = 0;
        }
      }
    }
    Serial.printf("move_deg: %f, lock_deg: %f left_deg: %f, right_deg: %f, mag%f\n", move_deg, lock_deg, left_deg, right_deg, mag);
  }
  
  float res_deg = atan2f(vy, vx) * 57.2958f;
  if (res_deg < 0.0f) res_deg += 360.0f;
  Serial.print("Combined vector deg: ");
  Serial.print(res_deg);
  Serial.print(" | vx=");
  Serial.print(vx);
  Serial.print(", vy=");
  Serial.print(vy);
  Serial.print(" | gyro_heading=");
  Serial.println(gyroData.heading);
  Serial.print(" | pos_x=");
  Serial.println(subMonitor.pos_x);
  Serial.print(" | pos_y=");
  Serial.println(subMonitor.pos_y);
  
  Vector_Motion(vx, vy, 0);
}
void loop(){
  
  while(1){
    update_gyro_sensor();
    update_line_sensor();
    //Serial.printf("Gyro Heading: %f\n", gyroData.heading);
    //for(uint8_t i = 0; i < 32; i++){
      //Serial.printf("%d", (lineData.state >> i) & 1);
    //}
    //Serial.println();
    if (Serial8.available()) {  
      uint8_t cmd = Serial8.read();
      Serial.println(cmd,HEX);
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
    defense_mode3();
    //SetMotorSpeed(1,50);
  }
}   