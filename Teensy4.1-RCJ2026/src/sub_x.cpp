#include "sub_core.h"

#define DtoR_const 0.0174529f

//Line Sensor
#define EMERGENCY_THRESHOLD 90

int8_t mode = 0; // mode: 0 for defense, 1 for switching state, 2 for offense
float linesensorDegreelist[32] = {
      0.00, 11.25, 22.50, 33.75, 45.00, 56.25, 67.50, 78.75, 
      90.00, 101.25, 112.50, 123.75, 135.00, 146.25, 157.50, 168.75, 
      180.00, 191.25, 202.50, 213.75, 225.00, 236.25, 247.50, 258.75, 
      270.00, 281.25, 292.50, 303.75, 315.00, 326.25, 337.50, 348.75
  };

void setup() {
    sub_core_init();
    digitalWrite(LED_BUILTIN, HIGH); // Indicate setup incomplete
    while(1){
        Serial.println("Waiting for MainCore...");
        if(Serial8.available()){
            uint8_t temp = Serial8.read();
            if(temp == 0x0A){
                Serial8.write(PROTOCAL_ACT);
                break;
            }
            else if(temp == 0x0D){
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
      //Vector_Motion(0, 0, 90); // Stop the robot while waiting for commands
    }
    //digitalWrite(LED_BUILTIN, HIGH); // Toggle LED for visual feedback
}
float angle_diff(float a, float b) {
  float d = fabsf(a - b);
  if (d > 180.0f) d = 360.0f - d;
  return d;
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
void defense_mode3(){
  static uint32_t ball_in_front_time = 0;
  const float deg2rad = M_PI / 180.0f;
  float vx = 0, vy = 0;
  int count = 0;
  float x_sum = 0, y_sum = 0;
  for (int i = 0 ; i < 32; i++) {
      if (!((lineData.state >> i) & 1)) {
          float rad = (i * 11.25f) * deg2rad;
          x_sum += cosf(rad);
          y_sum += sinf(rad);
          count++;
      }
  }
  Serial.printf("Ball valid:%d angle:%d dist:%d | Line count:%d\n",subMonitor.ball_valid, subMonitor.ball_angle, subMonitor.ball_dist, count);
  if(count < 3) {
    //禁區附近的防守策略：當線感測器數量少於3個時，表示機器人可能接近禁區，應該返回到預設位置以避免進入禁區。
    Serial.println("No line sensors detected - returning to home position (0, -80)");
    float dx = 0.0f - subMonitor.pos_x;
    float dy = -80.0f - subMonitor.pos_y;
    if (subMonitor.pos_y > -90.0f && subMonitor.pos_y < -80.0f) {
      dy = 0.0f;
    }
      if (subMonitor.pos_x < 30.0f && subMonitor.pos_x > -30.0f) {
      dx = 0.0f;
    }     

    // Increase the response so the robot returns to home more promptly.
    float base_vx = dx * 0.30f;
    float base_vy = dy * 0.30f;

    if(base_vx !=0 && fabs(base_vx)  < 20.0f ) base_vx = (base_vx > 0) ? 20.0f : -20.0f; 
    //Ensure some minimum forward/backward speed for smooth return on Y
    if (base_vy != 0 && fabs(base_vy) < 20.0f) base_vy = (base_vy > 0) ? 20.0f : -20.0f;

    vx = constrain(base_vx, -80.0f, 80.0f);
    vy = constrain(base_vy, -80.0f, 80.0f);

    // Only dead-band near-zero noise; avoid forcing a minimum speed that makes the return feel sluggish.
    if (fabsf(vx) < 4.0f) vx = 0.0f;
    if (fabsf(vy) < 4.0f) vy = 0.0f;
  }
  else{
    Serial.println("Line detected");
    bool ball_in_front = (subMonitor.ball_valid) && (subMonitor.ball_angle > 80 && subMonitor.ball_angle < 100);
    if(ball_in_front){//ball in front motion
      Serial.printf("ball in front %d\n", millis() - ball_in_front_time);
      if(ball_in_front_time != 0){
        if(millis() - ball_in_front_time >= 5000){
          mode = 1;// switch to offense mode
        }
      }
      else{
        ball_in_front_time = millis();
      }
    }
    else{//defense 
      ball_in_front_time = 0;
      float right_deg = get_line_move_deg(11, lineData.state, 0);
      float top_deg = get_line_move_deg(5, lineData.state, 8);
      float left_deg = get_line_move_deg(11, lineData.state, 16);
      float down_deg = get_line_move_deg(5, lineData.state, 24);
      bool ball_on_right = (subMonitor.ball_valid) && (subMonitor.ball_angle > 270 || subMonitor.ball_angle < 80);
      bool ball_on_left = (subMonitor.ball_valid) && (subMonitor.ball_angle > 100 && subMonitor.ball_angle < 270);
      float lock_deg = atan2(y_sum, x_sum) * 57.2958f;
      if (lock_deg < 0) lock_deg += 360.0f;
      float mag = sqrt(y_sum * y_sum + x_sum * x_sum);//index of checking how far the robot have shifted from the line
      float move_deg = -1;
      float move_speed = 50;
      if(ball_on_right){
        move_deg = right_deg;
        // always positive speed, deg decide the direction
        //move_speed = move_speed * (angle_diff(subMonitor.ball_angle, 90.0f) / 90.0f);
      }
      else if (ball_on_left){
        move_deg = left_deg;
        // always positive speed, deg decide the direction
        //move_speed = move_speed * (angle_diff(subMonitor.ball_angle, 90.0f) / 90.0f);
      }
      else if(!subMonitor.ball_valid){
        // if no ball detected, move to the direction of the line
        if(left_us < 40){
          move_deg = right_deg;
        }
        if(right_us < 40){
          move_deg = left_deg;
        }
        move_speed = 30;
      }
      Serial.printf("move_deg: %f, lock_deg: %f left_deg: %f, right_deg: %f, mag%f\n", move_deg, lock_deg, left_deg, right_deg, mag);
      if(move_deg != -1){
        if(move_speed < 20){ 
          move_speed = 20;
        }
        if(move_speed > 50){
          move_speed = 50;
        }
        move_speed = 50;
        float move_rad = move_deg * DtoR_const;
        vx = move_speed * cosf(move_rad);
        vy = move_speed * sinf(move_rad);
        if(left_us < 40){
          vx += (40-left_us) / 40.0f * 15.0f;
        }
        if(right_us < 40){
          vx += -(40-right_us) / 40.0f * 15.0f;
        }

        //還需要調整

        
        /*
        const float STOP_US  = 30.0f;  // 低於這個距離開始煞車
        const float DECAY_K  = 0.01f;  // 越大煞車越急，越小煞車越平緩

        // move_deg 在 0~180(偏右移動)且右側太近牆 → 指數衰減煞停
        if(move_deg >= 0.0f && move_deg < 180.0f && right_us < STOP_US){
          float scale = expf(-DECAY_K * (STOP_US - right_us));
          vx *= scale;
          vy *= scale;
        }
        // move_deg 在 180~360(偏左移動)且左側太近牆 → 指數衰減煞停
        else if(move_deg >= 180.0f && move_deg < 360.0f && left_us < STOP_US){
          float scale = expf(-DECAY_K * (STOP_US - left_us));
          vx *= scale;
          vy *= scale;
        }*/
        //Serial.printf("left_us: %d, right_us: %d\n", left_us, right_us);
        //vx = fmaxf(90.0f * expf(-0.1f * fmaxf(fabsf(subMonitor.pos_x) - 25.0f, 0.0f)), 20.0f) * cosf(move_rad);
        //vy = fmaxf(90.0f * expf(-0.1f * fmaxf(fabsf(subMonitor.pos_x) - 25.0f, 0.0f)), 20.0f) * sinf(move_rad);
      }
      else if(lock_deg != -1){
        float lock_rad = lock_deg * DtoR_const;;
        vx = mag * 15 * cosf(lock_rad);
        vy = mag * 15 * sinf(lock_rad);
      }
      if(top_deg != -1 && down_deg != -1 && fabsf(down_deg - 270) < 45.0f){
        Serial.println("side");
        float escape_deg = get_line_move_deg(9, lineData.state, 8);
        Serial.printf("escape%f\n", escape_deg);
        bool robot_on_right = (escape_deg > 90 && escape_deg < 180);
        bool robot_on_left = (escape_deg < 90 && escape_deg > 0);
        bool robot_at_corner = (escape_deg < 112.5 && escape_deg > 57.5);
        move_deg = -1;
        if(ball_on_right){
          Serial.println("ball right lock");
          if(robot_on_left){           
            move_deg = escape_deg;
          }
        }
        else if (ball_on_left){
          Serial.println("ball left lock");
          if(robot_on_right){
            move_deg = escape_deg;
          }
        }
        if(robot_at_corner){
          move_deg = escape_deg;
        }
        if(move_deg != -1){
          float move_rad = move_deg * DtoR_const;
          vx = 70.0f * cosf(move_rad);
          vy = 70.0f * sinf(move_rad);
        }
        else{
          //車和球都在同一側，則不動
          vx = 0;
          if(vy < 0){
            vy = 20;
          }
        }
      }
      //Serial.printf("move_deg: %f, lock_deg: %f left_deg: %f, right_deg: %f, mag%f\n", move_deg, lock_deg, left_deg, right_deg, mag);
    }
  }
  Serial.printf("right_us: %d, left_us: %d, back_us: %d\n", right_us, left_us, back_us);
  if(left_us < 40){
    Serial.printf("left_us\n");
    vx = (40-left_us) / 40.0f * 15.0f;
    if(vx < 20 && vx > 0) vx = 20;
  }
  if(right_us < 40){
    Serial.printf("right_us\n");
    vx = -(40-right_us) / 40.0f * 15.0f;
    if(vx > -20 && vx < 0) vx = -20;
  }
  if(back_us < 20){
    Serial.printf("back_us\n");
    vy = (20-back_us) / 20.0f * 20.0f;
    if(vy < 20 && vy > 0) vy = 20;
  }
  if(vx > 90){
    vx = 90;
  }
  else if(vx < -90){
    vx = -90;
  }
  if(vy > 90){
    vy = 90;
  }
  else if(vy < -90){
    vy = -90;
  }
  Serial.printf("vx: %f, vy %f\n", vx, vy);
  Vector_Motion(vx, vy, 0);
}
void offense(){
  static float init_lineDegree = -1;
  static float diff = 0;
  static bool first_detect = false;
  float vx = 0;
  float vy = 0;
  float sumX = 0.0f;
  float sumY = 0.0f;
  int count = 0;
  for(int i = 0; i < LS_count; i++){
    if(bitRead(lineData.state, i) == 0){
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
    }
  }
  if(count > 1){
    float lineDegree = atan2(sumY, sumX) * 57.2958f;
    if(lineDegree < 0) lineDegree += 360;

    if(!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
    }

    diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180) diff = 360 - diff;

    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      finalDegree = fmod(init_lineDegree + 180.0f, 360.0f);
    }
    else{
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    vx = 70.0f * cos(finalDegree * DtoR_const);
    vy = 70.0f * sin(finalDegree * DtoR_const); 
  }
  else{
    first_detect = false;
    if(subMonitor.ball_valid){
      float moving_degree = subMonitor.ball_angle;
      float offset = 0;
      float ballspeed = constrain(map(subMonitor.ball_dist, 55, 80, 30, 60), 30, 60);
      //float ballspeedVx = constrain(map(subMonitor.ball_dist, 70, 85, 30, 50), 30, 50);
      //float ballspeedVy = constrain(map(subMonitor.ball_dist, 70, 85, 25, 50), 25, 50);

      //Serial.println(subMonitor.ball_dist);
      if(subMonitor.ball_angle >= 83 && subMonitor.ball_angle <= 97){
        //moving_degree = subMonitor.ball_angle;
        moving_degree = 90;
        ballspeed = 40;
      }
      else if(subMonitor.ball_angle > 97 && subMonitor.ball_angle <= 130){
        ballspeed = constrain(map(subMonitor.ball_dist, 55, 80, 30, 50),30,50);
        float offsetRatio = exp(-0.1 * (subMonitor.ball_dist - 65));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 10 * offsetRatio;
        moving_degree = subMonitor.ball_angle + offset;
      }
      else if(subMonitor.ball_angle > 130 && subMonitor.ball_angle < 155){
        float offsetRatio = exp(-0.1 * (subMonitor.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 90 * offsetRatio;
        moving_degree = subMonitor.ball_angle + offset;
        digitalWrite(LED_BUILTIN,LOW);
        //float angleError = fabs(ballData.angle - 90);
        //float smoothWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
      }
      else if(subMonitor.ball_angle >= 155 && subMonitor.ball_angle <= 270){
        float offsetRatio = exp(-0.03 * (subMonitor.ball_dist - 65));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 100 * offsetRatio;
        //Serial.print(" offset=");Serial.print(offset);
        moving_degree = subMonitor.ball_angle + offset;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if(subMonitor.ball_angle >= 50 && subMonitor.ball_angle < 83){
        ballspeed =constrain(map(subMonitor.ball_dist, 55, 80, 30, 50),30,50); //30;
        float offsetRatio = exp(-0.1 * (subMonitor.ball_dist - 65));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 10 * offsetRatio;
        moving_degree = subMonitor.ball_angle - offset;
      }
      else if(subMonitor.ball_angle < 50 && subMonitor.ball_angle >=25){
        float offsetRatio = exp(-0.1 * (subMonitor.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 90 * offsetRatio;
        moving_degree = subMonitor.ball_angle - offset;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if(subMonitor.ball_angle < 25 || subMonitor.ball_angle > 270){
        float offsetRatio = exp(-0.03 * (subMonitor.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 100 * offsetRatio;
        moving_degree = subMonitor.ball_angle - offset;
        digitalWrite(LED_BUILTIN,LOW);
      }
      if(moving_degree < 0) moving_degree += 360;
      if(moving_degree >= 360) moving_degree -= 360;
      vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
      vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
      Serial.print(" ball=");Serial.print(subMonitor.ball_angle);
      Serial.print(" balldist=");Serial.print(subMonitor.ball_dist);
      Serial.print(" balldist=");Serial.print(subMonitor.ball_dist);
      Serial.print(" move=");Serial.println(moving_degree);
      Serial.print(" ballspeed=");Serial.println(ballspeed);
    }
    else{
      vx = 0;
      vy = 0;
    }
    Vector_Motion(vx, vy, 0);
  }
  
}
void loop(){
  MotorStop();
  while(1){
    Serial.println("Waiting for MOVE_CMD from MainCore...");
    if(Serial8.available()){
      uint8_t cmd = Serial8.read();
      Serial.println(cmd, HEX);
      if(cmd == MOVE_CMD){
        Serial8.write(PROTOCAL_ACT);
        break;
      }
    }
  }  

  /*main loop*/
  while(1){
    static uint32_t offense_start_time = 0;
    update_gyro_sensor();
    update_line_sensor();
    readMaincoreData();
    if(subMonitor.role == 255){
      subMonitor.role = 0;
      break; // Exit the loop if the role is 255, indicating a special condition  }
    }
    /*test*/
    //mode=-1;//can be removed, just for test
    /*test*/
    switch(mode){
      case -1:{
        defense_mode3();
        mode = -1; // switch to defense mode
        break;
      }
      case 0:{
        //defense_mode3();
        //判斷是否于禁區
        if(abs(subMonitor.pos_x) < 30 && subMonitor.pos_y < -70 && subMonitor.pos_y > -90){
          Serial.println("mode 0");
          defense_mode3();
        }
        else{
          //out of penalty area
          float vx = 0, vy = 0;
          float dx = 0.0f - subMonitor.pos_x;
          float dy = -80.0f - subMonitor.pos_y;
          if (subMonitor.pos_y > -90.0f && subMonitor.pos_y < -80.0f) {
            dy = 0.0f;
          }
          if (subMonitor.pos_x < 30.0f && subMonitor.pos_x > -30.0f) {
            dx = 0.0f;
          }

          // Increase the response so the robot returns to home more promptly.
          float base_vx = dx * 0.30f;
          float base_vy = dy * 0.30f;

          if(base_vx != 0 && fabs(base_vx) < 20.0f) base_vx = (base_vx > 0) ? 20.0f : -20.0f;
          if (base_vy != 0 && fabs(base_vy) < 20.0f) base_vy = (base_vy > 0) ? 20.0f : -20.0f;

          vx = constrain(base_vx, -80.0f, 80.0f);
          vy = constrain(base_vy, -80.0f, 80.0f);

          // ---- ball avoidance correction ----
          float speed = sqrtf(vx * vx + vy * vy);

          if (speed > 0.001f) {
            // move_deg: 0=right, 90=front, 180=left, 270=back
            float move_deg = atan2f(vy, vx) * 180.0f / PI;
            if (move_deg < 0.0f) move_deg += 360.0f;

            if (subMonitor.ball_dist > 4.0f) {
              float diff = subMonitor.ball_angle - move_deg;
              diff = fmodf(diff, 360.0f);
              if (diff < 0.0f) diff += 360.0f;

              if (diff >= 270.0f && diff < 360.0f) {
                move_deg -= 90.0f;
              } else if (diff >= 180.0f && diff < 270.0f) {
                move_deg += 90.0f;
              }

              if (move_deg < 0.0f) move_deg += 360.0f;
              if (move_deg >= 360.0f) move_deg -= 360.0f;

              float rad = move_deg * PI / 180.0f;
              vx = cosf(rad) * speed;
              vy = sinf(rad) * speed;
            }
          }
          // Only dead-band near-zero noise; avoid forcing a minimum speed that makes the return feel sluggish.
          if (fabsf(vx) < 4.0f) vx = 0.0f;
          if (fabsf(vy) < 4.0f) vy = 0.0f;
          Vector_Motion(vx, vy, 0);
        }
        break;
      }
      case 1:{
        Serial.println("mode 1");
        int count = 0;
        for (int i = 0 ; i < 32; i++) {
          if (!((lineData.state >> i) & 1)) {
            count++;
          }
        }
        if(count > 3){
          Vector_Motion(0,30,0);
        }
        else{
          mode = 2; // switch to offense mode
        }
        break;
      }
      case 2:{
        if(offense_start_time == 0){
          offense_start_time = millis();
        }
        Serial.println("mode 2");
        offense();
        if(millis() - offense_start_time > 3000){
          Serial.println("offense end");
          mode = 0; // switch back to defense mode
          offense_start_time = 0;
        }
        break;
      }
    }
  }
}   