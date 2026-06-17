#include <sub_core.h>
uint8_t op_mode;

bool moveBackInBounds(){
  //-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  static float init_lineDegree = -1;
  static float diff = 0;
  static bool emergency = false;
  static bool start = false;
  static bool overhalf = false;
  static bool first_detect = false;
  static uint32_t speed_timer = 0;

  for(int i = 0; i < LS_count; i++){
    if(bitRead(lineData.state, i) == 0){
      //if(i==0){continue;}
      
      //Serial.printf("read%d", i);
      
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

  // B : 反彈

  if(linedetected && count >= 1){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 
    
    //Serial.print("degree=");Serial.println(lineDegree);

    if (!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
      speed_timer = millis();
      
      Serial.println("LINE DETECTED !!!");
      Serial.print("initlineDegree =");Serial.println(init_lineDegree);
    }

    diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180){diff = 360 - diff;}
    
    //Serial.print("diff =");Serial.println(diff);


    //-----BACK TO FIELD-----
    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree + 180.0f, 360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineVx = 60.0f *cos(finalDegree * DtoR_const);
    lineVy = 60.0f *sin(finalDegree * DtoR_const);   
    return true;
  }
  else{
    first_detect = false;
    lineVx = 0;
    lineVy = 0;
    return false;
  }
}

void c_mode_main_function() {
    Serial.println("Cmode Started");
    unsigned long f_front_line_timer = 0;
    unsigned long f_back_line_timer = 0;
    unsigned long left_line_timer = 0;
    unsigned long right_line_timer = 0;
    while (1){
      read_cam_and_pos_data();
      update_line_sensor(); // Keep updating sensors!
      update_gyro_sensor();
      Serial.printf("Gyro Heading: %f\n", gyroData.heading);
      Serial.printf("Ball Valid: %d, Ball Angle: %d, Ball Distance: %d \n", ballData.valid, ballData.angle, ballData.dist);
      Serial.printf("Robot Pos: (%f, %f)\n", RobotPos.x, RobotPos.y);
      float ball_vx = 0;
      
      bool f_back_touch = !((lineData.state >> 8) & 1); // Example: using the first line sensor as f_back touch
      static bool f_back_touch_state = false;
      bool front_touch = analogRead(Front_LS) < avg_ls[32];
      static bool f_front_touch_state = false;
      bool mid_touch = analogRead(Mid_LS) < avg_ls[33];

      bool left_in_touch = !((lineData.state >> 20) & 1) || !((lineData.state >> 21) & 1);
      bool left_out_touch = !((lineData.state >> 15) & 1) || !((lineData.state >> 16) & 1) || !((lineData.state >> 17) & 1);
      static bool left_touch_state = false;

      bool right_in_touch = !((lineData.state >> 27) & 1) || !((lineData.state >> 28) & 1);
      bool right_out_touch = !((lineData.state >> 0) & 1) || !((lineData.state >> 1) & 1) || !((lineData.state >> 31) & 1);
      static bool right_touch_state = false;

      bool robot_back_touch = !((lineData.state >> 22) & 1) || !((lineData.state >> 23) & 1) || !((lineData.state >> 24) & 1) || !((lineData.state >> 25) & 1) || !((lineData.state >> 26) & 1);
      bool robot_front_touch = !((lineData.state >> 6) & 1) || !((lineData.state >> 7) & 1) || !((lineData.state >> 8) & 1) || !((lineData.state >> 9) & 1) || !((lineData.state >> 10) & 1);
      bool vertical_line = robot_back_touch && robot_front_touch;

      bool ball_left = ballData.valid && (ballData.angle > 105 && ballData.angle < 270);
      bool ball_right = ballData.valid && (ballData.angle < 85 || ballData.angle > 270);
       
      //use Ultrasonic Sensor for localization
      


      //Ball Tracking
      if (ball_left) {
          // 180° = fully left (MAX_V), 90°/270° = barely left (0)
          ball_vx = -MAX_V * (1.0f - abs(180 - ballData.angle) / 90.0f);
      }
      else if (ball_right) {
          // 0°/360° = fully right (MAX_V), 85°/275° = barely right (0)
          int angle = ballData.angle;
          float dist = (angle <= 85) ? angle : (360 - angle); // distance from 0°/360°
          ball_vx = MAX_V * (1.0f - dist / 90.0f);
      }
      else if(!ball_left && !ball_right){
        ball_vx = 0;
      }

      //Vy logic
      /*Front Line Sensor Logic*/
      if(front_touch && !f_front_touch_state){
        f_front_touch_state = true;
        f_front_line_timer = millis();
      }
      /*f_back Line Sensor Logic*/
      if(f_back_touch && !f_back_touch_state){
        f_back_touch_state = true;
        f_back_line_timer = millis();
      }
      if(vertical_line && f_back_touch_state){
        f_back_touch_state = false;
        f_back_line_timer = 0;
        f_front_touch_state = true;
        f_front_line_timer = millis();
      }
      //Reset Vy timer
      if(mid_touch){
        f_back_line_timer = 0;
        f_front_line_timer = 0;
        f_back_touch_state = false;
        f_front_touch_state = false; 
      }

      //Left white line
      if(left_in_touch && !left_touch_state){
        left_touch_state = true;
        left_line_timer = millis();
      }
      else if(left_touch_state && left_out_touch && !left_in_touch){
        left_touch_state = false;
        left_line_timer = 0;
      }

      //Right white line
      if(right_in_touch && !right_touch_state){
        right_touch_state = true;
        right_line_timer = millis();
      }
      else if(right_touch_state && right_out_touch && !right_in_touch){
        right_line_timer = 0;
        right_touch_state = false;
      }

      
      /*
      //left and right corner edge case, prevent robot moving backwards
      else if(f_back_line_timer){
        f_back_line_timer = 0;
        f_back_touch_state = false;
        Serial.print("sec\n");
      }*/
      //the squence of 3 sensors touching the line are same in two sceneario:
      /* front then left || right : hoizontal line
         front then left || right : vertical line
      */
      //front edge case, half of the robot pass the front line and moving backwards
      if(left_line_timer > f_back_line_timer && right_line_timer > f_back_line_timer){
        right_touch_state = false;
        right_line_timer = 0;
        left_touch_state = false;
        left_line_timer = 0;
        Serial.print("fec\n");
      }
      /*
      else if(left_line_timer < right_line_timer && left_line_timer < f_back_line_timer){// left side edge case      
        right_touch_state = false;
        right_line_timer = 0;
        f_back_line_timer = 0;
        f_back_touch_state = false;
        Serial.print("lec\n");
      }
      else if(left_line_timer > right_line_timer && right_line_timer > f_back_line_timer){//side edge case
        left_touch_state = false;
        left_line_timer = 0;
        f_back_line_timer = 0;
        f_back_touch_state = false;
        Serial.print("rec\n");
      }*/
      
      /*Assign Velocity*/
      float line_vx = 0;
      //X axis
      if(left_line_timer != 0 && right_line_timer == 0){//move left motion
        line_vx = (20 + (millis() - left_line_timer) * 0.01);
        if(line_vx > MAX_V) line_vx = MAX_V;
      }
      else if(right_line_timer != 0 && left_line_timer == 0){//move right motion
        line_vx = -(20 + (millis() - right_line_timer) * 0.01);
        if(line_vx < -MAX_V) line_vx = -MAX_V;
      }

      float vx = (line_vx == 0) ? ball_vx : line_vx;

      if((ball_vx > 0 && right_out_touch) || (ball_vx < 0 && left_out_touch)){
        vx = 0;
      }
      else if((ball_vx < 0 && right_out_touch) || (ball_vx > 0 && left_out_touch)){
        vx = ball_vx;
      }
      /*(ball_vx > 0 && line_vx > 0) || (ball_vx < 0 && line_vx < 0) */
      
      // Y axis
      float vy = 0;
      Serial.println(f_front_line_timer, f_back_line_timer);
      if(f_front_line_timer && f_back_line_timer == 0){
        vy = (5 + (millis() - f_front_line_timer) * 0.1);
        if(vy > MAX_V) vy = MAX_V;
      }
      else if(f_front_line_timer == 0 && f_back_line_timer){
        vy = -(5 + (millis() - f_back_line_timer) * 0.1);
        if(vy < -MAX_V) vy = -MAX_V;
      }


      //Serial.printf("Front LS: %d, Mid LS: %d, Back LS: %d\n",  int(front_touch), int(mid_touch), int(f_back_touch));
      Serial.printf("vx: %f, vy: %f\n", vx, vy);
      FC_Vector_Motion(0, vy, 90); 
    }
}

void t_mode_main_function() {
    Serial.println("Tmode Started");
    while(1) {
        update_line_sensor(); // Keep updating sensors!
        update_gyro_sensor();
        readMotorandSendSensors();
        Serial.printf("vx:%f, vy:%f, rot_v:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v, mainCommand.heading);
        FC_Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.heading); 
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
    t_mode_main_function();
  }
}
/*
void loop(){
  update_line_sensor();
  update_gyro_sensor();
  readfrom_MainCore();
  switch (mainCommand.type) {
    case MainCoreCommand::ACTUATE:
      white_line_handle();
      break;
    case MainCoreCommand::CALIBRATE:
      calibrate();
      mainCommand.type = MainCoreCommand::ACTUATE; // Reset to default after calibration
      break;
  }
}
*/