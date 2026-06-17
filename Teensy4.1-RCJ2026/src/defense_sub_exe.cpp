#include <sub_core.h>
uint8_t op_mode;

float lineVx = 0;
float lineVy = 0;

// ------------------------------------------------------------
#define LS_MASK_FRONT  0x00000FE0UL
#define LS_MASK_RIGHT  0x001FF000UL
#define LS_MASK_BACK   0x0FE00000UL
#define LS_MASK_LEFT   0xF000001FUL

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f


const float linesensor_cos[32] = {
    1.0000f,  0.9808f,  0.9239f,  0.8315f,  0.7071f,  0.5556f,  0.3827f,  0.1951f,
    0.0000f, -0.1951f, -0.3827f, -0.5556f, -0.7071f, -0.8315f, -0.9239f, -0.9808f,
   -1.0000f, -0.9808f, -0.9239f, -0.8315f, -0.7071f, -0.5556f, -0.3827f, -0.1951f,
    0.0000f,  0.1951f,  0.3827f,  0.5556f,  0.7071f,  0.8315f,  0.9239f,  0.9808f
};

const float linesensor_sin[32] = {
    0.0000f,  0.1951f,  0.3827f,  0.5556f,  0.7071f,  0.8315f,  0.9239f,  0.9808f,
    1.0000f,  0.9808f,  0.9239f,  0.8315f,  0.7071f,  0.5556f,  0.3827f,  0.1951f,
    0.0000f, -0.1951f, -0.3827f, -0.5556f, -0.7071f, -0.8315f, -0.9239f, -0.9808f,
   -1.0000f, -0.9808f, -0.9239f, -0.8315f, -0.7071f, -0.5556f, -0.3827f, -0.1951f
};

//Line Sensor
#define EMERGENCY_THRESHOLD 90


int prev_final_degree = -1;
bool moveBackInBounds(){
  //-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  static float init_lineDegree = -1;
  static float diff = 0;
  static bool first_detect = false;
  static bool overhalf = false;
  bool online = false;
  for(int i = 0; i < LS_count; i++){
    if(i>=5 && i<= 9){ // ignore middle 5 sensors
      if(bitRead(lineData.state, i) == 0){
        return false;
      }
    } 
    if(bitRead(lineData.state, i) == 0){
      Serial.print(i);
      sumX += linesensor_cos[i];
      sumY += linesensor_sin[i];
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
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    prev_final_degree = finalDegree;
    Serial.print("finalDegree =");Serial.println(finalDegree);
    /*
    if(finalDegree < 45 || finalDegree >= 315){
        finalDegree = 0;
    }
    */
    if(finalDegree < 135 && finalDegree >= 45){
        finalDegree = 90;
    }
    else if(finalDegree < 225 && finalDegree >= 135){
        finalDegree = 180;
    }
    /*
    else if(finalDegree >= 225 && finalDegree < 315){
      finalDegree = 270;
    }*/
    
    float speed = 40;

    lineVx = speed *cos(finalDegree * DtoR_const);
    lineVy = speed * 0.5 *sin(finalDegree * DtoR_const);
    //corner edge case
    bool left = (lineData.state & LS_MASK_LEFT) != LS_MASK_LEFT;
    bool right = (lineData.state & LS_MASK_RIGHT) != LS_MASK_RIGHT;
    bool front = (lineData.state & LS_MASK_FRONT) != LS_MASK_FRONT;
    bool front_in = analogRead(A6) < avg_ls[32];

    if(right && !left){
      if(!front_in){
        lineVy = 20;
      }
    }
    if(!right && left){
      if(!front_in){
        lineVy = 20;
      }
    }
    return true;
  }
  else{
    first_detect = false;
    lineVx = 0;
    lineVy = 0;
    return false;
  }
}

void main_function() {
  Serial.println("Cmode Started");
  while (1){
    //read_cam_and_pos_data();
    readMainPacket();
    update_line_sensor(); // Keep updating sensors!
    update_gyro_sensor();
    //Serial.printf("Gyro Heading: %f\n", gyroData.heading);
    //Serial.printf("Ball Valid: %d, Ball Angle: %d, Ball Distance: %d \n", ballData.valid, ballData.angle, ballData.dist);
    Serial.printf("Robot Pos: (%d, %d)\n", RobotPos.x, RobotPos.y);
    Serial.printf("Goal Valid: %d, Goal X: %d, Goal W: %d , Y: %d\n, B: %d\n", goalData.valid, goalData.x, goalData.w, RobotPos.y, usData.dist_b);
    //use Ultrasonic Sensor for localization
    /*
    if(goalData.valid){
      if(goalData.x < 70){
        RobotPos.x = -goalData.dist;
      }
      else if(goalData.x > 90){
        RobotPos.x = goalData.dist;
      }
      else{
        RobotPos.x = 0;
      }
    }
    */
    /*
    if(usData.dist_b < 20 || usData.dist_b > 30 ){
      float back_vy = 0;
      if(usData.dist_b < 20){
        back_vy = 20;
      }
      else if(usData.dist_b > 30){
        back_vy = -20;
      }
      FC_Vector_Motion(0, back_vy, 90);
    }
    else if(goalData.x <= 320){
      float back_vx = 0;
      if(goalData.x > 260){
        back_vx = -20;
      }
      else if(goalData.x < 260){
        back_vx = 20;
      }
      FC_Vector_Motion(back_vx, 0, 90);
    }
    else{*/
      if(moveBackInBounds()){
        Serial.printf("MOVING BACK IN BOUNDS %f %f", lineVx, lineVy);
        FC_Vector_Motion(lineVx, lineVy, 90);
      }
      else{
        float ball_vx = 0;
        float ball_vy = 0;
        bool ball_left = ballData.valid && (ballData.angle > 105 && ballData.angle < 270);
        bool ball_right = ballData.valid && (ballData.angle < 85 || ballData.angle > 270);
        if (ball_left) {
            // 180° = fully left (MAX_V), 90°/270° = barely left (0)
            ball_vx = -20;
        }
        else if (ball_right) {
            // 0°/360° = fully right (MAX_V), 85°/275° = barely right (0)
            ball_vx = 20;
        }
        else if(!ball_left && !ball_right){
            ball_vx = 0;
        }
        
        static bool front_leave = false;
        //bool front_back = !((lineData.state >> 5) & 1) || !((lineData.state >> 6 ) & 1) || !((lineData.state >> 10) & 1) || !((lineData.state >> 11) & 1);
        if(analogRead(A7) < avg_ls[33]){//前白後黑
          front_leave = true;
        }
        else if(analogRead(A6) > avg_ls[32] && analogRead(A7) < avg_ls[33]){//前黑後白
          front_leave = false;
        }
        if(front_leave){
          ball_vy = 40;
        }
        else if(!front_leave){
          ball_vy =  0;
        }

        if(!((lineData.state >> 5) & 1) || !((lineData.state >> 9) & 1) ){
          ball_vy = -20;
        }
        else if(!((lineData.state >> 6) & 1) || !((lineData.state >> 7) & 1) || !((lineData.state >> 8) & 1)){
          ball_vy = -10;
        }
        Serial.printf("Vx%f,Vy%f\n", ball_vx, ball_vy);
        FC_Vector_Motion(ball_vx, ball_vy, 90); 
      }
    //}
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
  main_function();
}