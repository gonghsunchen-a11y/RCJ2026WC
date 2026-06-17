#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#define SENSE(i) (!(lineData.state >> (i)&1))

#define possession_Threshold 205

//SPEED
float ballVx = 0;
float ballVy = 0;
float lineVx = 0;
float lineVy = 0;

int count = 0;
float x = 2;
float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

int lastLeftState = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;
bool showData = false; // false = Start/Run, true = 顯示數據
bool showRun = false;
// #define DEBUG
void robot_offense();
bool Debug();
bool ball_search();
void offense();
void line_processing();
void attack();
void outlinesensor();

void setup(){
  Robot_Init();
  showMessage("Start");
}

void loop(){
  //Debug();
  while(Debug());
  while(1){
    attack();
  }  
}

bool Debug(){
  int leftState = digitalRead(BUTTON_LEFT);
  int rightState = digitalRead(BUTTON_RIGHT);

  // ------------------ 左鍵切換 Start / Data ------------------
  if(leftState == LOW && lastLeftState == HIGH &&(millis() - lastPress) > 100){
    showData = !showData;
    showRun = false; // 切回 Data 時取消 Run
    lastPress = millis();
    if(!showData){
      showMessage("Start");
    }
  }
  lastLeftState = leftState;

  // ------------------ 右鍵在 Start 顯示 Run ------------------
  if(rightState == LOW && lastRightState == HIGH &&(millis() - lastPress) > 100){
    if(!showData){ // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if(showRun){
        showMessage("Run");
        return false;
      }
    }
    else{
      showMessage("Start");
    }
  }
  lastRightState = rightState;

  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();
  //readussensor();
  if(showData &&(millis() - lastUpdate > 200)){ 
    display.clearDisplay();
    showSensors(gyroData.heading, ballData.dir, lineData.valid);
    showUS(usData.dist_l, usData.dist_r, usData.dist_b);
    if(rightState == LOW){
      showMessage("scanning...");
      Serial5.write(0xAA);
      while(digitalRead(BUTTON_LEFT));
      Serial5.write(0xEE);
    }
    // Serial.print("ball_dir="); Serial.println(ballData.dir);
    lastUpdate = millis();
  }
  Serial.println("debug");
  return true;
}
//                    外側光感
void outlinesensor(){
  if(analogRead(back_ls) >= 480){
    backtouch = true;
  }
  else{
    backtouch = false;
  }
  if(analogRead(left_ls) >= 550){
    lefttouch = true;
  }
  else{
    lefttouch = false;
  }
  if(analogRead(right_ls) >= 550){
    righttouch = true;
  }
  else{
    righttouch = false;
  }
}
void attack(){
  static int catch_timer = 0;
  static int16_t le = 0;
  static int16_t re = 0;
  //HIGH FREQUENCY
  readBNO085Yaw();
  linesensor();
  readussensor();
  outlinesensor();
  //LOW FREQUENCY
  unsigned long now = millis();
  if(now - lastUpdate >= interval){
    lastUpdate = now;
    readCameraData();
    kicker_control(0);
    control.picked_up =(gyroData.pitch > 20) ? true : false;
  }
  //Serial.print("height");Serial.println(targetData.h);
  double rotate = 0;
  if(targetData.valid){
    if(targetData.y > 90){
      if(targetData.x >= 130 && targetData.x <= 190){
        rotate=0;
        le = 0;
        re = 0;
      }
      else if(targetData.x < 120){
        int16_t e = (120-targetData.x);
        //le = le - e;
        rotate = 0.8 * (e / 120.0);
        //le = e;
      }
      else if(targetData.x > 200){
        int16_t e = (targetData.x - 200);
        //re = re - e;
        rotate = -0.8 * (e / 120.0);
        //re = e;
      }
    }
    control.robot_heading += rotate;
    if(control.robot_heading < 35){ 
      control.robot_heading = 35;
    }
    if(control.robot_heading > 145){ 
      control.robot_heading = 145;
    }
  }
  else{
    rotate = control.robot_heading - 90;
    if(rotate > 10){
      rotate = -0.8;

    }
    else if(rotate < -10){
      rotate = 0.8; 
    }
    else{
      rotate = 0;
    }
    control.robot_heading += rotate; 
  }
  
  /*Serial.print(" Y=");
  Serial.print(targetData.y);
  Serial.print(" W=");
  Serial.print(targetData.w);
  Serial.print(" H=");
  Serial.println(targetData.h);*/
  
  // Serial.print("lineData.state=");
  // Serial.println(lineData.state, BIN);
  //Serial.print("dist_f = ");Serial.println(dist_f);
  //Serial.print("dist_b = ");Serial.println(dist_b );
  //Serial.print("dist_l = ");Serial.println(dist_l);
  //Serial.print("dist_r = ");Serial.println(dist_r);
  float sumX = 0, sumY = 0;
  static int speed_timer = 0;
  //bool hori_half = (SENSE(0)| SENSE(1) | SENSE(2))&(SENSE(6) | SENSE(7) | SENSE(8));
  //bool vert_half = (SENSE(2) | SENSE(3) | SENSE(4) | SENSE(5) | SENSE(6))&(SENSE(11) | SENSE(12) | SENSE(13) | SENSE(14) | SENSE(5));
  for(int i = 0; i < 18; i++){
    bool detected =((lineData.state &(1UL << i)) == 0); // 0 表有線
    // Serial.print("Sensor "); Serial.print(i);
    // Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
    if(detected){
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
    }
  }
  if(lineData.state == 0b111111111111111111 && !overhalf && count > 0){ // no line
    count = 0;
    lineVx = 0;
    lineVy = 0;
    first_detect = false;
    init_lineDegree = -1;
    speed_timer = 0;
  }
  if(control.picked_up){
    count = 0;
    lineVx = 0;
    lineVy = 0;
    first_detect = !first_detect;
    init_lineDegree = -1;
    overhalf = false;
    speed_timer = 0;
  }
  if(count > 0 || overhalf){
    speed_timer++;
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if(lineDegree < 0){
      lineDegree += 360;
    }
    if(start){                                
      showMessage("LINE");
      start = false;
    }
    // Serial.print("sumX="); Serial.print(sumX);
    // Serial.print(", sumY="); Serial.print(sumY);
    // Serial.print(", average lineDegree="); Serial.println(lineDegree);
    if(first_detect == false){
      init_lineDegree = lineDegree;
      first_detect = true;
    }
    if(init_lineDegree > 270 && lineDegree < 90){
      lineDegree += 360;
    }
    else if(lineDegree > 270 && init_lineDegree < 90){
      init_lineDegree += 360;
    }
    
    diff = fabs(fmod((lineDegree - init_lineDegree), 360));
    float finalDegree;
    float speed = 40;
    //Serial.printf("overhalf = %d", overhalf);
    //Serial.printf("lineDegre = %d", lineDegree);
    if(diff > EMERGENCY_THRESHOLD){
      /*
      if(overhalf){
        finalDegree = fmod(init_lineDegree + 180, 360);
      }
      else{
        overhalf = true;
        finalDegree = lineDegree;
      }*/
      overhalf = true;
      finalDegree = lineDegree;
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180, 360);
      // delay(1000);
    }

    /*
    if(overhalf && (hori_half | vert_half)){
      finalDegree =  init_lineDegree + 180;
    }*/
    /*
    Serial.print("lineDegree=");
    Serial.println(lineDegree);
    Serial.print("finalDegree=");
    Serial.println(finalDegree);
    Serial.print("first_detect");
    Serial.println(first_detect);
    */
    //float speed = constrain(speed_timer, -40,40);
    lineVx = speed * cos(finalDegree * DtoR_const);
    lineVy = speed * sin(finalDegree * DtoR_const);
    Vector_Motion(int(lineVx), int(lineVy));
    //Vector_Motion(0,0);
    //Serial.print("lineVx="); Serial.print(lineVx);
    //Serial.print("lineVy="); Serial.println(lineVy);
  }
  else{
    lineVx = 0;
    lineVy = 0;
    if(!start){
      showMessage("Start");
      start = true;
    }
    ballsensor();
    if(ballData.dir == 255){ 
      Vector_Motion(0,0);
      /*if(usData.dist_b <= 110 && usData.dist_b >= 90){
        ballVy = 0;
      }
      else if(usData.dist_b < 90 ){
        ballVy = 25;
      }
      else if(usData.dist_b > 110){
        ballVy = -25;
      }
    Serial.print("ballVy");Serial.println(ballVy);
    

    if((abs(usData.dist_r - usData.dist_l)) <= 30){
        ballVx = 0;
      }
      else if(abs(usData.dist_r - usData.dist_l) > 30 && usData.dist_r > usData.dist_l){
        ballVx = 25;
      }
      else if(abs(usData.dist_r - usData.dist_l) > 30 && usData.dist_r < usData.dist_l){
        ballVx = -25;
      }*/
    //Serial.print("ballVx");Serial.println(ballVx);
    }
    else{
      float ballDegree = ballDegreelist[ballData.dir];
      float offset = 0;
      //Serial.print("balldir=");Serial.println(ballData.dir);
      //Serial.print("ballDegree=");Serial.println(ballDegree);
      //Serial.print("ballData.dis=");Serial.println(ballData.dis);
      // Serial.print("exp");Serial.println(exp(-0.55*(ballData.dis-7)));
      //delay(500);
      float ballspeed = map(ballData.dis, 0, 12, 20, 40);
      if(ballDegree == 87.5 || ballDegree == 92.5){
        offset = 0;
        ballspeed = 40;
      }
      else{
        double offsetRatio = exp(-0.55 *(ballData.dis - 7));
        offsetRatio =(exp(-0.55 *(ballData.dis - 7)) > 1) ? 1 : offsetRatio;
        offset = 95 * offsetRatio;
        offset =(ballDegree > 90) ? offset : -offset;
        offset =(ballDegree < 270) ? offset : -offset;
        //Serial.print("offset=");
        //Serial.println(offset);
      }
      float moving_Degree = ballDegree + offset;
      // Serial.print("moving_Degree="); Serial.println(moving_Degree);
      ballspeed = constrain(ballspeed, 20, 40);
      // Serial.print("BallSpeed="); Serial.println(ballspeed);
      ballVx = ballspeed * cos(moving_Degree * DtoR_const);
      ballVy = ballspeed * sin(moving_Degree * DtoR_const);
    }

    // INTERUPT
    /*if(backtouch && ballVy < 0){
      ballVy = 0;
      if(lefttouch && backtouch){
        if(targetData.valid == 65535){
          ballVy = 30;
        }
        else if(targetData.valid != 65535){
          ballVx = 30;  
        } 
      }
      else if(righttouch && backtouch){
        if(targetData.valid == 65535){
          ballVy = 30;
        }
        else if(targetData.valid != 65535){
          ballVx = -30;
        } 
      } 
    }
    if(lefttouch && ballVx < 0){
      if(back_us < 60){
        ballVy = 30;
      }
      else{
        ballVx = 20;
      }
      if(lefttouch && backtouch){
        if(targetData.valid == 65535){
          ballVy = 30;
        }
        else if(targetData.valid != 65535){
          ballVx = 30;
        } 
      }
    }
    if(righttouch && ballVx > 0){
      if(analogRead(back_us) < 60){
        ballVy = 30;
      }
      else{
        ballVx = -20;
      }
      if(righttouch && backtouch){
        if(targetData.valid == 65535){
          ballVy = 30;
        }
        else if(targetData.valid != 65535){
          ballVx = -30;
        } 
      }
    }
    /*if(lefttouch || righttouch){
      if(targetData.h > 25 && ballVy > 0){
        ballVy = -15;
      }
    }*/
   /*
    if(analogRead(right_us) < 50 && ballVx > 0){
      ballVx *= 0.5;
    }
    else if(analogRead(left_us) < 50 && ballVx < 0){
      ballVx *= 0.5;
    }/*
    /*if(backtouch && lefttouch && targetData.h != 65535){
    }
    else if(backtouch && righttouch && targetData.h != 65535){
    }*/
    if(ballData.possession < possession_Threshold){
      ballVy = MAX_V;
      //catch_timer+=2;
      //if(catch_timer > 4){
        //if(targetData.valid){
          //if(targetData.h > 23){
            //kicker_control(1);
            //Serial.print("kick");
            //catch_timer = 0;
          //}
        //}
      //}
    }
    else{
      //catch_timer--;
      //if(catch_timer < 0){
        catch_timer = 0;
      //}
    }
    //Serial.println(catch_timer);
    //        降速
    if(targetData.valid){
      if(targetData.x >= 210 && ballVx < 0){
        Serial.println("1");
        ballVx = ballVx * 0.7;
        if(targetData.h > 30 && ballVy > 0){
          ballVy = 0.7 * ballVy;
        }
      }
      else if(targetData.x <= 110 && ballVx > 0){
        Serial.println("2");
        ballVx = ballVx * 0.7;
        if(targetData.h > 30 && ballVy > 0){
          ballVy = 0.7 * ballVy;
        }
      }/*球門降速
      if(ballVy > 0 && targetData.h != 65535){
        Serial.println("3");
        if(targetData.h > 25){
          ballVy = 20;
        }
        else if(targetData.h < 25){
          ballVy *= 0.8;
        }
      }*/
    }
    else{
      if(analogRead(left_us < 80) && ballVx < 0 || analogRead(right_us) < 80 && ballVx > 0){
        ballVx *= 0.7;
        Serial.print("0.7");
      }
      if(analogRead(left_us) < 60 && ballVx < 0 || analogRead(right_us) < 60 && ballVx > 0){
        ballVx *= 0.4;
        if(analogRead(front_us) < 40){
          ballVy = -15;
        }
        if(analogRead(front_us) < 30){
          ballVy = -30;
        }
        Serial.print("0.5");
      }
    }
    if(analogRead(left_us) < 25 && ballVx < 0 ){
      ballVx = 30;
      Serial.print("-15");
    }
    if(analogRead(right_us) < 25 && ballVx > 0){
      ballVx = -30;
    }


    Vector_Motion(int(ballVx), int(ballVy));

    //Vector_Motion(0,0);
    //Serial.print("ballVy");Serial.println(ballVy);
    //Serial.print("ballVx");Serial.println(ballVx);
  }
  // float finalVx =(lineVx != 0) ? lineVx : ballVx + lineVx;
  // float finalVy =(lineVy != 0) ? lineVy : ballVy + lineVy;
  

  Serial.print("Target X =");Serial.println(targetData.x);
  //Serial.print("Target Y =");Serial.println(targetData.y);
  //Serial.print("Target H =");Serial.println(targetData.h);
  //Serial.print("control.robot_heading =");Serial.println(control.robot_heading);
  //Serial.print("rotate =");Serial.println(rotate);
  //Serial.print("ballData.possession =");Serial.println(ballData.possession);
  //Serial.print("usback =");Serial.println(analogRead(back_us));
  //Serial.print("gyroData.pitch=");Serial.println(gyroData.pitch);
  //Serial.print("usleft =");Serial.println(analogRead(left_us));  
  //Serial.print("usright =");Serial.println(analogRead(right_us));
}