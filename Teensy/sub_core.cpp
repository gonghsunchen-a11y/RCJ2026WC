#include "sub_core.h"

// 1. Define the actual memory for these variables here
LineData lineData; 
GyroData gyroData;

int readMux(int ch, int sig) {
    digitalWrite(s0, (ch >> 0) & 1);
    digitalWrite(s1, (ch >> 1) & 1);
    digitalWrite(s2, (ch >> 2) & 1);
    digitalWrite(s3, (ch >> 3) & 1);
    delayMicroseconds(10);
    if(sig == 1){
        return analogRead(M1);
    }
    if(sig == 2){
        return analogRead(M2);
    }
    return 4095;
}

void update_line_sensor(){
  lineData.state = 0xFFFFFFFF;
  
  for (uint8_t i = 0; i < LS_count; i++) {
    uint16_t reading = readMux(i % 16, (i < 16) ? 1 : 2);;
    
    if (reading < avg_ls[i]) {
      lineData.state &= ~(1UL << i); 
      //Serial.printf("%d,%d",i,reading);
      //Serial.print(" avg ");Serial.print(i);Serial.print(" = ");Serial.print(avg_ls[i]);
      //Serial.println();
    }
  }
}

void fast_update_line_sensor(){
  static uint32_t prevRaw = 0xFFFFFFFF;
  uint32_t rawState       = 0xFFFFFFFF;

  for(uint8_t ch = 0; ch < 16; ch++){
    digitalWriteFast(s0, (ch >> 0) & 1);
    digitalWriteFast(s1, (ch >> 1) & 1);
    digitalWriteFast(s2, (ch >> 2) & 1);
    digitalWriteFast(s3, (ch >> 3) & 1);
    /*might be allowed to removed*/
    delayMicroseconds(10);

    uint16_t r1 = analogRead(M1);
    uint16_t r2 = analogRead(M2);

    if(r1 < avg_ls[ch])    rawState &= ~(1UL << ch);
    if(r2 < avg_ls[ch+16]) rawState &= ~(1UL << (ch + 16));
  }

  lineData.state = prevRaw | rawState;
  prevRaw        = rawState;
}

void update_gyro_sensor(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.exist = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    esti_checksum %= 256;

    // Compare with buffer[18]
    if(esti_checksum != buffer[18]){
      //Serial.println("Checksum error");
      continue;
    }

    // --- Extract yaw (Little Endian) ---
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);

    //Serial.print("yaw_raw: ");
    //Serial.println(yaw_raw);

    // Convert to degrees if within range
    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.exist = true;
    }
    
    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }
    break; // Process one packet per call
  }
}

void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * 50, 1.5 * 50);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 1:
      Serial.print("1 ");Serial.println(speed);
      if(speed<0){
        digitalWrite(DIR_1, LOW);
        analogWrite(pwmPin1, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_1, HIGH);
        analogWrite(pwmPin1, pwmVal);
      }
      else{
        analogWrite(pwmPin1, 0);
      }
    break;
    case 2:
      if(speed<0){
        digitalWrite(DIR_2, LOW);
        analogWrite(pwmPin2, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_2, HIGH);
        analogWrite(pwmPin2, pwmVal);
      }
      else{
        analogWrite(pwmPin2, 0);
      }
      //Serial.print("2 ");Serial.println(pwmVal);
      break;
    case 3:
      if(speed<0){
        digitalWrite(DIR_3, LOW);
        analogWrite(pwmPin3, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_3, HIGH);
        analogWrite(pwmPin3, pwmVal);
      }
      else{
        analogWrite(pwmPin3, 0);
      }
      break;
    case 4:
      if(speed<0){
        digitalWrite(DIR_4, LOW);
        analogWrite(pwmPin4, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_4, HIGH);
        analogWrite(pwmPin4, pwmVal);
      }
      else{
        analogWrite(pwmPin4, 0);
      }
      //Serial.print("4 ");Serial.println(pwmVal);
    break;
  }
}
