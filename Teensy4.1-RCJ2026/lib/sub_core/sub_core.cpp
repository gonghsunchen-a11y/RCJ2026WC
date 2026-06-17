#include "sub_core.h"

// 1. Define the actual memory for these variables here
LineData lineData; 
GyroData gyroData;       
RobotStatus robot;
MainCoreCommand mainCommand;
GoalData goalData;
USSensor usData;

// --- Sensor Data ---
BallData ballData;
Position RobotPos;

uint16_t avg_ls[34];

void sub_core_init() {
    Serial8.begin(921600); // For communication with MainCore
    Serial2.begin(115200); // For gyro sensor
    Serial.begin(115200);  // For debugging

    // Multiplexer Control
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    //pinMode(M1, INPUT_PULLDOWN);
    //pinMode(M2, INPUT_PULLDOWN);

    pinMode(M1, INPUT);
    pinMode(M2, INPUT);

    // Motor Initialization
    // Motor 1
    pinMode(pwmPin1, OUTPUT);
    analogWriteFrequency(pwmPin1, 5000); // Set to 5 kHz
    pinMode(DIRA_1, OUTPUT);
    pinMode(DIRB_1, OUTPUT);
    // Motor 2
    pinMode(pwmPin2, OUTPUT);
    analogWriteFrequency(pwmPin2, 5000); // Set to 5 kHz
    pinMode(DIRA_2, OUTPUT);
    pinMode(DIRB_2, OUTPUT);
    // Motor 3
    pinMode(pwmPin3, OUTPUT);
    analogWriteFrequency(pwmPin3, 5000); // Set to 5 kHz
    pinMode(DIRA_3, OUTPUT);
    pinMode(DIRB_3, OUTPUT);
    // Motor 4
    pinMode(pwmPin4, OUTPUT);
    analogWriteFrequency(pwmPin4, 5000); // Set to 5 kHz
    pinMode(DIRA_4, OUTPUT);
    pinMode(DIRB_4, OUTPUT);

    //LED
    pinMode(LED_BUILTIN, OUTPUT);

    EEPROM.begin();
    EEPROM.get(0, avg_ls);
}

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

/* --- Actuators Part --- */

void SetMotorSpeed(uint8_t port, float speed) {
    // Constrain speed to prevent PWM overflow
    speed = constrain(speed, -255, 255); 
    //int pwmVal = abs((int)speed);
    int pwmVal = abs(speed) * 255 / 100;

    uint8_t p_pwm, p_a, p_b;
    switch(port) {
        case 4: p_pwm = pwmPin1; p_a = DIRA_1; p_b = DIRB_1; break;
        case 3: p_pwm = pwmPin2; p_a = DIRA_2; p_b = DIRB_2; break;
        case 2: p_pwm = pwmPin3; p_a = DIRA_3; p_b = DIRB_3; break;
        case 1: p_pwm = pwmPin4; p_a = DIRA_4; p_b = DIRB_4; break;
        default: return;
    }

    analogWrite(p_pwm, pwmVal);
    digitalWrite(p_a, (speed > 0) ? HIGH : LOW);
    digitalWrite(p_b, (speed < 0) ? HIGH : LOW);
}

void RobotIKControl(float vx, float vy, float omega) {
    // Applying the Inverse Kinematics Matrix
    float p1 = -0.643f * vx + 0.766f * vy + omega;
    float p2 = -0.643f * vx - 0.766f * vy + omega;
    float p3 =  0.707f * vx - 0.707f * vy + omega;
    float p4 =  0.707f * vx + 0.707f * vy + omega;

    SetMotorSpeed(1, p1);
    SetMotorSpeed(2, p2);
    SetMotorSpeed(3, p3);
    SetMotorSpeed(4, p4*0.7);
}

void Vector_Motion(float Vx, float Vy, float rot_V) {  
    robot.robot_heading += rot_V; // Update target heading based on input
    if(robot.robot_heading > 135){
      robot.robot_heading =  135;  
    }
    else if(robot.robot_heading < 45){
      robot.robot_heading = 45;  
    }
    Serial.printf("robot.robot_heading%f\n",robot.robot_heading);
    
    float e = robot.robot_heading - (90.0f - gyroData.heading);

    // Normalize error (-180 to 180)
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    float omega = (fabs(e) > robot.heading_threshold) ? (e * robot.P_factor) : 0;
    RobotIKControl(Vx, Vy, omega);
}

void FC_Vector_Motion(float WVx, float WVy, float target_heading) {
    // 1. Convert to Robot Frame
    float rad = (target_heading - 90.0f) * (M_PI / 180.0f);
    float cos_h = cos(rad);
    float sin_h = sin(rad);

    float robot_vx = WVx * cos_h + WVy * sin_h;
    float robot_vy = -WVx * sin_h + WVy * cos_h;

    // 2. Heading Correction
    float current_gyro_heading = 90.0f - gyroData.heading;
    float e = target_heading - current_gyro_heading;
    
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    float omega = (fabs(e) > robot.heading_threshold) ? (e * robot.P_factor) : 0;

    RobotIKControl(robot_vx, robot_vy, omega);
}

void readMotor() {
    // Process all available bytes to find a valid packet
    while (Serial8.available() >= 6) {
        if (Serial8.peek() != PROTOCAL_HEADER) {
            Serial8.read(); 
            continue;
        }

        uint8_t buf[6];
        Serial8.readBytes(buf, 6);

        if (buf[5] == PROTOCAL_END) {
            // 1. Update motor values
            mainCommand.vx = (float)((int8_t)buf[1]);
            mainCommand.vy = (float)((int8_t)buf[2]);
            mainCommand.rot_v = (float)((int8_t)buf[3]) / 100.0f;
            mainCommand.heading = (uint16_t)((int8_t)buf[4]) * 10;
            break; 
        }
    }
}

void readMotorandSendSensors() {
    // Process all available bytes to find a valid packet
    while (Serial8.available() >= 6) {
        if (Serial8.peek() != PROTOCAL_HEADER) {
            Serial8.read(); 
            continue;
        }

        uint8_t buf[6];
        Serial8.readBytes(buf, 6);

        if (buf[5] == PROTOCAL_END) {
            // 1. Update motor values
            mainCommand.vx = (float)((int8_t)buf[1]);
            mainCommand.vy = (float)((int8_t)buf[2]);
            mainCommand.rot_v = (float)((int8_t)buf[3]) / 100.0f;
            mainCommand.heading = (uint16_t)((int8_t)buf[4]) * 10;

            // 2. Prepare FRESH sensor data to send back immediately
            uint8_t res[7];
            res[0] = PROTOCAL_HEADER;
            float temp = fmod((90 - gyroData.heading + 360), 360);
            res[1] = (uint8_t)(temp / 10.0);
            res[2] = (uint8_t)(lineData.state & 0xFF);
            res[3] = (uint8_t)((lineData.state >> 8) & 0xFF);
            res[4] = (uint8_t)((lineData.state >> 16) & 0xFF);
            res[5] = (uint8_t)((lineData.state >> 24) & 0xFF);
            res[6] = PROTOCAL_END;

            // 3. Respond exactly once per received command
            Serial8.write(res, 7);
            break; 
        }
    }
}

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;

            ballData.angle = (int16_t)((buffer[3] << 8) | buffer[2]);
            ballData.dist  = (int16_t)((buffer[5] << 8) | buffer[4]);
            ballData.valid = buffer[6] == 0xFF;
            goalData.x     = (int16_t)((buffer[8] << 8) | buffer[7]);
            goalData.valid = buffer[9] == 0xFF;
            // usData.dist_ｆ       = (int16_t)((buffer[11] << 8) | buffer[10]);
            // usData.dist_ｌ       = (int16_t)((buffer[15] << 8) | buffer[14])
            // usData.dist_ｒ       = (int16_t)((buffer[16] << 8) | buffer[17]);
            usData.dist_b     = (int16_t)((buffer[13] << 8) | buffer[12]);
        }
    }
}