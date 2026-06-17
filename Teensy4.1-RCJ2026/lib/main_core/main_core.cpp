#include "main_core.h"


// --- Sensor Data ---
CamData camData;
BallData ballData;
USSensor usData;
SubCoreData subCoreData;
RobotMovement robotMovement;

struct Point {
    float x;
    float y;
} RobotPos;


// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum RobotState { STATE_READY, STATE_CALIBRATING, STATE_SAVING };
RobotState currentState = STATE_READY;
// --- Kicker Constants ---
#define Charge_Pin 33 // Update to your actual pins
#define Kicker_Pin 32

void main_core_init() {
    // Initialize all Hardware Serials
    Serial.begin(115200);
    Serial2.begin(115200); 
    Serial3.begin(115200);
    Serial4.begin(115200); 
    Serial5.begin(921600);
    Serial6.begin(115200); 
    Serial7.begin(115200);
    Serial8.begin(921600);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Lock if OLED fails
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Pin Setups
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(Charge_Pin, OUTPUT);
    pinMode(Kicker_Pin, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);
    
    // Ensure kicker starts safe
    digitalWrite(Charge_Pin, LOW); // Start charging
    digitalWrite(Kicker_Pin, LOW);

    // Button
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);

    // Ultrasonic Sensor
    pinMode(front_us, INPUT_DISABLE);
    //pinMode(back_us, INPUT_DISABLE);
    //pinMode(left_us, INPUT_DISABLE);
    //pinMode(right_us, INPUT_DISABLE);
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}

void kicker_control(bool kick) {
    static uint32_t charge_start_time = 0;
    static bool is_charged = false;
    static bool is_charging = false;

    const uint32_t CHARGE_MS = 5000; 
    uint32_t now = millis();

    // 1. Kick Logic (Priority)
    if (kick && is_charged) {
        digitalWrite(Kicker_Pin, HIGH);
        delay(15); // Solenoid pulse
        digitalWrite(Kicker_Pin, LOW);
        
        is_charged = false; 
        is_charging = false;
        Serial.println("KICKED");
        return; // Exit to prevent immediate re-charge trigger in same frame
    }

    // 2. Charging State Machine
    if (!is_charged && !is_charging) {
        // Start a new charge cycle
        charge_start_time = now;
        is_charging = true;
        digitalWrite(Charge_Pin, HIGH);
        Serial.println("CHARGING...");
    }

    if (is_charging) {
        if (now - charge_start_time >= CHARGE_MS) {
            // Charge complete
            digitalWrite(Charge_Pin, LOW);
            is_charging = false;
            is_charged = true;
            Serial.println("READY TO KICK");
        }
    }
}

void readBallCam() {
    static uint16_t buffer[6] = {0};
    static uint16_t idx = 0;
    while(Serial4.available()){
        uint16_t b = Serial4.read();
        if(idx == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[idx++] = b;

        if(idx == 6){ //裝包 共6組
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
              ballData.angle = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
              ballData.dist  = (uint16_t)buffer[3] | ((uint16_t)buffer[4] << 8);
            
                if(ballData.angle != 65535 && ballData.dist != 65535){
                ballData.valid = true;
               }
               else{
                ballData.valid = false;
               }  //無球
            }
            else{
                ballData.valid = false;
            }  //無數據
            idx = 0;  // reset buffer
        }
    }
}


void readcamera(){
  static uint8_t buffer[20]; // 稍微開大一點點
  static uint8_t index = 0;
  while (Serial5.available()){
    uint8_t b = Serial5.read();
    
    if(index == 0 && b != 0xCC){
      continue;  // 等待開頭 0xCC
    }
    buffer[index++] = b;
    if (index == 18) {
      // 3. 檢查頭尾是否正確
      if (buffer[0] == 0xCC && buffer[17] == 0xEE) {
        // --- 解析球門 (Goal) ---
        int g_x = buffer[9] | (buffer[10] << 8);
        int g_y = buffer[11] | (buffer[12] << 8);
        int g_w = buffer[13] | (buffer[14] << 8);
        int g_h = buffer[15] | (buffer[16] << 8);

        camData.goal_x = g_x;
        camData.goal_y = g_y;
        camData.goal_w = g_w;
        camData.goal_h = g_h;
        camData.goal_valid = (g_x != 65535);
      }
      index = 0;  // reset buffer
    }
  }
}

void readussensor() {
    
    // 2. Timing control: Sequential reading (The "Better Break")
    static uint32_t lastReadTime = 0;
    static int sensorStep = 0;
    const int pingInterval = 50; // 35ms break between sensors

    // Static variables for low-pass filtering
    static float dist_b_f, dist_l_f, dist_r_f, dist_f_f;

    // Only process ONE sensor every 35ms to prevent acoustic interference
    if (millis() - lastReadTime >= pingInterval) {
        float rawValue = 0;
        switch(sensorStep) {
            case 0: // BACK
                rawValue = analogRead(back_us) * 520.0f / 1024.0f;
                dist_b_f = (alpha * rawValue) + ((1.0f - alpha) * dist_b_f);
                usData.dist_b = (int)dist_b_f;
                sensorStep = 1;
                break;
                
            case 1: // LEFT
                rawValue = analogRead(left_us) * 520.0f / 1024.0f;
                dist_l_f = (alpha * rawValue) + ((1.0f - alpha) * dist_l_f);
                usData.dist_l = (int)dist_l_f;
                sensorStep = 2;
                break;
                
            case 2: // RIGHT
                rawValue = analogRead(right_us) * 520.0f / 1024.0f;
                dist_r_f = (alpha * rawValue) + ((1.0f - alpha) * dist_r_f);
                usData.dist_r = (int)dist_r_f;
                sensorStep = 3;
                break;
                
            case 3: // FRONT
                rawValue = analogRead(front_us) * 520.0f / 1024.0f;
                dist_f_f = (alpha * rawValue) + ((1.0f - alpha) * dist_f_f);
                usData.dist_f = (int)dist_f_f;
                sensorStep = 0; // Reset to start
                break;
        }
        lastReadTime = millis();
    }
}

void update_all_sensor(){
    readBallCam();
    readcamera();
    readussensor();
}

void defense_localizeRobot() {
    // To do: localize using ultrasonic sensors and camera data
    if(usData.dist_b < 50){
        RobotPos.y = -(120 - usData.dist_b);
    }
    RobotPos.x = (camData.goal_x - 160) * 0.75;
}

void localizeRobot() {
    // X axis: ultrasonic default
    if(usData.dist_l < 50){
        RobotPos.x = -(90 - usData.dist_l);
    }
    else if(usData.dist_r < 50){
        RobotPos.x = 90 - usData.dist_r;
    }
    else{
        RobotPos.x = usData.dist_l - usData.dist_r;
    }
     
    // Y axis: ultrasonic near-wall estimates
    if(usData.dist_f < 50){
        RobotPos.y = 120 - usData.dist_f;
    }
    else if(usData.dist_b < 50){
        RobotPos.y = -(120 - usData.dist_b);
    }
    else
    // Camera refinement (only overwrites if goal visible and in valid range)
    if(camData.goal_valid){
        int16_t goal_height = camData.goal_h;
        int16_t goal_x = camData.goal_x;

        if(goal_height > Y_LOCALIZE_THRESHOLD_L && goal_height < Y_LOCALIZE_THRESHOLD_H){
            RobotPos.y = GOAL_LOCALIZATION_C1 / goal_height;
        }
    }
    /*
        if(goal_height > X_LOCALIZE_THRESHOLD_L && goal_height < X_LOCALIZE_THRESHOLD_H){
            RobotPos.x = goal_x - 160;
    }*/
}

bool move_to_position(int pos_x, int pos_y){
    while(1){
        update_all_sensor();
        localizeRobot();
        Serial.printf("RobotPos: (%.2f, %.2f)\n", RobotPos.x, RobotPos.y);
        int16_t dx = pos_x - RobotPos.x;
        int16_t dy = pos_y - RobotPos.y;
        if(abs(dx) < 10){
            sendMotor(0, 0, 0, subCoreData.gyroHeading); // Stop the robot
            break; // Target reached
        }
        robotMovement.vx = dx * 0.1 * 20;
        robotMovement.vy = 0;//to do
        if(robotMovement.vx != 0){
            if(robotMovement.vx > 30) robotMovement.vx = 30;
            if(robotMovement.vx < 15 && robotMovement.vx > 0) robotMovement.vx = 15;
            if(robotMovement.vx < -30) robotMovement.vx = -30;
            if(robotMovement.vx > -15 && robotMovement.vx < 0) robotMovement.vx = -15;
        }
        if(robotMovement.vy != 0){
            if(robotMovement.vy > 30) robotMovement.vy = 30;
            if(robotMovement.vy < 15 && robotMovement.vy > 0) robotMovement.vy = 15;
            if(robotMovement.vy < -30) robotMovement.vy = -30;
            if(robotMovement.vy > -15 && robotMovement.vy < 0) robotMovement.vy = -15;
        }
        //robotMovement.vy = dy / unit_v * 20;
        sendMotorAndGetSensors(robotMovement.vx, robotMovement.vy, 0, 90); // Move towards target, maintain current heading
        Serial.printf("vx: %.2f, vy: %.2f\n", robotMovement.vx, robotMovement.vy);
    }
    return true;
}



void sendMotor(float vx, float vy, float rot_v, int target_heading) {
    uint8_t data[6];
    data[0] = PROTOCAL_HEADER;
    data[1] = (int8_t)vx;
    data[2] = (int8_t)vy;
    // Scale up by 100 so we don't lose decimals (e.g., 0.5 rad/s becomes 50)
    data[3] = (int8_t)(rot_v * 100.0f);
    // Scale down by 10 to fit 0-360 into a single byte (0-36)
    data[4] = (int8_t)(target_heading / 10);
    data[5] = PROTOCAL_END;
    Serial8.write(data, sizeof(data));
}

void sendMotorAndGetSensors(float vx, float vy, float rot_v, int target_heading) {
    // 1. Send the command ONLY once.
    // To avoid flooding, you should only call this function at a set interval (e.g. 20ms)
    uint8_t data[6];
    data[0] = PROTOCAL_HEADER;
    data[1] = (int8_t)vx;
    data[2] = (int8_t)vy;
    data[3] = (int8_t)(rot_v * 100.0f);
    data[4] = (int8_t)(target_heading / 10);
    data[5] = PROTOCAL_END;
    
    Serial8.write(data, sizeof(data));

    // 2. Try to read the response from the PREVIOUS command.
    // If it's not here yet, we don't wait. We move on to keep the robot moving.
    if(Serial8.available() >= 7) { 
        if (Serial8.peek() == PROTOCAL_HEADER) {
            uint8_t buf[7];
            Serial8.readBytes(buf, 7);
            if (buf[6] == PROTOCAL_END) {                
                subCoreData.gyroHeading = buf[1] * 10;
                subCoreData.lineState = ((uint32_t)buf[5] << 24) | ((uint32_t)buf[4] << 16) | 
                                        ((uint32_t)buf[3] << 8) | buf[2];
            }
        } 
        else {
            Serial8.read(); // Clear one byte of junk if it's not the header
        }
    } 
}



bool UI_Interface(){
    readussensor();
    readBallCam();
    static uint32_t lastDisplayTime = 0;
    switch (currentState) {
        case STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {
                Serial8.write(LS_CAL_START); // Command to Sensor Board
                drawMessage("SCANNING");
                delay(500);
                currentState = STATE_CALIBRATING;
                break;
            }

            if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0);
                display.printf("ball dist: %d\n", ballData.dist);
                display.printf("ball angle: %d\n",ballData.angle);
                display.printf("us f: %d\n", usData.dist_f);
                display.printf("us l: %d\n", usData.dist_l);
                display.printf("us r: %d\n", usData.dist_r);
                display.printf("us b: %d\n", usData.dist_b);
                display.display();
                lastDisplayTime = millis();
            }
            if(digitalRead(BTN_UP) == LOW){
                display.clearDisplay();
                display.display();
                return false;
            }
            //offense

            //defense

            break;


        case STATE_CALIBRATING:
            if (digitalRead(BTN_ESC) == LOW) {
                Serial8.write(LS_CAL_END); // Command to Save
                drawMessage("SAVING...");
                delay(500);
                currentState = STATE_SAVING;
            }
            break;
        case STATE_SAVING:
            if(Serial8.available()){
                uint8_t c = Serial8.read();
                if(c == LS_CAL_END){
                    drawMessage("SAVED!");
                    delay(1000); // 讓 SAVED 停一下
                }
                // 回到初始狀態
                drawMessage("READY");
                delay(1000);
                display.clearDisplay();
                currentState = STATE_READY;
            }
            break;
    }
    return true;
}

void send_cam_and_pos_data() {
    // Send ball cam data
    uint8_t data[10];
    data[0] = PROTOCAL_HEADER;
    data[1] = (uint8_t)(ballData.valid);
    data[2] = (uint8_t)(ballData.angle & 0xFF); //lower bit
    data[3] = (uint8_t)((ballData.angle >> 8) & 0xFF); //higher bit
    data[4] = (uint8_t)(ballData.dist & 0xFF); //lower bit
    data[5] = (uint8_t)((ballData.dist >> 8) & 0xFF); //higher bit
    data[6] = (int8_t)(RobotPos.x);
    data[7] = (int8_t)(RobotPos.y);
    uint8_t checksum = 0;
    for(uint8_t i = 0; i < 8; i++){
        checksum += data[i];
    }
    data[8] = checksum % 256; // checksum
    data[9] = PROTOCAL_END;
    // pos.x pos.y
    // ball.valid ball.x ball.y
    Serial8.write(data, sizeof(data));
}

void sendPacket() {
    int16_t ball_angle = (int16_t)ballData.angle;
    int16_t ball_dist  = (int16_t)ballData.dist;
    uint8_t ball_valid = ballData.valid ? 0xFF : 0x00;
    int16_t goal_x     = (int16_t)camData.goal_x;
    uint8_t goal_valid = camData.goal_valid ? 0xFF : 0x00;
    int16_t us_f       = (int16_t)usData.dist_f;
    int16_t us_b       = (int16_t)usData.dist_b;
    int16_t us_l       = (int16_t)usData.dist_l;
    int16_t us_r       = (int16_t)usData.dist_r;

    uint8_t packet[21];
    packet[0]  = 0xAA;
    packet[1]  = 0xAA;
    packet[2]  = ball_angle & 0xFF;
    packet[3]  = (ball_angle >> 8) & 0xFF;
    packet[4]  = ball_dist  & 0xFF;
    packet[5]  = (ball_dist  >> 8) & 0xFF;
    packet[6]  = ball_valid;
    packet[7]  = goal_x & 0xFF;
    packet[8]  = (goal_x >> 8) & 0xFF;
    packet[9]  = goal_valid;
    packet[10] = us_f & 0xFF;
    packet[11] = (us_f >> 8) & 0xFF;
    packet[12] = us_b & 0xFF;
    packet[13] = (us_b >> 8) & 0xFF;
    packet[14] = us_l & 0xFF;
    packet[15] = (us_l >> 8) & 0xFF;
    packet[16] = us_r & 0xFF;
    packet[17] = (us_r >> 8) & 0xFF;

    uint8_t sum = 0;
    for (int i = 2; i <= 17; i++) sum += packet[i];
    packet[18] = sum;
    packet[19] = 0xEE;

    Serial8.write(packet, 20);
}
