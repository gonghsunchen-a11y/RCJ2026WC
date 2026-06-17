#include "main_core.h"


// --- Sensor Data ---
CamData camData;
BallData ballData;
USSensor usData;
RobotMovement robotMovement;

// Role Switching
int8_t role = 0; // 0: default, 1: defense, 2: offense

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
/*
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Lock if OLED fails
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
*/    
    // Pin Setups
    pinMode(Charge_Pin, OUTPUT);
    pinMode(Kicker_Pin, OUTPUT);
    
    // Ensure kicker starts safe
    digitalWrite(Charge_Pin, LOW); // Start charging
    digitalWrite(Kicker_Pin, LOW);

    // Button
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);
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

void get_ball_sensor() {
    uint8_t buffer[5];
    Serial6.write(0xDD);
    while(!Serial6.available());
    Serial6.readBytes(buffer,5);
    ballData.exist = false;
    if(buffer[0] != 0xCC) return;
    if (buffer[4] != 0xEE) return;
    if(buffer[0] == 0xCC && buffer[4] == 0xEE){
        uint16_t angle = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
        ballData.exist = true;
        ballData.angle = angle;
    }
    else{
        ballData.exist = false;
    }
}

//send own data to esp, if role = ATTACK
void ATTACK_send_data_to_esp(){
    Serial.write(0xBE); // Indicate BT data from ESP32

    Serial.write(0xEE); // End of packet
}

//read teammate data from esp, if role = DEFENSE
void DEFENSE_read_data_from_esp(){
    uint8_t buffer[5];
    while(!Serial6.available());//should use time out to avoid deadlock
    Serial6.readBytes(buffer,5);
    if(buffer[0] != 0xCC) return;
    if(buffer[4] != 0xEE) return;
    if(buffer[0] == 0xCC && buffer[4] == 0xEE){
        //teammate position x = 8 bits
        //teammate position y = 8 bits
        //ball angle(0,1,2,3 for 0 90 180 270) = 2 bits 
        //ball distance = 4 bits
        //ball possession = 1 bit
        //connect or disconnect = 1 bit
        int teammateX = buffer[1];
        int teammateY = buffer[2];
        
        // 4. Bit-Unpacking Section (Deconstructing buffer[3])
        // Use bitwise shift (>>) to move bits into place, and bitwise AND (&) to mask them.
        
        // Ball Angle: 2 bits (Bits 6 and 7) -> shift right by 6, mask with 0b11 (3)
        int ballAngleCode = (buffer[3] >> 6) & 0x03; 
        int ballAngle = ballAngleCode * 90; // Converts 0,1,2,3 directly into 0, 90, 180, 270 degrees!

        // Ball Distance: 4 bits (Bits 2, 3, 4, 5) -> shift right by 2, mask with 0b1111 (15)
        int ballDistance = (buffer[3] >> 2) & 0x0F; 

        // Ball Possession: 1 bit (Bit 1) -> shift right by 1, mask with 0b1 (1)
        bool ballPossession = (buffer[3] >> 1) & 0x01;

        // Connection Status: 1 bit (Bit 0) -> no shift needed, just mask the final bit
        bool isConnected = buffer[3] & 0x01;
    }
}

//send role switch if role = DEFENSE
void DEFENSE_send_role_switch(){
    Serial.write(0xBF); // Indicate Role Switch Command
    Serial.write(0xEE); // Indicate Role Switch Command
}

void update_all_sensor(){
    get_ball_sensor();
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


bool UI_Interface(){
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


