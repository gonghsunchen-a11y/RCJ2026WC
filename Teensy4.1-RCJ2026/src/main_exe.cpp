#include <main_core.h>
#define EAT_BALL_IR_PIN A16
unsigned long last_1000hz_tick = 0;
unsigned long last_50hz_tick = 0;

//#define DEFAULT_ROLE 1 // 1: offense, 2: defense
#define DEFAULT_ROLE 2 // 1: offense, 2: defense

#include <main_core.h>


//#define DEFAULT_ROLE 1 // 1: offense, 2: defense
#define DEFAULT_ROLE 2 // 1: offense, 2: defense

void setup() {
    delay(500); // Allow time for serial monitor to connect
    main_core_init();
    drawMessage("init");
    sendMaincoreData();
    //delay(500);
    while(1) {
        if(DEFAULT_ROLE == 1) {// 1: offense, 2: defense
            Serial8.write(0x0A);
        } else if(DEFAULT_ROLE == 2) {// 1: offense, 2: defense
            Serial8.write(0x0D);
        }
        // Wait a short moment for the sub-core to respond 
        if(Serial8.available() > 0) {
            if(Serial8.read() == PROTOCAL_ACT) {
                break; // Connection confirmed
            }
        }
    }
    while(UI_Interface()) {
        ;
    }
}



// 宣告微秒等級的計時器
void loop() {
    Serial8.read();
    Serial8.printf("MOVE_CMD"); // Tell SubCore to start sending commands
    while(Serial8.read() != PROTOCAL_ACT) {
        Serial8.write(MOVE_CMD); // Tell SubCore to start sending commands
        delay(1); // Short delay to allow SubCore to process
    }
    Serial.printf("MainCore Loop start\n");
    unsigned long last_1hz_tick = 0;
    unsigned long camra_read_time = millis(); // 新增：攝影機讀取計時器
    
    while(1){
        /* code */
        update_all_sensor();
        sensor_fusion();
        sendMaincoreData();
        if (millis() - camra_read_time >= 5000) {
            readTopMaix();
        }
        Serial.printf("pos_x: %d, pos_y: %d, ball_dist: %d, ball_angle: %d, ball_valid: %d, has_possession: %d, strategy: %d, role: %d\n",
            robotMonitor.pos_x,
            robotMonitor.pos_y,
            robotMonitor.ball_dist,
            robotMonitor.ball_angle,
            robotMonitor.ball_valid,
            robotMonitor.has_possession,
            robotMonitor.strategy,
            robotMonitor.role
        );
        if(digitalRead(EAT_BALL_IR_PIN) == 0){
            if(usData.dist_cm[US_FRONT] > 30){
                kicker_control(1);
            }
            else{
                kicker_control(0);
            }
        }
        if(millis() - last_1hz_tick >= 10) {
            last_1hz_tick = millis();
            if(digitalRead(COM_O1_PIN) == LOW){
                robotMonitor.role = 255;
                sendMaincoreData();
                break; // Exit the loop if COM_O1_PIN is LOW
            }
        }
    }
    robotMonitor.role = 0;
    Serial.printf("MainCore Loop end\n");
}