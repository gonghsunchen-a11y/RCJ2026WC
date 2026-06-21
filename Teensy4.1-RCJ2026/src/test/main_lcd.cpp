#include <main_core.h>


//#define DEFAULT_ROLE 1 // 1: offense, 2: defense
#define DEFAULT_ROLE 2 // 1: offense, 2: defense

void setup() {
    delay(3000); // Allow time for serial monitor to connect
    main_core_init();
    drawMessage("init");
    delay(500);
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
    Serial8.read(); // Clear the MOVE_CMD from the buffer
    while(Serial8.read() != PROTOCAL_ACT) {
        Serial8.write(MOVE_CMD); // Tell SubCore to start sending commands
    }

}

void loop() {
    ballsensor();
    if(DEFAULT_ROLE == 1) {
        ;// Handle offense logic
    } 
    else if(DEFAULT_ROLE == 2) {
        sendMaincoreData(); // Handle defense logic
        readMaix(); // Update Maix data for defense
        Serial.print("rooboo Data - X: ");
        Serial.print(robotMonitor.pos_x);
        Serial.print("  Y: ");
        Serial.println(robotMonitor.pos_y);
        //Serial.printf("Sent ball data - Valid: %d, Angle: %d, Dist: %d\n", robotMonitor.ball_valid, robotMonitor.ball_angle, robotMonitor.ball_dist);
    }
}