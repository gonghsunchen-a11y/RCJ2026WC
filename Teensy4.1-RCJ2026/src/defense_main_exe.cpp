#include "main_core.h"

// 1. FUNCTION PROTOTYPES (Tells the compiler these exist later)
void c_mode_main_function();
void t_mode_main_function();

// --- Main Logic Functions ---

void t_mode_main_function() {
    Serial.println("Tmode Started");
    move_to_position(0, 100); // Move to (100, 100)
    //Missioin Complete, stop the robot
    while(1) {
        sendMotor(0, 0, 0, subCoreData.gyroHeading); // Stop the robot
    }
}

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while(1) {  
        update_all_sensor(); // Keep updating sensors!
        localizeRobot();
        Serial.printf("GX %d %d\n ", camData.goal_x, usData.dist_b);
        sendPacket();
    }
}

void setup() {
    main_core_init();
    uint8_t header = 0;

    #ifdef C_MODE
        header = C_MODE_HEADER;
        drawMessage("C Mode Locked");
    #elif defined(T_MODE)
        header = T_MODE_HEADER;
        drawMessage("T Mode Locked");
    #else
        header = C_MODE_HEADER;
        drawMessage("Default");
    #endif

    while(1) {
        Serial.println("Waiting for SubCore...");
        Serial8.write(header);
        // Wait a short moment for the sub-core to respond 
        if(Serial8.available() > 0) {
            if(Serial8.read() == PROTOCAL_ACT) {
                break; // Connection confirmed
            }
        }
    }
    Serial.println("SubCore exists");
}

void loop() {
    // Wait for UI to finish
    while(UI_Interface()) {
        ;
    }
    Serial8.read(); // Clear the MOVE_CMD from the buffer
    while(Serial8.read() != PROTOCAL_ACT) {
        Serial8.write(MOVE_CMD); // Tell SubCore to start sending commands
    }
    // The code only reaches here AFTER UI_Interface() returns false
    #ifdef C_MODE
        c_mode_main_function();
    #endif
    
    #ifdef T_MODE
        t_mode_main_function();
    #endif
}