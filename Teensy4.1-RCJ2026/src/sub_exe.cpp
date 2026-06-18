#include "sub_core.h"
uint8_t role = 0; // 0: default, 1: defense, 2: offense
void setup() {
    sub_core_init();
    digitalWrite(LED_BUILTIN, HIGH); // Indicate setup complete
    while(1){
        Serial.println("Waiting for MainCore...");
        if(Serial8.available()){
            uint8_t temp = Serial8.read();
            if(temp == 0x0A){
                role = 1; // offense
                Serial8.write(PROTOCAL_ACT);
                break;
            }
            else if(temp == 0x0D){
                role = 2; // defense
                Serial8.write(PROTOCAL_ACT);
                break;
            }
            else{
                Serial.println("Invalid handshake, retrying...");
            }
        }
        else{
            Serial.println("Invalid handshake, retrying...");
        }
    }
    digitalWrite(LED_BUILTIN, LOW); // Indicate setup complete
}

void loop() {
    //update role with low frequency
    //read motor command high frequency
    if(role == 1){
        // Offense-specific logic
    }
    else if(role == 2){
        // Defense-specific logic
    }
}
