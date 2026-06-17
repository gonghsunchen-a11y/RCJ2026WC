#include "main_core.h"

void setup() {
    main_core_init();
}

void loop() {
    digitalWrite(32, LOW);
    digitalWrite(33, HIGH); // Start charging
    delay(5000); // Keep the kicker on for 5 seconds
    digitalWrite(33, LOW); // Start charging
    digitalWrite(32, HIGH); // Trigger the kick
    Serial.println("KICKED");
    delay(50);
    digitalWrite(32, LOW); // Trigger the kick
    Serial.println("KICKED END");
    delay(5000); // Wait before the next kick
    /*
    for (int signal = 1450; signal <= 1500; signal += 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(2000);  // Slow enough for ESC to recognize change
        //Stop at 1460
    }
    for (int signal = 1550; signal >= 1500; signal -= 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(2000);  // Slow enough for ESC to recognize change
        //Stop at 1500
    }*/
}