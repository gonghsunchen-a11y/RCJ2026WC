#include "sub_core.h"

void setup() {
    sub_core_init();
}

void loop() {
    Serial.println("Motor Test");
    for(uint8_t i = 0; i < 4; i++) {
        SetMotorSpeed(i + 1, 30);
        delay(2000);
        SetMotorSpeed(i + 1, -30);
        delay(2000);
    }
    MotorStop();
    delay(5000);
    RobotIKControl(0, 20, 0);
    delay(5000);
    RobotIKControl(0, -20, 0);
    delay(5000);
    RobotIKControl(-20, 0, 0);
    delay(5000);
    RobotIKControl(20, 0, 0);
    delay(5000);
}