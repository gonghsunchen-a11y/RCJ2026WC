#include <Arduino.h>
void setup(){
    Serial.begin(115200);
    pinMode(A6, INPUT);
}

void loop(){
    Serial.println(analogRead(A6));
    delay(100);
}