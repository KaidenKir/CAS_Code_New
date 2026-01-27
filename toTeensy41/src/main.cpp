#include <Arduino.h>
#include <servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);

  Serial.begin(9800);
}

void loop() {
  servo1.write(95);
  servo2.write(95);    
  servo3.write(95);
  servo4.write(95);
}