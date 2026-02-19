#include <Arduino.h>
#include <PWMServo.h>

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;

void setup() {
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);

  Serial.begin(9600);
}

void loop() {
  servo1.write(0);
  servo2.write(0);    
  servo3.write(0);
  servo4.write(0);

  delay(500);

  servo1.write(180);
  servo2.write(180);    
  servo3.write(180);
  servo4.write(180);

  delay(500);
}