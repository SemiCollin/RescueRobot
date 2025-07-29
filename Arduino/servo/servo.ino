#include <Servo.h>

Servo servo;
int angle = 10;

void setup() {
  servo.attach(2);
  servo.write(angle);
}


void loop() 
{ 
  servo.write(130);
  servo.detach();
}