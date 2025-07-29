#include "controls.h"
#include "pins.h"

#include <Wire.h>
// #include <Arduino.h>

int r1, r2, g1, g2, b1, b2;
byte pi_control;
bool control_bits[8];

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    // motor_setup();
    // clr_sensor_setup();
    I2C_setup();
    Wire.onReceive(receiveEvent);
    Serial.println("Initialized I2C");
}

void receiveEvent(int control) {
  pi_control = Wire.read();
  for (int i = 0; i < 8; i++) {
    control_bits[i] = (pi_control >> i) & 1;
  }
  for (int i = 0; i < 5; i++) {
    Serial.println(control_bits[i]);
  }
}

void loop() {
    // // put your main code here, to run repeatedly:
    // read_r(r1, r2);
    // // delay(200);
    // read_g(g1, g2);
    // // delay(200);
    // read_b(b1, b2);
    // // delay(200);

    // // Normalize color values
    // float total1 = r1 + g1 + b1;
    // float total2 = r2 + g2 + b2;

    // float nr1 = (total1 > 0) ? (r1 / total1) * 255.0 : 0;
    // float ng1 = (total1 > 0) ? (g1 / total1) * 255.0 : 0;
    // float nb1 = (total1 > 0) ? (b1 / total1) * 255.0 : 0;

    // float nr2 = (total2 > 0) ? (r2 / total2) * 255.0 : 0;
    // float ng2 = (total2 > 0) ? (g2 / total2) * 255.0 : 0;
    // float nb2 = (total2 > 0) ? (b2 / total2) * 255.0 : 0;

    // float combinedError = calc_error(nr1 , nr2 , ng1 , ng2 , nb1 , nb2);
    // float control_PID = PID(combinedError);

    // Serial.print("left sensor: ");
    // Serial.println(r1);
    // Serial.println(nr1);
    // Serial.print("right sensor: ");
    // Serial.println(r2);
    // Serial.println(nr2);

    // Serial.println(combinedError);
    // Serial.println(control_PID);
    // motor_control(control_PID);
    // // delay(500);
  delay(100);

  if (control_bits[4]) Serial.println("Program Started");
  
}
