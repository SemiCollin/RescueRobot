#include "controls.h"
#include "pins.h"

#include <Wire.h>

int r1, g1, b1, r2, g2, b2;
byte pi_control;
bool control_bits[8];
bool lego_retrieved = false;
int speed_increase = 0;

void setup() {
  Serial.begin(9600);
  motor_setup();
  clr_sensor_setup();
  servo_setup();
  I2C_setup();

  Wire.onReceive(receiveEvent);
  Serial.println("\nI2C Initialized. Waiting for Input...");
  // while(true) {
  //   if (pi_control & (1 << START)) {
  //     Serial.println("Program Started");
  //     break;
  //   }
  //   delay(100);
  // }
}

void loop() {
  process_rgb_values(r1, g1, b1, r2, g2, b2);
  
  float total1 = r1 + g1 + b1;
  float total2 = r2 + g2 + b2;

  float nr1 = (total1 > 0) ? (r1 / total1) * 255.0 : 0;
  float nr2 = (total2 > 0) ? (r2 / total2) * 255.0 : 0;
  float nb1 = (total1 > 0) ? (b1 / total1) * 255.0 : 0;
  float nb2 = (total2 > 0) ? (b2 / total2) * 255.0 : 0;
  float ng1 = (total1 > 0) ? (g1 / total1) * 255.0 : 0;
  float ng2 = (total2 > 0) ? (g2 / total2) * 255.0 : 0;

  if (pi_control & (1 << RED)) Serial.println("RED LINE");
  if (pi_control & (1 << GREEN)) Serial.println("GREEN LINE");
  if (pi_control & (1 << BLUE)) Serial.println("BLUE LINE");

  // might need to double check blue range
  if ((nr1 >= 95 && nr1 <= 110 && ng1 >= 85 && ng2 <=95 && nb1 >= 55 && nb2 <= 64)) { // &&  pi_control & (1 << BLUE)
    // Serial.println("blue target");
    target_detected();
    lego_retrieved = true;
  }

  if (lego_retrieved) {
    speed_increase = 7;
  }

  float error = nr2 - nr1;

  float correction = PID(error);
  motor_control(correction , BASE_SPEED_L + speed_increase , BASE_SPEED_R + speed_increase);

  // Serial.println("Left");
  // Serial.println(nr1);
  // Serial.println(ng1);
  // Serial.println(nb1);
  // Serial.println("Right");
  // Serial.println(nr2);
  // Serial.println(ng2);
  // Serial.println(nb2);
}

void receiveEvent(int control) {
  pi_control = Wire.read();
  for (int i = 8; i > 0; i--) {
    control_bits[i] = (pi_control >> i) & 1;
  }
  // delay(500);
  // for (int i = 0; i < 8; i++) {
  //   Serial.print(control_bits[i]);
  // }
  // Serial.println();
}

void process_rgb_values(int &r1, int &g1, int &b1, int &r2, int &g2, int &b2) {
  int timeout = 10000;
  digitalWrite(F_R_S2, LOW); digitalWrite(F_R_S3, LOW);
  digitalWrite(F_L_S2, LOW); digitalWrite(F_L_S3, LOW);
  r2 = pulseIn(F_R_OUT, HIGH, timeout);
  r1 = pulseIn(F_L_OUT, HIGH, timeout);
  
  digitalWrite(F_R_S2, HIGH); digitalWrite(F_R_S3, HIGH);
  digitalWrite(F_L_S2, HIGH); digitalWrite(F_L_S3, HIGH);
  g2 = pulseIn(F_R_OUT, HIGH, timeout);
  g1 = pulseIn(F_L_OUT, HIGH, timeout);
  
  digitalWrite(F_R_S2, LOW); digitalWrite(F_R_S3, HIGH);
  digitalWrite(F_L_S2, LOW); digitalWrite(F_L_S3, HIGH);
  b2 = pulseIn(F_R_OUT, HIGH, timeout);
  b1 = pulseIn(F_L_OUT, HIGH, timeout);
}
