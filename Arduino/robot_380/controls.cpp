#include "controls.h"
#include "pins.h"

#include <Wire.h>
#include <Arduino.h>
#include <math.h>

int motor_pins[] = {E1, M1_F, M1_B, E2, M2_F, M2_B};
int servo_pins[] = {SERVO_1, SERVO_2};
int S2_S3_pins[] = {F_R_S2, F_R_S3, F_L_S2, F_L_S3};  // , B_R_S2, B_R_S3, B_L_S2, B_L_S3
int out_pins[] = {F_R_OUT, F_L_OUT};  // , B_R_OUT, B_L_OUT

unsigned long lastTime = 0;
float lastError = 0;
float integral = 0;

float Kp = 2;
float Ki = 0;
float Kd = 0;

void motor_setup() {
  // initialize pin mode for motor pins
  for (int i = 0; i < sizeof(motor_pins) / sizeof(motor_pins[0]); i++) {
    pinMode(motor_pins[i], OUTPUT);
  }

  // initialize forward direction
  digitalWrite(M1_F, HIGH);
  digitalWrite(M1_B, LOW);
  digitalWrite(M2_F, HIGH);
  digitalWrite(M2_B, LOW);
}

void servo_setup() {
  // initialize pin mode for servo pins
  pinMode(SERVO_1, OUTPUT);
  pinMode(SERVO_2, OUTPUT);

}

void clr_sensor_setup() {
  // initialize pin mode for shared S0 and S1 pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);

  // initialize pin mode for pins S2 and S3 for each sensor
  for (int i = 0; i < sizeof(S2_S3_pins) / sizeof(S2_S3_pins[0]); i++) {
    pinMode(S2_S3_pins[i], OUTPUT);
  }

  // initialize pin mode for out pins for each sensor
  for (int i = 0; i < sizeof(out_pins) / sizeof(out_pins[0]); i++) {
    pinMode(out_pins[i], INPUT);
  }

  /*
    LOW-HIGH: 20% freq scale, moderate sensitivity
    HIGH-LOW: 50% freq scale, high sensitivity
    HIGH-HIGH: 100% freq scal, max sensitivity
  */
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
}

void I2C_setup() {
  Wire.begin(0x08);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
}

void drive_forward(int motor , int speed) {
    digitalWrite(M1_F, HIGH);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_F, HIGH);
    digitalWrite(M2_B, LOW);

    analogWrite(motor, speed);
}

void drive_backward(int motor , int speed) {
  digitalWrite(M1_F , LOW);
  digitalWrite(M1_B , HIGH);
  digitalWrite(M2_F , LOW);
  digitalWrite(M2_B , HIGH);

  analogWrite(motor , speed);
}

void read_r(int &left, int &right) {
  digitalWrite(F_R_S2, LOW);
  digitalWrite(F_R_S3, LOW);
  digitalWrite(F_L_S2, LOW);
  digitalWrite(F_L_S3, LOW);

  left = pulseIn(F_L_OUT, HIGH, 100000);
  right = pulseIn(F_R_OUT, HIGH, 100000);
}

void read_b(int &left, int &right) {
  digitalWrite(F_R_S2, LOW);
  digitalWrite(F_R_S3, HIGH);
  digitalWrite(F_L_S2, LOW);
  digitalWrite(F_L_S3, HIGH);

  right = pulseIn(F_R_OUT, HIGH, 100000);
  left = pulseIn(F_L_OUT, HIGH, 100000);
}

void read_g(int &left, int &right) {
  digitalWrite(F_R_S2, HIGH);
  digitalWrite(F_R_S3, HIGH);
  digitalWrite(F_L_S2, HIGH);
  digitalWrite(F_L_S3, HIGH);

  right = pulseIn(F_R_OUT, HIGH, 100000);
  left = pulseIn(F_L_OUT, HIGH, 100000);
}

float calc_error(float nr1 , float nr2 , float ng1 , float ng2 , float nb1 , float nb2) {

  float r_diff_L = TARGET_R - nr1;
  float g_diff_L = TARGET_G - ng1;
  float b_diff_L = TARGET_B - nb1;
  
  float r_diff_R = TARGET_R - nr2;
  float g_diff_R = TARGET_G - ng2;
  float b_diff_R = TARGET_B - nb2;

  float error_L = sqrt(r_diff_L + g_diff_L + b_diff_L);
  float error_R = sqrt(r_diff_R + g_diff_R + b_diff_R);
  float combinedError = sqrt((pow(error_L , 2) + pow(error_R , 2) / 2));
  return combinedError;
}

float PID(float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime == 0) return 0;

  integral += error * deltaTime;
  float derivative = (error - lastError) / deltaTime;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error;
  lastTime = currentTime;
  return output;
}

void motor_control(float PID) {
  int leftSpeed = BASE_SPEED_L + PID;
  int rightSpeed = BASE_SPEED_R - PID;

  leftSpeed = constrain(leftSpeed, 30, 80);
  rightSpeed = constrain(rightSpeed, 30, 80);

  Serial.print("left: ");
  Serial.println(leftSpeed);
  Serial.print("right: ");
  Serial.println(rightSpeed);

  drive_forward(E1 , leftSpeed);
  drive_forward(E2 , rightSpeed);
}
