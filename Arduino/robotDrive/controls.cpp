#include "controls.h"
#include "pins.h"

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Servo.h>

int motor_pins[] = {E1, M1_F, M1_B, E2, M2_F, M2_B};
int servo_pins[] = {SERVO_1};
int S2_S3_pins[] = {F_R_S2, F_R_S3, F_L_S2, F_L_S3, B_R_S2, B_R_S3, B_L_S2, B_L_S3};  // , B_R_S2, B_R_S3, B_L_S2, B_L_S3
int out_pins[] = {F_R_OUT, F_L_OUT, B_R_OUT, B_L_OUT};  // , B_R_OUT, B_L_OUT

unsigned long lastTime = 0;
float lastError = 0;
float integral = 0;

// PID Constants
float Kp = 3; // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 2; // Derivative gain

// Servo
Servo servo;
int angle = 0;

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
  servo.attach(SERVO_1);
  servo.write(angle); // CHNGE THIS SHIT
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
  Wire.begin(0x8);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
}

void drive_forward(int motor , int speed) {
  if (motor == E1) {
    digitalWrite(M1_F, HIGH);
    digitalWrite(M1_B, LOW);
  } else if (motor == E2) {
    digitalWrite(M2_F, HIGH);
    digitalWrite(M2_B, LOW);
  }
  analogWrite(motor, speed);
}

void drive_backwards(int motor , int speed) {
  if (motor == E1) {
    digitalWrite(M1_F , LOW);
    digitalWrite(M1_B , HIGH);
  } else if (motor == E2) {
    digitalWrite(M2_F , LOW);
    digitalWrite(M2_B , HIGH);
  }
  analogWrite(motor , speed);
}

void dynamic_drive(int motor , int speed) {
  if (speed > 0) {
    digitalWrite(M1_F, HIGH);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_F, HIGH);
    digitalWrite(M2_B, LOW);
  } else if (speed < 0) {
    digitalWrite(M1_F , LOW);
    digitalWrite(M1_B , HIGH);
    digitalWrite(M2_F , LOW);
    digitalWrite(M2_B , HIGH);
    speed = abs(speed);
  }
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
  // unsigned long currentTime = millis();
  // float deltaTime = (currentTime - lastTime) / 1000.0;

  // if (deltaTime == 0) return 0;

  // integral += error * deltaTime;
  // float derivative = (error - lastError) / deltaTime;
  // float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // lastError = error;
  // lastTime = currentTime;
  // return output;

  // Prevent integral windup
  if (abs(error) < 20) {  
    integral += error;
  } else {
    integral = 0;  // Reset integral if error is too high
  }

  float derivative = error - lastError;

  // Reset integral when error direction changes
  if (error * lastError < 0) {
    integral = 0;
  }

  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  return correction;
}

void motor_control(float correction , int speed_L , int speed_R) {
  int leftSpeed = speed_L - correction;
  int rightSpeed = speed_R + correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Serial.print("left: ");
  // Serial.println(leftSpeed);
  // Serial.print("right: ");
  // Serial.println(rightSpeed);

  drive_forward(E1 , leftSpeed);
  drive_forward(E2 , rightSpeed);
}

void target_detected() {
  Serial.println("Blue Target Detected");

  // // turn a little to face the figurine
  // drive_forward(E1, 0);
  // drive_forward(E2, 40);
  // delay(200);  // Wait for 200 milliseconds
  // drive_forward(E1, 0);
  // drive_forward(E2, 0);
  // delay(15000);
  

  // // drive forward a little if needed
  // drive_forward(E1, 58);
  // drive_forward(E2, 40);
  // delay(200);  // Wait for 200 milliseconds
  drive_forward(E1, 0);
  drive_forward(E2, 0);
  delay(1000);

  // close claw
  // scan from 0 to 180 degrees
  for(angle = 20; angle < 130; angle++)
  {
    servo.write(angle);
    delay(15);
  }
  // servo.write(130);

  // turn 45 degrees CCW
  drive_backwards(E1, 70);
  drive_forward(E2, 50);
  delay(2900);  // CHANGE THIS SHIT GUSS N CHECK

  // servo.detach();


  //? Stop the run 
  
  // turn left until red line detected on right sensor
  // while(true){
  //   drive_backwards(E1, 70);
  //   drive_forward(E2, 50);

  //   digitalWrite(F_R_S2, LOW);
  //   digitalWrite(F_R_S3, LOW);
  //   int right_sensor = pulseIn(F_R_OUT, HIGH, 100000);

  //   if(right_sensor >= 48 && right_sensor <= 64){
  //     drive_forward(E1, 0);
  //     drive_forward(E2, 0);
  //     break;
  //   }
    // robot should be aligned on the red tape and facing back towards the start of the track
  // }

  // back to PID driving
}