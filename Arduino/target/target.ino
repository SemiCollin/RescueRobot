/*
  Red tape readings:
  r = ~47 g = ~199 b = ~95
  Green tape readings:
  r = ~65 g = ~55 b = ~55
  Tile readings:
  r = ~ 36-45 g = ~ 50-55 b = ~ 50-55
  Blue tape readings:
  r = ~50 g = ~60 b = ~35
*/

void setup() {
  // put your setup code here, to run once:
  /*
    Colour sensors initialized for reading values
    2 front sensors for target sensing
    8 pins

    S2_L
    S3_L
    OUT_L

    S2_R
    S3_R
    OUT_R

    Servo pin initialized to actuate the claw

    SERVO_PIN

    Blue target already detected flag in case the sensors detect the target again after turning
    blue_detected = 0;
  */
}

void loop() {
  // put your main code here, to run repeatedly:

  /*
    if (redLineDetected) {
      delay(1000);

      PID drive algorithm
      
    }

    if (OUT_R || OUT_L == rgb(blue_tape) && !blue_detected) {
      blue_target_detected();
      delay(1000);
      close_claw();

      u_turn();

      blue_detected = 1;
    }

    after target detection algorithm, return to running the PID drive function
  */
}

void blue_target_detected() {
  /*
    hardcode positioning to face the figure directly (within the given range)

    hardcode motor to drive forward a fixed distance to reach the figure
  */
}

void u_turn() {
  /*
    360 turn on the spot to reposition robot back towards the red line
    sensor to detect red tape again dependant on turn direction to center robot
    turning right = left sensor

    while(OUT_L != rgb(red_tape)) {
      digitalWrite(left_motor, forward);
      digitalWrite(right_motor, backward);
    }

    tape should be between L/R colour sensors again
  */
}

void close_claw() {
  /*
    Consider:
      Robot drives with claw open, run close_claw() after blue_target_detected() finishes in loop()

      Robot drives with claw closed, run open/close_claw() wihtin blue_target_detected()

    Drive servo motor to tested position to enclose around lego figure
    digitalWrite(SERVO_PIN, position);
  */
}