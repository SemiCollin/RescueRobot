#ifndef CONTROLS_H
#define CONTROLS_H

#define NUM_SENSORS 2
#define BASE_SPEED_L 120
#define BASE_SPEED_R 100

#define TARGET_R 56
#define TARGET_G 108
#define TARGET_B 90

/* Setup Functions: purely intializing pinModes and on-start conditions, meant for void setup()
  --------------------------------------------------------------------------------------------
  motor_setup()       -- DC motors for driving
  servo_setup()       -- servo motors for claw + arm
  clr_sensor_setup()  -- 4 colour sensors on the robot base to detect the line + PID
  I2C_setup()         -- communication setup between arduino and rpi5
*/
void motor_setup();
void servo_setup();
void clr_sensor_setup();
void I2C_setup();

/* Controls Functions: general control commands for the robot
  --------------------------------------------------------------------------------------------
  drive_forward()     -- drive forward at given speed
  drive_backwards()   -- drive backwards at given speed
  turn()              -- turn in a direction
  u_turn()            -- u-turn drive to return to the start from target (hard coded)
  read_r()            -- return array of red light readings from each sensor
  read_b()            -- return array of blue light readings from each sensor
  read_g()            -- return array of green light readings from each sensor
  target_detected()   -- positioning routine once target with figure is detected
  actuate_claw()      -- drive claw mechanism
  calc_error()        -- calculate error for PID control
  PID_control         -- PID control for drive
*/
void drive_forward(int motor , int speed);
void drive_backwards(int motor , int speed);
void turn(char dir);
void u_turn();

void read_r(int &left, int &right);
void read_b(int &left, int &right);
void read_g(int &left, int &right);

void target_detected();
void actuate_claw();

float calc_error(float nr1 , float nr2 , float ng1 , float ng2 , float nb1 , float nb2);
float PID(float error);
void motor_control(float PID);

/* Testing Functions: functionality testing for components on the robot, ensuring no wiring and hardware issues.
  --------------------------------------------------------------------------------------------
  drive_demo()        -- general drive routine to test forward/backward/turning
  clr_read()          -- read values from one colour sensor to the serial monitor
*/
void drive_demo();

#endif