#ifndef PINS_H
#define PINS_H

//Comms
// #define SDA 20
// #define SCL 21

//Motors
#define E1 4
#define M1_F 24
#define M1_B 22
#define E2 3
#define M2_F 30
#define M2_B 32

extern int motor_pins[];

//Servos
#define SERVO_1 5
#define SERVO_2 6

extern int servo_pins[];

//Sensors
#define S0 9
#define S1 10

#define F_R_S2 48
#define F_R_S3 50
#define F_R_OUT 52

#define F_L_S2 49
#define F_L_S3 51
#define F_L_OUT 53

#define B_R_S2
#define B_R_S3
#define B_R_OUT

#define B_L_S2
#define B_L_S3
#define B_L_OUT

// int S2_S3_pins[] = {F_R_S2, F_R_S3, F_L_S2, F_L_S3, B_R_S2, B_R_S3, B_L_S2, B_L_S3};
// int out_pins[] = {F_R_OUT, F_L_OUT, B_R_OUT, B_L_OUT};
extern int S2_S3_pins[];
extern int out_pins[];

#endif