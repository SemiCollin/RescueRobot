#include <Wire.h>

const int SDA_Pin = 20;
const int SCL_Pin = 21;

int motor_byte = 0;

int E1 = 4; 
int M1A = 22;
int M1B = 24; 

int E2 = 3;
int M2A = 30;
int M2B = 32;

void setup() {
  // Start the serial communication at 9600 baud rate
  Serial.begin(9600);

  pinMode(E1, OUTPUT);
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);

  pinMode(E2, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);

  // Arduino joins I2C bus as slave with address 8
  // Wire.begin(0x8);

  // Call receiveEvent function when data received                
  // Wire.onReceive(receiveEvent);

  // Turn off 20k-50k ohm built-in pull-up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);

  // Print a message to the terminal indicating setup completion
  Serial.println("Setup complete. Waiting for I2C data...");
} 

// Function that executes whenever data is received from master device (e.g., Pi 5)
void receiveEvent(int howMany) {
  motor_byte = Wire.read();  // Receive byte as an integer
  Serial.println(motor_byte);

  // If received byte is 0x1, turn both motors on
  if (motor_byte == 0x1) {
    drive_Demo();
    Serial.println("Both motors ON");
  } 
  // If received byte is 0x0, turn both motors off
  else if (motor_byte == 0x0) {
    digitalWrite(M1A, LOW); // Direction, forwards
    digitalWrite(M1B, LOW); // Direction, forwards
    analogWrite(E1, 0); // Speed, 0-255

    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
    analogWrite(E2, 0);
    Serial.println("Both motors OFF");
  }
}

void loop() {
  // Continuously print a message to the terminal
  delay(100);
  // drive_Demo();
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, HIGH);
  digitalWrite(M2B, LOW);
  analogWrite(E1, 50);
  analogWrite(E2, 50);
}

void drive_Demo() {
    // Demo 1: Driving forward
  digitalWrite(M1A, LOW); // Direction, forwards
  digitalWrite(M1B, HIGH); // Direction, forwards
  analogWrite(E1, 255); // Speed, 0-255

  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
  analogWrite(E2, 255);

  delay(2000);

  // // Demo 2: Driving backwards
  // digitalWrite(M1A, HIGH);
  // digitalWrite(M1B, LOW);
  // analogWrite(E1, 255);

  // digitalWrite(M2A, HIGH);
  // digitalWrite(M2B, LOW);
  // analogWrite(E2, 255);

  // delay(2000);


  // // Demo 3: Turning
  // digitalWrite(M1A, LOW);
  // digitalWrite(M1B, HIGH);
  // analogWrite(E1, 255);

  // digitalWrite(M2A, HIGH);
  // digitalWrite(M2B, LOW);
  // analogWrite(E2, 255);
  
  // delay(2000);
}