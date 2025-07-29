#define S0 5
#define S1 4
#define S2 7
#define S3 6
#define OUT  8

#define E1 5
#define M1A 22
#define M1B 24

#define E2 6
#define M2A 30
#define M2B 32

void setup() {
  // Set the S0, S1, S2, S3 Pins as Output
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  //Set OUT_PIN as Input
  pinMode(OUT, INPUT);
  // Set Pulse Width scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Set motor pins for output
  pinMode(E1, OUTPUT);
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  // Initialize motor pin directions
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);

  // Enable UART for Debugging
  Serial.begin(9600);
}

int[] colour_read() {
  digitalWrite(S2)
}

void loop() {
  // put your main code here, to run repeatedly:

}
