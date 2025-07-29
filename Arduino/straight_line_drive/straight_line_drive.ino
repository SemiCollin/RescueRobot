// Motor control pins
#define E1 4
#define M1_F 24
#define M1_B 22

#define E2 3
#define M2_F 30
#define M2_B 32

// Sensors
#define S0 9
#define S1 10

#define F_R_S2 48
#define F_R_S3 50
#define F_R_OUT 52

#define F_L_S2 49
#define F_L_S3 51
#define F_L_OUT 53

void setup() {
  Serial.begin(115200);  // Faster serial communication

  pinMode(F_R_S2, OUTPUT);
  pinMode(F_R_S3, OUTPUT);
  pinMode(F_L_S2, OUTPUT);
  pinMode(F_L_S3, OUTPUT);
  
  pinMode(F_R_OUT, INPUT);
  pinMode(F_L_OUT, INPUT);
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  
  pinMode(E1, OUTPUT);
  pinMode(M1_F, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2_F, OUTPUT);
  pinMode(M2_B, OUTPUT);
  
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {
  int r1, g1, b1, r2, g2, b2;
  process_rgb_values(r1, g1, b1, r2, g2, b2);

  // Normalize color values
  float total1 = r1 + g1 + b1;
  float total2 = r2 + g2 + b2;

  float nr1 = (total1 > 0) ? (r1 / total1) * 255.0 : 0;
  float ng1 = (total1 > 0) ? (g1 / total1) * 255.0 : 0;
  float nb1 = (total1 > 0) ? (b1 / total1) * 255.0 : 0;

  float nr2 = (total2 > 0) ? (r2 / total2) * 255.0 : 0;
  float ng2 = (total2 > 0) ? (g2 / total2) * 255.0 : 0;
  float nb2 = (total2 > 0) ? (b2 / total2) * 255.0 : 0;

  static int count = 0;
  if (count % 10 == 0) {  // Print every 10 loops to reduce lag
    Serial.print("LEFT SENSOR: r="); Serial.print(nr1);
    Serial.print(" g="); Serial.print(ng1);
    Serial.print(" b="); Serial.println(nb1);

    Serial.print("RIGHT SENSOR: r="); Serial.print(nr2);
    Serial.print(" g="); Serial.print(ng2);
    Serial.print(" b="); Serial.println(nb2);
  }
  count++;

  follow_line(nr1, ng1, nb1, nr2, ng2, nb2);
}

void process_rgb_values(int &r1, int &g1, int &b1, int &r2, int &g2, int &b2) {
  int timeout = 10000; // 10ms timeout for faster detection

  // Read Red
  digitalWrite(F_R_S2, LOW); digitalWrite(F_R_S3, LOW);
  digitalWrite(F_L_S2, LOW); digitalWrite(F_L_S3, LOW);
  r2 = pulseIn(F_R_OUT, HIGH, timeout);
  r1 = pulseIn(F_L_OUT, HIGH, timeout);

  // Read Green
  digitalWrite(F_R_S2, HIGH); digitalWrite(F_R_S3, HIGH);
  digitalWrite(F_L_S2, HIGH); digitalWrite(F_L_S3, HIGH);
  g2 = pulseIn(F_R_OUT, HIGH, timeout);
  g1 = pulseIn(F_L_OUT, HIGH, timeout);

  // Read Blue
  digitalWrite(F_R_S2, LOW); digitalWrite(F_R_S3, HIGH);
  digitalWrite(F_L_S2, LOW); digitalWrite(F_L_S3, HIGH);
  b2 = pulseIn(F_R_OUT, HIGH, timeout);
  b1 = pulseIn(F_L_OUT, HIGH, timeout);
}

void follow_line(float r1, float g1, float b1, float r2, float g2, float b2) {
  bool left_detected = (r1 >= 48 && r1 <= 64 && g1 >= 100 && g1 <= 117 && b1 >= 84 && b1 <= 97);
  bool right_detected = (r2 >= 48 && r2 <= 64 && g2 >= 100 && g2 <= 117 && b2 >= 84 && b2 <= 97);

  if ((left_detected && right_detected)) { 
    drive_forward();
  } else if (left_detected) { 
    Serial.println("GRADUAL LEFT TURN");
    gradual_turn(50, 140);  // Smooth left turn
  } else if (right_detected) { 
    Serial.println("GRADUAL RIGHT TURN");
    gradual_turn(140, 50);  // Smooth right turn
  } else {  
    stop_motors();
  }
}

void drive_forward() {
  analogWrite(E1, 140);
  analogWrite(E2, 140);
  digitalWrite(M1_F, HIGH);
  digitalWrite(M1_B, LOW);
  digitalWrite(M2_F, HIGH);
  digitalWrite(M2_B, LOW);
}

void gradual_turn(int speed_left, int speed_right) {
  analogWrite(E1, speed_left);
  analogWrite(E2, speed_right);
  digitalWrite(M1_F, HIGH);
  digitalWrite(M1_B, LOW);
  digitalWrite(M2_F, HIGH);
  digitalWrite(M2_B, LOW);
}

void stop_motors() {
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}