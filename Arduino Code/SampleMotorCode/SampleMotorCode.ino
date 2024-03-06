
/*
  L298N H-Bridge Demo
  l298-demo.ino
  Demonstrates operation of L298N Dual H-Bridge Motor Driver
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
 
// Motor Connections (ENA & ENB must use PWM pins)
#define IN1 9
#define IN2 8
#define ENA 10

 
void motorAccel() {
  for (int i = 0; i < 256; i++) {
    analogWrite(ENA, i);
    delay(20);
  }
}
 
void motorDecel() {
  for (int i = 255; i >= 0; --i) {
    analogWrite(ENA, i);
    delay(20);
  }
}
 
void setup() {
 
  // Set motor connections as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

 
  // Start with motors off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

}
 
void loop() {
 
  // Set motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);


 
  // Accelerate & decelerate both motors forward
  motorAccel();
  motorDecel(); 
 
  delay(500);
 
  // Set motors reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);


 
  // Accelerate & decelerate both motors backward
  motorAccel();
  motorDecel();
 
  delay(500);
 
  // Set motors in opposite directions (full speed)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, 255);

  delay(500);
 
  analogWrite(ENA, 0);

  delay(500);
}