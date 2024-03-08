
/*
  L298N H-Bridge Demo
  l298-demo.ino
  Demonstrates operation of L298N Dual H-Bridge Motor Driver
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
#include <SD.h>
#include <SPI.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// Motor Connections (ENA & ENB must use PWM pins)
#define IN1 9
#define IN2 8
#define ENA 11

File myFile;
char fileName[] = "simple.txt";
const int chipSelect = 10;
char charRead;
long timer = 0;

void motorAccel() {
  for (int i = 0; i < 256; i++) {
    analogWrite(ENA, i);
    delay(20);
    mpu6050.update();
    if (Serial.available()) 
    {
      charRead = tolower(Serial.read());  //force ucase
      Serial.write(charRead); //write it back to Serial window
      Serial.println();
    }
    writeToFile();
    timer = millis();
  }
}
 
void motorDecel() {
  for (int i = 255; i >= 0; --i) {
    analogWrite(ENA, i);
    delay(20);
    mpu6050.update();
    if (Serial.available()) 
    {
      charRead = tolower(Serial.read());  //force ucase
      Serial.write(charRead); //write it back to Serial window
      Serial.println();
    }
    writeToFile();
    timer = millis();
  }
}

void writeToFile()
{
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) // it opened OK
    {
      Serial.println("=======================================================");
      myFile.print("temp : ");myFile.println(mpu6050.getTemp());
      myFile.print("accX : ");myFile.print(mpu6050.getAccX());
      myFile.print("\taccY : ");myFile.print(mpu6050.getAccY());
      myFile.print("\taccZ : ");myFile.println(mpu6050.getAccZ());
    
      myFile.print("gyroX : ");myFile.print(mpu6050.getGyroX());
      myFile.print("\tgyroY : ");myFile.print(mpu6050.getGyroY());
      myFile.print("\tgyroZ : ");myFile.println(mpu6050.getGyroZ());
      myFile.print("\tgyroZ : ");myFile.println(mpu6050.getGyroZ());
      Serial.println("=======================================================");
      myFile.close(); 
      Serial.println("Done");
    }
  else 
    Serial.println("Error opening simple.txt");
}


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("Simple SD Card Demo");

   if (SD.begin(chipSelect))
    {
      Serial.println("SD card is present & ready");
    } 
    else
    {
      Serial.println("SD card missing or failure");
      while(1);  //wait here forever
    }
    Serial.println("Enter w for write, r for read or d for delete");
  
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
 
  analogWrite(ENA, 255);

  delay(500);
 
  analogWrite(ENA, 0);

  delay(500);
 
}