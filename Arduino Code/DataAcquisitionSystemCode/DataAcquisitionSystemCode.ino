#include <SD.h>
#include <SPI.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

File myFile;
char fileName[] = "simple.txt";
const int chipSelect = 10;
char charRead;
long timer = 0;
char pangram_1[] = "The five boxing wizards jump quickly";
char pangram_2[] = "Pack my box with five dozen liquor jugs";
char pangram_3[] = "The quick brown fox jumps over the lazy dog";


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
  
}

void loop() {
  mpu6050.update();
  if(millis() - timer > 1000){
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


/*void loop() 
{
  //Create a loop to read a command character from the keyboard
  //This will be 'r' for read, 'w' for write and 'd' for delete.

  if (Serial.available()) 
     {
      charRead = tolower(Serial.read());  //force ucase
      Serial.write(charRead); //write it back to Serial window
      Serial.println();
     }
     
  //get command from keyboard:
   switch(charRead)
   {
    case 'r':
        readFromFile();   //read
        break;
    case 'w':
        writeToFile(); //write to file
        break;
    case 'd':
        deleteFile();  //delete
        break;
   }
}*/
/*
void readFromFile()
{
  byte i=0; //counter
  char inputString[100]; //string to hold read string
  
  //now read it back and show on Serial monitor 
  // Check to see if the file exists:
  if (!SD.exists(fileName)) 
      Serial.println("simple.txt doesn't exist."); 
  Serial.println("Reading from simple.txt:");
  myFile = SD.open(fileName);

  while (myFile.available()) 
  {   
   char inputChar = myFile.read(); // Gets one byte from serial buffer
    if (inputChar == '\n') //end of line (or 10)
    {
      inputString[i] = 0;  //terminate the string correctly
      Serial.println(inputString);
      i=0;
    }
    else
    {
      inputString[i] = inputChar; // Store it
      i++; // Increment where to write next
      if(i> sizeof(inputString))
        {
        Serial.println("Incoming string longer than array allows");
        Serial.println(sizeof(inputString));
        while(1);
        }
    }
  }
 }*/

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
    
      /*Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
      Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
    
      Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
      Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
      Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
      
      Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
      Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
      Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
      Serial.println("=======================================================\n");

      Serial.println("Writing to simple.txt");
      myFile.println(pangram_1);
      myFile.println(pangram_2);
      myFile.println(pangram_3);*/
      myFile.close(); 
      Serial.println("Done");
    }
  else 
    Serial.println("Error opening simple.txt");
}

void deleteFile()
{
 //delete a file:
  if (SD.exists(fileName)) 
    {
    Serial.println("Removing simple.txt");
    SD.remove(fileName);
    Serial.println("Done");
   } 
}


