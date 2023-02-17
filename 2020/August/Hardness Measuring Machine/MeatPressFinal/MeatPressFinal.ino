#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
String dataString = "";

LiquidCrystal_I2C lcd(0x3F,16,2);
Servo servo1;

#define button 2
#define Time 10
#define Down 80
#define Up 100

void setup() {
  servo1.attach(3);
  servo1.write(90);
  lcd.init(); 
  lcd.backlight();
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(button, INPUT_PULLUP);

  lcd.clear();

  if (!SD.begin(chipSelect)) {

  }
}

float volt = 0.0;
float current = 0.0;
int dataNum = 0;

void SDCARD() {
   File dataFile = SD.open("MEAT.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    delay(100);
    dataFile.println();
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void checkCurrent() {
  lcd.clear();
  
  dataNum = 0;
  servo1.write(Down);
  
  while(dataNum < Time) {
    dataString = "";
    for(int j = 0; j <10; j++){
        for(int i = 0; i<10; i++){
          volt += analogRead(A0) * (5.0 / 1024);
          delay(10);
        }
      volt = volt / 10;
      current += (volt - 2.5) * (5 / 2);
    }
    current = current / 10;
    dataNum++;
    
    lcd.setCursor(0,0);
    lcd.print("Current");
    lcd.setCursor(11,0);
    lcd.print(dataNum);
    lcd.setCursor(13,0);
    lcd.print("/");
    lcd.print(Time);
    lcd.setCursor(0,1);
    lcd.print(current*1000);
    lcd.setCursor(7,1);
    lcd.print("mA");

    dataString += String(current*1000);
    dataString += " ";

    File dataFile = SD.open("MEAT.txt", FILE_WRITE);

    if (dataFile) {
      delay(100);
      dataFile.print(dataString);
      dataFile.close();
    } else {
      Serial.println("error opening datalog.txt");
    }

    delay(10);
  }
  SDCARD();
}

void backTo() {
  servo1. write(Up);
  lcd.setCursor(0,0);
  lcd.print("SET POS...");
}

void Complete(){
  File meat = SD.open("MEAT.txt");
  if(meat) {
    meat.close();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SD Writing");
    lcd.setCursor(0,1);
    lcd.print("SUCCESS");
  } else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SD Writing");
    lcd.setCursor(0,1);
    lcd.print("FAILED");
  }
  
}

void loop() {
  lcd.setCursor(0,0);
  lcd.print("READY...");
  lcd.setCursor(0,1);
  lcd.print("PRESS BUTTON");
  if(digitalRead(button) == LOW){
    checkCurrent();
    lcd.clear();
    backTo();
    delay(1000);
    Complete();
    delay(1000);
    lcd.clear();
  } else {
    servo1.write(90);
  }
}
