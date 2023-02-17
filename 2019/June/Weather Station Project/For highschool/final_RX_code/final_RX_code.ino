#define SOFTWAREVERSION 004

#include <SoftwareSerial.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

#include "Crc16.h"

Crc16 crc;

#define LED 13
#include <RH_RF95.h>

int WindDirection;          // 풍향
float AveWindSpeed;         // 풍속
float RainMm;               // 강수량 (mm)
float TotalRain;            // 누적강수량
float MaxWindGust;          // 최대돌발풍속
float OutsideTemperature;   // 외부 온도
float OutsideHumidity;      // 외부 습도
float BatteryVoltage;       // 배터리 전압
float BatteryCurrent;       // 배터리 전류
float LoadCurrent;          // 전류 사용량
float SolarPanelVoltage;    // 태양광 발전 전압 
float SolarPanelCurrent;    // 태양광 발전 전류 (얜 회로 문제인지 0이 나올 때가 있음)
float AirPressure;          // 기압


byte rain[8] = {
  B00100,
  B00100,
  B01110,
  B01110,
  B11111,
  B11111,
  B11111,
  B01110
};

byte battery[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111
};

byte loading[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

SoftwareSerial SoftSerial(6, 7); // TX, RX
RH_RF95 rf95(SoftSerial);

byte buffer[70];
byte buflen = 0;


long consecutiveGoodMessages;
long lastGoodMessageID;
long goodMessages;
long badMessages;

void printBuffer(byte *buffer, int buflen) {
  int i;
  for (i = 0; i < buflen; i++)
  {
    Serial.print(F("i="));
    Serial.print(i);
    Serial.print(F(" | "));
    Serial.println(buffer[i], HEX);
  }

}


int convert2BytesToInt(byte *buffer, int bufferStart) {

  union u_tag {
    byte b[2];
    int fval;
  } u;

  u.b[0] = buffer[bufferStart];
  u.b[1] = buffer[bufferStart + 1];


  return u.fval;

}

long convert4BytesToLong(byte *buffer, int bufferStart) {

  union u_tag {
    byte b[4];
    long fval;
  } u;

  u.b[0] = buffer[bufferStart];
  u.b[1] = buffer[bufferStart + 1];
  u.b[2] = buffer[bufferStart + 2];
  u.b[3] = buffer[bufferStart + 3];

  return u.fval;

}


float convert4BytesToAM2315Float(byte *buffer, int bufferStart) {


  union u_tag {
    byte b[4];
    float fval;
  } u;


  u.b[0] = buffer[bufferStart + 3];
  u.b[1] = buffer[bufferStart + 2];
  u.b[2] = buffer[bufferStart + 1];
  u.b[3] = buffer[bufferStart + 0];
  Serial.print(F("fval="));
  Serial.println(u.fval);

  return u.fval;


}

float convert4BytesToFloat(byte *buffer, int bufferStart) {


  union u_tag {
    byte b[4];
    float fval;
  } u;


  u.b[0] = buffer[bufferStart + 0];
  u.b[1] = buffer[bufferStart + 1];
  u.b[2] = buffer[bufferStart + 2];
  u.b[3] = buffer[bufferStart + 3];


  return u.fval;

}



int interpretBuffer(byte *buffer, int buflen) {
  if (!((buffer[0] == 0xAB) && (buffer[1] == 0x66))) {
    return 1;
  }
  Serial.println(F("Start Bytes Found"));

  if (buflen != 67) {
    return 2;
  }
  unsigned short checksumValue;

  checksumValue = crc.XModemCrc(buffer, 0, 64);
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);

  if ((checksumValue >> 8) != buffer[65]) {
    return 3;
  }
  if ((checksumValue & 0xFF) != buffer[66]) {
    return 3;
  }


  Serial.println(F("Correct Buffer Length"));

  Serial.print(F("Protocol = "));
  Serial.println(buffer[2]);

  Serial.print(F("TimeSinceReboot(msec) = "));
  Serial.println(convert4BytesToLong(buffer, 3));

  Serial.print(F("Wind Direction = "));
  Serial.println(convert2BytesToInt(buffer, 7));
  WindDirection = convert2BytesToInt(buffer, 7);

  Serial.print(F("Average Wind Speed (KPH) = "));
  Serial.println(convert4BytesToFloat(buffer, 9));
  AveWindSpeed = convert4BytesToFloat(buffer, 9);

  Serial.print(F("Wind Clicks = "));
  Serial.println(convert4BytesToLong(buffer, 13));

  Serial.print(F("Total Rain Per Hour (mm) = "));
  Serial.println(convert4BytesToLong(buffer, 17));
  RainMm = convert4BytesToFloat(buffer, 17);
  
  Serial.print(F("Total Rain (mm) = "));
  Serial.println(convert4BytesToLong(buffer, 21));
  TotalRain = convert4BytesToFloat(buffer, 21);

  Serial.print(F("Max Wind Gust = "));
  Serial.println(convert4BytesToFloat(buffer, 25));
  MaxWindGust = convert4BytesToFloat(buffer, 25);

  Serial.print(F("Outside Temperature = "));
  Serial.println(convert4BytesToFloat(buffer, 29));
  OutsideTemperature = convert4BytesToFloat(buffer, 29);

  Serial.print(F("Outside Humidity = "));
  Serial.println(convert4BytesToFloat(buffer, 33));
  OutsideHumidity = convert4BytesToFloat(buffer, 33);

  Serial.print(F("Battery Voltage = "));
  Serial.println(convert4BytesToFloat(buffer, 37));
  BatteryVoltage = convert4BytesToFloat(buffer, 37);
  
  Serial.print(F("Battery Current = "));
  Serial.println(convert4BytesToFloat(buffer, 41));
  BatteryCurrent = convert4BytesToFloat(buffer, 41);
  
  Serial.print(F("Load Current = "));
  Serial.println(convert4BytesToFloat(buffer, 45));
  LoadCurrent = convert4BytesToFloat(buffer, 45);
  
  Serial.print(F("Solar Panel Voltage = "));
  Serial.println(convert4BytesToFloat(buffer, 49));
  SolarPanelVoltage = convert4BytesToFloat(buffer, 49);
  
  Serial.print(F("Solar Panel Current = "));
  Serial.println(convert4BytesToFloat(buffer, 53));
  SolarPanelCurrent = convert4BytesToFloat(buffer, 53);
  
  Serial.print(F("Air Pressure = "));
  Serial.println(convert4BytesToFloat(buffer, 57));
  AirPressure = convert4BytesToFloat(buffer, 57);

  Serial.print(F("Message ID = "));
  Serial.println(convert4BytesToLong(buffer, 61));


  Serial.print(F("Checksum High = 0x"));
  Serial.println(buffer[65], HEX);
  Serial.print(F("Checksum Low = 0x"));
  Serial.println(buffer[66], HEX);

  return 0;

}

void DisplayFirst() {
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);
    lcd.print(F("Wind Dierction:")); 
    if(WindDirection == 0) lcd.print(F("N"));
    else if(WindDirection == 45) lcd.print(F("NE"));
    else if(WindDirection == 90) lcd.print(F("E"));
    else if(WindDirection == 135) lcd.print(F("SE"));
    else if(WindDirection == 180) lcd.print(F("S"));
    else if(WindDirection == 225) lcd.print(F("SW"));
    else if(WindDirection == 270) lcd.print(F("W"));
    else if(WindDirection == 315) lcd.print(F("NW"));
    lcd.setCursor(0,1);
    lcd.print(F("Wind Speed:"));
    lcd.print(AveWindSpeed);
    lcd.print(F(" KPH"));
    lcd.setCursor(0,2);
    lcd.print(F("Wind Gust:"));
    lcd.print(MaxWindGust);
    lcd.print(F(" KPH"));
}
void DisplaySecond() {
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,0);
  lcd.print(F("Temperature:"));
  lcd.print(OutsideTemperature);
  lcd.print((char)223);
  lcd.print(F("C"));
  lcd.setCursor(0,1);
  delay(50);
  lcd.print(F("Humidity:"));
  lcd.print(OutsideHumidity);
  lcd.print(F("%"));
  lcd.write(1);
  lcd.setCursor(0,2);
  lcd.print(F("Pressure:"));
  lcd.print(AirPressure);
  lcd.print(F("hPa"));
}
void DisplayThird() {
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,0);
  lcd.print(F("Battery"));
  lcd.write(2);
  lcd.setCursor(0,1);
  lcd.print(F("Battery : "));
  lcd.print(BatteryVoltage);
  lcd.print(F(" V"));
  lcd.setCursor(0,2);
  lcd.print(F("Current : "));
  lcd.print(BatteryCurrent);
  lcd.print(F(" mA"));
  lcd.setCursor(0,3);
  lcd.print(F("Board Load:"));
  lcd.print(LoadCurrent);
  lcd.print(F(" mA"));
}
void DisplayForth() {
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,0);
  lcd.print(F("Solar Panel"));
  lcd.setCursor(0,1);
  lcd.print(F("Voltage : "));
  lcd.print(SolarPanelVoltage);
  lcd.print(F(" V"));
  lcd.setCursor(0,2);
  lcd.print(F("Current : "));
  lcd.print(SolarPanelCurrent);
  lcd.print(F(" mA"));
}
void DisplayFifth() {
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,0);
  lcd.print(F("Rain Total Check"));
  lcd.write(1);
  lcd.setCursor(0,1);
  lcd.print(F("Total Rain:"));
  lcd.print(TotalRain);
  lcd.print(F("mm"));
  lcd.setCursor(0,2);
  lcd.print(F("Hour Rain:"));
  lcd.print(RainMm);
  lcd.print(F("mm"));
}

void Loading() {
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,1);
  lcd.print(F("     LOADING...     "));
  lcd.setCursor(5,2);
}

void setup() {
  
  lcd.init();
  lcd.backlight();

  lcd.createChar(1, rain);
  lcd.createChar(2, battery);
  lcd.createChar(3, loading);

  Serial.begin(115200);
  Serial.println(F("-------Receive Started---------"));
  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  if (!rf95.init()) {
    Serial.println(F("init failed"));
    while (1);
  }


  rf95.setFrequency(434.0);

  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);

  consecutiveGoodMessages = 0;
  pinMode(LED, OUTPUT);

}


void blinkGood() {
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(100);

}
void blinkError() {
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}

void loop() {
  delay(1000);
  lcd.clear();
  Loading();
  lcd.write(3);
   if (rf95.waitAvailableTimeout(6000)) {
    
    delay(50);
    lcd.write(3);
    buflen = 75;
    byte messageLength;

    if (rf95.recv(buffer, &buflen)) {
      Serial.println(F("Message Received"));
    }

    messageLength = buflen;
    lcd.write(3);
    for (int i = 0; i < buflen; i++) {
       Serial.print(" ");
       if (buffer[i] < 16)
       {
         Serial.print(F("0"));
       }
       Serial.print(buffer[i], HEX);
      }

      Serial.println();
    lcd.write(3);
    int interpretResult = interpretBuffer(buffer, buflen);
    lcd.write(3);
    delay(50);
    lcd.write(3);
    delay(50);
    lcd.write(3);
    delay(50);
    lcd.write(3);
    delay(50);
    lcd.write(3);
    switch (interpretResult) {
      case 0:
        {
          int previousGoodMessageID = lastGoodMessageID;
          goodMessages++;

          lastGoodMessageID = convert4BytesToLong(buffer, 55);

          if (lastGoodMessageID == previousGoodMessageID + 1)
          {
            consecutiveGoodMessages++;
          }
          digitalWrite(LED, HIGH);
          delay(100);
          digitalWrite(LED, LOW);
          blinkGood();

        }
        break;
      case 1:
        Serial.println(F("Bad Message - No Start Bytes"));
        badMessages++;
        consecutiveGoodMessages = 0;
        blinkError();

        break;
      case 2:
        Serial.println(F("Bad Message - buffer length incorrect"));
        badMessages++;
        consecutiveGoodMessages = 0;
        blinkError();
        break;
      case 3:
        Serial.println(F("Bad Message - Bad Checksum"));
        badMessages++;
        consecutiveGoodMessages = 0;
        blinkError();


        break;
      default:

        //Serial.print(F("Bad Message - Unknown Return Code ="));
        Serial.println(interpretResult);
        badMessages++;
        consecutiveGoodMessages = 0;
        blinkError();
        break;
    }
    lcd.write(3);
    
    Serial.print(F("GM: "));
    Serial.print(goodMessages);
    Serial.print(F(" BM: "));
    Serial.println(badMessages);
    lcd.write(3);
    delay(100);

    buflen = 0;

    lcd.clear();
    DisplayFirst();
    delay(5000);
    lcd.clear();
    DisplayFifth();
    delay(5000);
    lcd.clear();
    DisplaySecond();
    delay(5000);
    lcd.clear();
    DisplayThird();
    delay(5000);
    lcd.clear();
    DisplayForth();
    delay(5000);

  }

  delay(100);


}
