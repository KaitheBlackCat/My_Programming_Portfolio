#define TXDEBUG 
#include <JeeLib.h> 

#define LED 13

#define SOFTWAREVERSION 8 

#define WIRELESSID 3

#define SLEEPCYCLE 29000

#include "Crc16.h"

Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
} //절전모드

#include <SoftwareSerial.h>
#include <RH_RF95.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"

SDL_Arduino_INA3221 SunAirPlus;

#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3

#define ENABLE_RADIO 5

#define WATCHDOG_1 8
#define WATCHDOG_2 9

SoftwareSerial SoftSerial(6, 7); // TX, RX  LoRa 통신을 위함.

RH_RF95 rf95(SoftSerial);

unsigned long MessageCount = 0;

unsigned long badAM2315Reads = 0;

#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>
#include "DS3232RTC.h"

#include <Wire.h>

#include <Adafruit_AM2315.h>

typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  RAIN_INTERRUPT,
  ANEMOMETER_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;

bool SunAirPlus_Present;
bool DS3231_Present;

byte byteBuffer[200];

// State Variables
byte Protocol;              // 프로토콜
long TimeStamp;             // 시간 문자열
int WindDirection;          // 풍향
float AveWindSpeed;         // 풍속
long TotalRainClicks;       // 총 강수계 클릭수
float RainMm;               // 강수량 (mm)
float MaxWindGust;          // 최대돌발풍속
float OutsideTemperature;   // 외부 온도
float OutsideHumidity;      // 외부 습도
float BatteryVoltage;       // 배터리 전압
float BatteryCurrent;       // 배터리 전류
float LoadVoltage;          // 전압 사용량
float LoadCurrent;          // 전류 사용량
float SolarPanelVoltage;    // 태양광 발전 전압 
float SolarPanelCurrent;    // 태양광 발전 전류 (얜 회로 문제인지 0이 나올 때가 있음)
float AirPressure;          // 기압
int protocolBufferCount;    // 프로토콜 문자열 총 길이

wakestate wakeState;

bool Alarm_State_1;
bool Alarm_State_2;
bool ignore_anemometer_interrupt;
long nextSleepLength;

long lastRainTime;
long windClicks;
long  lastWindTime;
long shortestWindTime;

const double bucketAmount = 0.2794;
float dailyRain = 0.0;
float hourlyRain = 0.0;
float dailyRain_till_LastHour = 0.0;
bool first = false;

#include "WeatherRack.h"

Adafruit_AM2315 am2315;

int convert4ByteLongVariables(int bufferCount, long myVariable) {

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (int i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable) {
  
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (int i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable) {
  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;

  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;
  return bufferCount;
}

int convert1ByteVariables(int bufferCount, int myVariable) {
  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;
}

int checkSum(int bufferCount) {
  unsigned short checksumValue;
  checksumValue = crc.XModemCrc(byteBuffer, 0, 64);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}

int buildProtocolString() {

  int bufferCount;


  bufferCount = 0;

  byteBuffer[bufferCount] = 0xAB;//0
  bufferCount++;
  byteBuffer[bufferCount] = 0x66;//1
  bufferCount++;
  bufferCount = convert1ByteVariables(bufferCount, Protocol);//2
  Serial.println(bufferCount);
  bufferCount = convert4ByteLongVariables(bufferCount, TimeStamp / 100);//3
  Serial.println(bufferCount);
  bufferCount = convert2ByteVariables(bufferCount, WindDirection);//7
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, AveWindSpeed);//9
  Serial.println(bufferCount);
  bufferCount = convert4ByteLongVariables(bufferCount, windClicks);//13
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, RainMm);//17
  Serial.print("Sampling : ");
  Serial.println(RainMm);
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, dailyRain);//21
  Serial.print("Sampling : ");
  Serial.println(dailyRain);
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, MaxWindGust);//25
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, OutsideTemperature);//29
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, OutsideHumidity);//33
  Serial.println(bufferCount);

  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);//37
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);//41
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);//45
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);//49
  Serial.println(bufferCount);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);//53
  Serial.println(bufferCount);

  bufferCount = convert4ByteFloatVariables(bufferCount, AirPressure);//57
  Serial.println(bufferCount);
  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);//61
  Serial.println(bufferCount);
  protocolBufferCount = bufferCount + 2;
  bufferCount = checkSum(bufferCount);
  Serial.println(bufferCount);

  return bufferCount;

}

void printStringBuffer() {
  int bufferLength;

  bufferLength = protocolBufferCount;
  int i;
  for (i = 0; i < bufferLength; i++)
  {
    Serial.print(F("i="));
    Serial.print(i);
    Serial.print(F(" | "));
    Serial.println(byteBuffer[i], HEX);
  }

}

void printDigits(int digits) {
  // utility function for digital clock display: prints an leading 0

  if (digits < 10)
    Serial.print(F("0"));
  Serial.print(digits);
}

void digitalClockDisplay() {
  tmElements_t tm;
  RTC.read(tm);

  Serial.print(tm.Year + 1970);
  Serial.print(F(". "));
  Serial.print(tm.Month);
  Serial.print(F(". "));
  Serial.print(tm.Day);
  Serial.print(F(" "));
  printDigits(tm.Hour);
  Serial.print(F(":"));
  printDigits(tm.Minute);
  Serial.print(F(":"));
  printDigits(tm.Second);
  Serial.println();
}

void return2Digits(char returnString[], char *buffer2, int digits) {
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);
}

void set32KHz(bool setValue) {
  uint8_t s = RTC.readRTC(RTC_STATUS);
  
  if (setValue == true) {
    s = s | (1 << EN32KHZ);
    RTC.writeRTC(RTC_STATUS, s);
  }
  else {
    uint8_t flag;
    flag =  ~(1 << EN32KHZ);
    s = s & flag;
    RTC.writeRTC(RTC_STATUS, s);
  }
}

void ResetWatchdog() {
    digitalWrite(WATCHDOG_1, LOW);
    delay(200);
    digitalWrite(WATCHDOG_1, HIGH);

#if defined(TXDEBUG)
    Serial.println(F("Watchdog1 Reset - Patted the Dog"));
#endif
}



void setup() {
  Serial.begin(115200);

  Serial.println(F("WXLink Tx Present"));

  if (!rf95.init()) {
    Serial.println(F("init failed"));
    while (1);
  }
  if(!am2315.begin()) {
    Serial.println(F("AM2315 NOPE"));
    while(1);
  }
  if(!bmp.begin(0x76)) {
    Serial.println(F("BMP280 NOPE"));
    while(1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // 준비
                  Adafruit_BMP280::SAMPLING_X2, // 온도샘플링
                  Adafruit_BMP280::SAMPLING_X16, // 기압 샘플링
                  Adafruit_BMP280::FILTER_X16, // 필터링
                  Adafruit_BMP280::STANDBY_MS_500); // 0.5초 대기

  rf95.setFrequency(434.0);
  rf95.setTxPower(13);
  
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);

  Serial.print(F("Wireless ID:"));
  Serial.println(WIRELESSID);

  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  wakeState = REBOOT;
  Alarm_State_1 = false;
  Alarm_State_2 = false;
  nextSleepLength = SLEEPCYCLE;


  Protocol = WIRELESSID * 10 + SOFTWAREVERSION;
  TimeStamp = 0;
  WindDirection = 0;
  AveWindSpeed = 0.0;
  TotalRainClicks = 0;
  MaxWindGust = 0.0;
  OutsideTemperature = 0.0;
  OutsideHumidity = 0.0;
  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;
  AirPressure = 0.0;

  lastRainTime = 0;
  lastWindTime = 0;
  shortestWindTime = 10000000;

  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);

  Wire.begin();

  pinMode(2, INPUT);
  digitalWrite(2, HIGH);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);

  digitalWrite(ENABLE_RADIO, HIGH);
  pinMode(ENABLE_RADIO, OUTPUT);

  ignore_anemometer_interrupt = false;
  attachInterrupt(0, serviceInterruptAnem, RISING);
  attachInterrupt(1, serviceInterruptRain, FALLING);

  DS3231_Present = false;

  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet) {
    Serial.println(F("DS3231 Not Present"));
    DS3231_Present = false;
  }
  else {
    Serial.println(F("DS3231 Present"));
    digitalClockDisplay();
    DS3231_Present = true;

    RTC.squareWave(SQWAVE_NONE);
    set32KHz(false);

    RTC.setAlarm(ALM1_MATCH_SECONDS, 30, 0, 0, 0);
    RTC.alarm(ALARM_1);
    RTC.setAlarm(ALM2_EVERY_MINUTE , 0, 0, 0, 0);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, true);
    RTC.alarmInterrupt(ALARM_2, true);
  }

  uint8_t s = RTC.readRTC(RTC_STATUS);
  Serial.print(F("RTC_STATUS="));
  Serial.println(s, BIN);
  s = RTC.readRTC(RTC_CONTROL);
  Serial.print(F("RTC_CONTROL="));
  Serial.println(s, BIN);

  SunAirPlus_Present = false;

  LoadVoltage = SunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL);

  Serial.print("SAP Load Voltage =");
  Serial.println(LoadVoltage);

  LoadVoltage = SunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL);
  LoadCurrent = SunAirPlus.getCurrent_mA(OUTPUT_CHANNEL);


  BatteryVoltage = SunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
  BatteryCurrent = SunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL);

  SolarPanelVoltage = SunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL);
  SolarPanelCurrent = -SunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL);

  Serial.println("");
  Serial.print(F("LIPO_Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
  Serial.print(F("LIPO_Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
  Serial.println("");

  Serial.print(F("Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
  Serial.print(F("Solar Panel Current:   ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
  Serial.println("");

  Serial.print(F("Load Voltage:   ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
  Serial.print(F("Load Current:   ")); Serial.print(LoadCurrent); Serial.println(F(" mA"));
  Serial.println("");

  if (LoadVoltage < 0.1) {
    SunAirPlus_Present = false;
    Serial.println(F("SunAirPlus Not Present"));
  }
  else {
    SunAirPlus_Present = true;
    Serial.println(F("SunAirPlus Present"));
  }
}


void loop() {
  tmElements_t tm;
  RTC.read(tm);

  if (DS3231_Present) {
    RTC.get();
#if defined(TXDEBUG)
    digitalClockDisplay();
#endif
  }

  if ((wakeState == SLEEP_INTERRUPT) || (Alarm_State_1 == true)  || (Alarm_State_2 == true)) {

    wakeState = NO_INTERRUPT;
    Alarm_State_1 = false;
    Alarm_State_2 = false;

    Serial.print(F("MessageCount="));
    Serial.println(MessageCount);
    delay(1000);
    Serial.print(F("Outside Temperature (C): ")); Serial.println(am2315.readTemperature());
    delay(1000);
    Serial.print(F("Outside Humidity (%RH): ")); Serial.println(am2315.readHumidity());
    delay(1000);
    OutsideTemperature = am2315.readTemperature();
    delay(1000);
    OutsideHumidity = am2315.readHumidity();

    float WindVaneVoltage;
    WindVaneVoltage = analogRead(A1) * (5.0 / 1023.0);
    Serial.print(F("Wind Vane Voltage ="));
    Serial.println(WindVaneVoltage);
    Serial.print(F("Wind Vane Degrees ="));
    WindDirection = voltageToDegrees(WindVaneVoltage, 0.0);
    Serial.println(WindDirection);
    Serial.print(F("TotalRainClicks="));
    Serial.println(TotalRainClicks);
    Serial.print(F("windClicks="));
    Serial.println(windClicks);

    int checkMinute = tm.Minute;
    int checkHour = tm.Hour;
    Serial.print(F("TotalRain mm : "));
    Serial.println(dailyRain, 2);
    Serial.print(F("Rain per Hour : "));
    Serial.println(RainMm, 2);
    Serial.print(F("dailyRain_till_LastHour : "));
    Serial.println(dailyRain_till_LastHour, 2);

      
    if(checkMinute != 0) first = true;
    if(checkMinute == 0 && first == true) {
      hourlyRain = dailyRain - dailyRain_till_LastHour;
      RainMm = hourlyRain;
      dailyRain_till_LastHour = dailyRain;
      
      first = false;    
    } 
    if(checkHour == 0) {
      dailyRain = 0.0;
      dailyRain_till_LastHour = 0.0;
    }

    
    
    AirPressure = bmp.readPressure()/100;
    Serial.print(F("Air Pressure : "));
    Serial.println(AirPressure);
    AveWindSpeed = (((float)windClicks / 2.0) / (((float)SLEEPCYCLE) / 1000.0)) * 2.4; // 2.4 KPH/click

    Serial.print(F("Average Wind Speed="));
    Serial.print(AveWindSpeed);
    Serial.println(F(" KPH"));

    Serial.print("shortestWindTime (usec)");
    Serial.println(shortestWindTime);

    MaxWindGust = 0.0;

    shortestWindTime = 10000000;

    TimeStamp = millis();

    if (SunAirPlus_Present) {

      LoadVoltage = SunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL);
      LoadCurrent = SunAirPlus.getCurrent_mA(OUTPUT_CHANNEL);

      BatteryVoltage = SunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
      BatteryCurrent = SunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL);

      SolarPanelVoltage = SunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL);
      SolarPanelCurrent = -SunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL);

      Serial.println("");
      Serial.print(F("LIPO_Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
      Serial.print(F("LIPO_Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
      Serial.println("");
      
      Serial.print(F("Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
      Serial.print(F("Solar Panel Current:   ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
      Serial.println("");
      
      Serial.print(F("Load Voltage:   ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
      Serial.print(F("Load Current:   ")); Serial.print(LoadCurrent); Serial.println(F(" mA"));
      Serial.println("");
    }
    int bufferLength;
    bufferLength = buildProtocolString();
    Serial.println(F("----------Sending packet----------"));

    rf95.send(byteBuffer, bufferLength);
    Serial.println(F("----------After Sending packet----------"));
    if (!rf95.waitPacketSent(6000)) {
      Serial.println(F("Timeout on transmission"));
      if (!rf95.init()) {
        Serial.println(F("init failed"));
        while (1);
      }
      rf95.setFrequency(434.0);
      rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
      rf95.setTxPower(13);
      Serial.println(F("----------Board Reinitialized----------"));
    }
    else {
      Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
      rf95.sleep();
    }
    Serial.println(F("----------After Wait Sending packet----------"));
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    Serial.println(bufferLength);

    for (int i = 0; i < bufferLength; i++) {
      Serial.print(" ");
     /* if (byteBuffer[i] < 16) {
        Serial.print(F("0"));
      }*/
      Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
    }
    Serial.println();
    MessageCount++;
    windClicks = 0;
  }

  if (DS3231_Present == false) {
    if (wakeState != REBOOT)
      wakeState = SLEEP_INTERRUPT;
    long timeBefore;
    long timeAfter;
    timeBefore = millis();
    delay(100);
    /*if (DS3231_Present == false) {
      for (long i = 0; i < nextSleepLength / 16; ++i) Sleepy::loseSomeTime(16);
      wakeState = SLEEP_INTERRUPT;
      timeAfter = millis();
      long time;
      time = millis();
    } */
  }

  if (DS3231_Present == true) {
    delay(50);
    Sleepy::powerDown ();
    if (RTC.alarm(ALARM_1)) {
      Serial.println(F("ALARM_1 Found"));
      wakeState = ALARM_INTERRUPT;
      Alarm_State_1 = true;
      // remove one rain click
      if (TotalRainClicks > 0)
        TotalRainClicks = TotalRainClicks - 1;
    }
    if (RTC.alarm(ALARM_2)) {
      Serial.println(F("ALARM_2 Found"));
      wakeState = ALARM_INTERRUPT;
      Alarm_State_2 = true;
      // remove one rain click
      if (TotalRainClicks > 0)
        TotalRainClicks = TotalRainClicks - 1;
    }
  }
  if (wakeState == ANEMOMETER_INTERRUPT) {
    ignore_anemometer_interrupt = true;
    delay(17);
    ignore_anemometer_interrupt = false;
  }
  ResetWatchdog();
}
