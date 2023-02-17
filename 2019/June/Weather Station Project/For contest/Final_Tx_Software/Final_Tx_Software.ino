
/****************************************************************************
 *            ● 선덕고등학교 소프트웨어 대회 소스코드 파일 ●              *       
 *                                                                          *
 *    이 소스코드는 OhMyWeather!에 들어가는 소프트웨어 중                   *
 *    날씨 측정기의 정보를 받아 전송하는 송신기 소프트웨어 입니다.          *
 *                                                                          *
 *    이 프로젝트에서 가장 중심이 되는 소프트웨어로, 연산과 함수,           *
 *    라이브러리가 가장 많이 포함됩니다.                                    *
 *                                                                          *
 *    다음 코드들은 대회에 이용 후 블로그나 깃허브에                        *
 *    공개할 예정입니다.                                                    *
 ****************************************************************************/

// Serial 통신 조건문
#define TXDEBUG 

// 절전모드 등 다양한 기능
#include <JeeLib.h>

// LoRa 통신 라이브러리
#include <RH_RF95.h>

// 소프트웨어 시리얼 핀 라이브러리
#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(6, 7);
RH_RF95 rf95(SoftSerial);

// 아두이노 기본 라이브러리
#include <Wire.h>

// 아두이노 통신 검사를 위한 라이브러리
#include "Crc16.h"

// 통신 상태 검사 라이브러리 설정
Crc16 crc;

//JeeLib 기본 설정. 라이브러리 지시사항
ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

// 기상측정대 라이브러리
#include "WeatherRack.h"

// 태양광 발전을 위한 라이브러리들과 설정
#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"

SDL_Arduino_INA3221 SunAirPlus;

// RTC모듈 사용을 위한 라이브러리
#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>
#include "DS3232RTC.h"


// 온습도 센서 라이브러리와 설정
#include "SDL_ESP8266_HR_AM2315.h"
unsigned long badAM2315Reads = 0;

SDL_ESP8266_HR_AM2315 am2315;
float dataAM2315[2];
boolean OK;

// SunAirPlus 핀 설정
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3

// 메세지 길이 저장
unsigned long MessageCount = 0;

// 기기의 상태 파악
typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  RAIN_INTERRUPT,
  ANEMOMETER_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;

wakestate wakeState;

// 각 기기들의 현 상황 파악
bool SunAirPlus_Present;
bool DS3231_Present;

// 전송 문자열 지정
byte byteBuffer[200];

// State Variables
byte Protocol;              // 프로토콜
long TimeStamp;             // 시간 문자열
int WindDirection;          // 풍향
float AveWindSpeed;         // 풍속
long TotalRainMm;           // 총 강수량(mm)
float MaxWindGust;          // 최대돌발풍속
float OutsideTemperature;   // 외부 온도
float OutsideHumidity;      // 외부 습도
float BatteryVoltage;       // 배터리 전압
float BatteryCurrent;       // 배터리 전류
float LoadVoltage;          // 전압 사용량
float LoadCurrent;          // 전류 사용량
float SolarPanelVoltage;    // 태양광 발전 전압 
float SolarPanelCurrent;    // 태양광 발전 전류 (얜 회로 문제인지 0이 나올 때가 있음)
int protocolBufferCount;    // 프로토콜 문자열 총 길이

// RTC 알림 설정, 절전모드 설정
bool Alarm_State_1;
bool Alarm_State_2;
long nextSleepLength;

// 기상 측정과 관련된 설정
bool ignore_anemometer_interrupt;
long lastRainTime;
long windClicks;
long  lastWindTime;
long shortestWindTime;

// LED를 사용하여 기기의 오류 판별

void ledError(int mode) {
  digitalWrite(LED, LOW);
  if(mode == 1) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  } else if(mode == 2) {
    digitalWrite(LED, HIGH);
    delay(100);
    digital(LED, LOW);
    delay(100);
  } else if(mode == 3) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  } else if(mode == 4) {
    digitalWrite(LED, HIGH);
    delay(2000);
    digitalWrite(LED, LOW);
    delay(2000);
  }
  
}

void ledNomal(int cycle) {
  for(int i = 0; i < cycle; i++) {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }
}

void LEDState(int mode) {
  switch(mode) {
    case 0: ledError(1); break;
    case 1: ledNomal(3); break;
    case 2: ledError(2); break;
    case 3: ledError(3); break;
    else: ledError(4); break;
  }
}


int convert4ByteLongVariables(int bufferCount, long myVariable) {

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable) {
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
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
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}

int buildProtocolString() {

  int bufferCount = 0;

  byteBuffer[bufferCount] = 0xAB;
  bufferCount++;
  byteBuffer[bufferCount] = 0x66;
  bufferCount++;
  bufferCount = convert1ByteVariables(bufferCount, Protocol);
  bufferCount = convert4ByteLongVariables(bufferCount, TimeStamp / 100);
  bufferCount = convert2ByteVariables(bufferCount, WindDirection);
  bufferCount = convert4ByteFloatVariables(bufferCount, AveWindSpeed);
  bufferCount = convert4ByteLongVariables(bufferCount, windClicks);
  bufferCount = convert4ByteLongVariables(bufferCount, TotalRainMm);
  bufferCount = convert4ByteFloatVariables(bufferCount, MaxWindGust);
  bufferCount = convert4ByteFloatVariables(bufferCount, OutsideTemperature);
  bufferCount = convert4ByteFloatVariables(bufferCount, OutsideHumidity);

  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);
  protocolBufferCount = bufferCount + 2;
  bufferCount = checkSum(bufferCount);




  return bufferCount;


}



void printStringBuffer()
{
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


// DS3231 Library functions

void printDigits(int digits) {
  if (digits < 10) Serial.print(F("0"));
  Serial.print(digits);
}

void digitalClockDisplay() {

  tmElements_t tm;
  RTC.read(tm);

  printDigits(tm.Hour);
  Serial.print(F("시 "));
  printDigits(tm.Minute);
  Serial.print(F("분 "));
  printDigits(tm.Second);
  Serial.print(F("초 "));
  Serial.print(tm.Year + 1970);
  Serial.print(F("년"));
  Serial.print(tm.Month);
  Serial.print(F("월"));
  Serial.print(tm.Day);
  Serial.print(F("일"));
  Serial.println();
}

void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);


}

void set32KHz(bool setValue)
{

  uint8_t s = RTC.readRTC(RTC_STATUS);

  if (setValue == true)
  {
    s = s | (1 << EN32KHZ);
    RTC.writeRTC(RTC_STATUS, s);

  }
  else
  {
    uint8_t flag;
    flag =  ~(1 << EN32KHZ);
    s = s & flag;
    RTC.writeRTC(RTC_STATUS, s);

  }



}


void ResetWatchdog()
{

  if (badAM2315Reads < 5) {
    digitalWrite(WATCHDOG_1, LOW);
    delay(200);
    digitalWrite(WATCHDOG_1, HIGH);
#if defined(TXDEBUG)
    Serial.println(F("Watchdog1 Reset - Patted the Dog"));
#endif
  }
  else {
#if defined(TXDEBUG)
    Serial.println(F("AM2315 bad read > 5, stop patting dog"));
#endif
  }
}



void setup()
{
  Serial.begin(9600);
  Serial.println(F("OhMyWeather Present"));

  

  if (!rf95.init()) {
    Serial.println("init failed");
    while(1);
  }

  rf95.setFrequency(434.0);
  rf95.setTxPower(13);
  
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);


  Serial.print(F("Wireless ID:"));
  Serial.println(WIRELESSID);

  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  pinMode(LED, OUTPUT);

  LEDState(1);

  wakeState = REBOOT;
  Alarm_State_1 = false;
  Alarm_State_2 = false;
  nextSleepLength = SLEEPCYCLE;


  Protocol = pow(WIRELESSID) + SOFTWAREVERSION;
  TimeStamp = 0;
  WindDirection = 0;
  AveWindSpeed = 0.0;
  TotalRainMm = 0;
  MaxWindGust = 0.0;
  OutsideTemperature = 0.0;
  OutsideHumidity = 0.0;
  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;

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

  ignore_anemometer_interrupt = false;
  attachInterrupt(0, serviceInterruptAnem, RISING);
  attachInterrupt(1, serviceInterruptRain, FALLING);
  
  DS3231_Present = false;

  setSyncProvider(RTC.get);
  
  //RTC모듈 초기 설정. 한번 컴파일 이후 다시 주석처리함.
  RTC.adjust(DateTime(__DATE__,__TIME__));
  
  if (timeStatus() != timeSet)
  {
    Serial.println(F("DS3231 Not Present"));
    DS3231_Present = false;
  }
  else
  {
    Serial.println(F("DS3231 Present"));
    digitalClockDisplay();
    DS3231_Present = true;

    RTC.squareWave(SQWAVE_NONE);
    set32KHz(false);

    if (SLEEPCYCLE < 1001) {

      RTC.setAlarm(ALM1_EVERY_SECOND, 0, 0, 0, 0);
      RTC.alarm(ALARM_1);
      RTC.alarm(ALARM_2);
      RTC.alarmInterrupt(ALARM_1, true);


    }
    else if (SLEEPCYCLE < 30001)
    {
      // choose once per 30 seconds

      RTC.setAlarm(ALM1_MATCH_SECONDS, 30, 0, 0, 0);
      RTC.alarm(ALARM_1);
      RTC.setAlarm(ALM2_EVERY_MINUTE , 0, 0, 0, 0);
      RTC.alarm(ALARM_2);
      RTC.alarmInterrupt(ALARM_1, true);
      RTC.alarmInterrupt(ALARM_2, true);

    }
    else // if (SLEEPCYCLE < 60001)
    {
      // choose once per minute

      RTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 0);
      RTC.alarm(ALARM_1);
      RTC.alarm(ALARM_2);
      RTC.alarmInterrupt(ALARM_1, true);


    }

    uint8_t readValue;

  }

  uint8_t s = RTC.readRTC(RTC_STATUS);
  Serial.print(F("RTC_STATUS="));
  Serial.println(s, BIN);
  s = RTC.readRTC(RTC_CONTROL);
  Serial.print(F("RTC_CONTROL="));
  Serial.println(s, BIN);


  // test for SunAirPlus_Present
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

  if (LoadVoltage < 0.1)
  {
    SunAirPlus_Present = false;
    Serial.println(F("SunAirPlus Not Present"));
  }
  else
  {
    SunAirPlus_Present = true;
    Serial.println(F("SunAirPlus Present"));
  }




}




void loop()
{

  if (DS3231_Present)
  {
    RTC.get();
#if defined(TXDEBUG)
    digitalClockDisplay();
#endif
  }
  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif


  if ((wakeState == SLEEP_INTERRUPT) || (Alarm_State_1 == true)  || (Alarm_State_2 == true))
  {

    wakeState = NO_INTERRUPT;
    Alarm_State_1 = false;
    Alarm_State_2 = false;

    Serial.print(F("MessageCount="));
    Serial.println(MessageCount);

    OK = am2315.readData(dataAM2315);

    if (OK) {
      Serial.print(F("Outside Temperature (C): ")); Serial.println(dataAM2315[1]);
      Serial.print(F("Outside Humidity (%RH): ")); Serial.println(dataAM2315[0]);
      OutsideTemperature = dataAM2315[1];
      OutsideHumidity = dataAM2315[0];


    }
    else
    {
      Serial.println(F("AM2315 not found"));
      badAM2315Reads++;   // increment the bad one

    }

    // now read wind vane

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

    // calculate wind
    AveWindSpeed = (((float)windClicks / 2.0) / (((float)SLEEPCYCLE) / 1000.0)) * 2.4; // 2.4 KPH/click

    Serial.print(F("Average Wind Speed="));
    Serial.print(AveWindSpeed);
    Serial.println(F(" KPH"));

    Serial.print("shortestWindTime (usec)");
    Serial.println(shortestWindTime);

    MaxWindGust = 0.0;

    shortestWindTime = 10000000;

    TimeStamp = millis();

    // if SunAirPlus present, read charge data

    if (SunAirPlus_Present)
    {

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

    // write out the current protocol to message and send.
    int bufferLength;
    bufferLength = buildProtocolString();

    Serial.println(F("----------Sending packet----------"));

    /*
      /// turn radio on
      digitalWrite(ENABLE_RADIO, LOW);
      delay(100);
      SoftSerial.write((uint8_t *)byteBuffer, bufferLength);
    */
    // Send a message

    rf95.send(byteBuffer, bufferLength);
    Serial.println(F("----------After Sending packet----------"));
    if (!rf95.waitPacketSent(6000))
    {
      Serial.println(F("Timeout on transmission"));
      // re-initialize board
      if (!rf95.init())
      {
        Serial.println(F("init failed"));
        while (1);
      }

      // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

      // The default transmitter power is 13dBm, using PA_BOOST.
      // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
      // you can set transmitter powers from 5 to 23 dBm:
      //rf95.setTxPower(13, false);

      rf95.setFrequency(434.0);

      rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
      // rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);

      rf95.setTxPower(13);

      //rf95.printRegisters();
      Serial.println(F("----------Board Reinitialized----------"));


    }
    else
    {
      Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
      rf95.sleep();
    }
    Serial.println(F("----------After Wait Sending packet----------"));
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
    Serial.print(F("bufferlength="));
    Serial.println(bufferLength);

    for (int i = 0; i < bufferLength; i++) {
      Serial.print(" ");
      if (byteBuffer[i] < 16)
      {
        Serial.print(F("0"));
      }
      Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
    }
    Serial.println();

    /*
      delay(100);
       digitalWrite(ENABLE_RADIO, HIGH);
       // turn radio off
    */

    MessageCount++;

    windClicks = 0;


  }





  if (DS3231_Present == false)
  {
    if (wakeState != REBOOT)
      wakeState = SLEEP_INTERRUPT;
    long timeBefore;
    long timeAfter;
    timeBefore = millis();
#if defined(TXDEBUG)
    Serial.print(F("timeBeforeSleep="));
    Serial.println(timeBefore);
#endif
    delay(100);
    // This is what we use for sleep if DS3231 is not present
    if (DS3231_Present == false)
    {
      //Sleepy::loseSomeTime(nextSleepLength);
      for (long i = 0; i < nextSleepLength / 16; ++i)
        Sleepy::loseSomeTime(16);

      wakeState = SLEEP_INTERRUPT;

#if defined(TXDEBUG)
      Serial.print(F("Awake now: "));
#endif
      timeAfter = millis();
#if defined(TXDEBUG)
      Serial.print(F("timeAfterSleep="));
      Serial.println(timeAfter);

      Serial.print(F("SleepTime = "));
      Serial.println(timeAfter - timeBefore);

      Serial.print(F("Millis Time: "));
#endif
      long time;
      time = millis();
#if defined(TXDEBUG)
      //prints time since program started
      Serial.println(time / 1000.0);
      Serial.print(F("2wakeState="));
      Serial.println(wakeState);
#endif
    }
  }

  if (DS3231_Present == true)
  {
    // use DS3231 Alarm to Wake up
#if defined(TXDEBUG)
    Serial.println(F("Using DS3231 to Wake Up"));
#endif
    delay(50);
    Sleepy::powerDown ();

    if (RTC.alarm(ALARM_1))
    {
      Serial.println(F("ALARM_1 Found"));
      wakeState = ALARM_INTERRUPT;
      Alarm_State_1 = true;
      // remove one rain click
      if (TotalRainClicks > 0)
        TotalRainClicks = TotalRainClicks - 1;

    }

    if (RTC.alarm(ALARM_2))
    {
      Serial.println(F("ALARM_2 Found"));
      wakeState = ALARM_INTERRUPT;
      Alarm_State_2 = true;
      // remove one rain click
      if (TotalRainClicks > 0)
        TotalRainClicks = TotalRainClicks - 1;

    }

  }

  // if it is an anemometer interrupt, do not process anemometer interrupts for 17 msec  debounce

  if (wakeState == ANEMOMETER_INTERRUPT)
  {
    ignore_anemometer_interrupt = true;
    delay(17);
    ignore_anemometer_interrupt = false;
  }

  // Pat the WatchDog
  ResetWatchdog();


}
