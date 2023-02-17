/**************************************************************************
 *                선덕고등학교 탐구학술대회 프로젝트 2                    *
 *                      -< 스마트 팜 프로젝트 >-                          *
 *                                                                        *  
 * 보고서 명 : 기존 농원의 자동화를 위한 MCU-control 시스템 구현하기      *
 *                                                                        *
 * 이 프로젝트는 선덕고등학교 탐구학술 대회에 쓰이는 코드로               *
 * 단일 아두이노 메가로 이루어진 프로젝트입니다.                          *
 *                                                                        *
 * This project is a code which is used to do for Sunduck Highshcool      *
 * Research scholarship competition, using two Arduino Mega 2560.         *
 *                                                                        *
 * Copyright ⓒ 2019 Sunduck School. All Rights Reserved.                 *
 *                                                                        *
 * 사용하는 센서는 다음과 같습니다.                                       *
 *                                                                        *
 * 1. 토양습도센서(부식방지)                                              *
 * 2. 토양온도센서(방수)                                                  *
 * 3. 온습도센서(농업용)                                                  *
 * 4. 비 감지센서                                                         *
 * 5. 조도센서                                                            *
 * 6. UV센서                                                              *
 * 7. 이산화탄소                                                          *                                                  *
 *                                                                        *
 * 출력 요소는 다음과 같습니다.                                           *
 *                                                                        *
 * 1. 물펌프                                                              *
 * 2. LoRa Radio Module 433MHz                                            *
 * 3. 12864 그래픽 LCD 모듈                                               *
 *                                                                        *
 * 이 장치는 수신용입니다.                                                *
 *                                                                        *
 **************************************************************************/

// ===== 모듈 핀아웃 정의 시작 ===== //


#define Co2PIN 8
#define temperData 10
#define temperClk 11


#define LightPin A3
#define SoilPin A0
#define RainPin A2

const int AirValue = 520;   //건조 상태에서 출력되는 값
const int WaterValue = 260;  //습윤 상태에서 출력되는 값
int intervals = (AirValue - WaterValue)/3; 
int soilMoistureValue = 0;

#define ONE_WIRE_BUS 6

#define Relay 7 

#define LoRaTx 4
#define LoRaRx 3

#define SLEEP_CYCLE 10 // 30초 주기로 절전모드가 반복됨.


// ===== 모듈 핀아웃 정리 종료 ===== //

// ==========================================================================

// ===== 모듈 라이브러리 지정 시작 ===== //

// 절전을 위한 라이브러리
#include <LowPower.h>

// DallarTemperature를 위한 기본 DS18B20 라이브러리
#include <OneWire.h>

// I2C 통신 라이브러리(기본)
#include <Wire.h>

// 온습도계
#include <SHT1x.h>

// 자외선
#include <DFRobot_VEML6075.h>

// 토양온도
#include <DallasTemperature.h>

// LoRa 모듈
#include <SoftwareSerial.h>

// ===== 모듈 라이브러리 지정 종료 ===== //

// ==========================================================================

// ===== 오브젝트 지정 시작 ===== //

// 온습도계
SHT1x sht1x(temperData, temperClk);

// 자외선
DFRobot_VEML6075_IIC VEML6075(&Wire, VEML6075_ADDR);

// LoRa 모듈 통신 직렬포트
//SoftwareSerial LORA(2, 3);

// 온도 DS18B20 설정
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


// ===== 오브젝트 지정 종료 ===== //

// ==========================================================================

// ===== 전역변수 지정 시작 ===== //

// 토양의 상태
typedef enum soilState {
  Watered, Wet, Dry
}soil;

soil soilState;

int dirt = 0;

// 공기 온습도
float airTemp = 0.0;
float airHumi = 0.0;

// 자외선
float Uva = 0.0;                   
float Uvb = 0.0;                   
float Uvi = 0.0;

// 광도
int LuxAnalog = 0;
float Lux = 0.0;


// 비 감지
int rain = 0;

// ===== 전역변수 지정 종료 ===== //

// ==========================================================================


void checkTemperandHumi() {
  airTemp = sht1x.readTemperatureC();
  airHumi = sht1x.readHumidity();
}

void checkUV() {
  Uva = VEML6075.getUva();                   
  Uvb = VEML6075.getUvb();                   
  Uvi = VEML6075.getUvi(Uva, Uvb);
}

void checkLux() {
  LuxAnalog = analogRead(LightPin);
}

void setup() {
  
  Serial.begin(9600);
  
// ==========================================================================

  // ===== 오류감지 시작 ===== //
  
  while(VEML6075.begin() != true) {
    //Serial.println("VEML6075 begin faild");
    delay(2000);
  }
  //Serial.println("UV sensor is Working");
  
  sensors.begin();
  //Serial.println("DS18B20(temperature) Sensor is Working");

  //Serial.println("※All Sensors are Working. Move to Next Sequince.");
  
  // ===== 오류감지 종료 ===== //
  
// ==========================================================================

  // ===== 핀번호 지정 시작 ===== //

  // 릴레이 모듈 설정
  pinMode(7, OUTPUT);
  
  // ===== 핀번호 지정 종료 ===== //
  
}

void loop() {
  
  // ===== 센서 값 읽기 시작 ===== //

  //Serial.println("READING SENSORS");

  // 토양 온도 DS18B20
  sensors.requestTemperatures();
  float SoilTemp = sensors.getTempCByIndex(0);
  //Serial.println("TEMP DONE");
  
  // 토양 습도
  soilMoistureValue = analogRead(SoilPin);
  //Serial.println("Soil Almost");
  
  if(soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals)) {
  //굉장히 습한 상태 - 코드 작성  
  dirt = 0;
  //Serial.println("SoilStaus : Watered");
  } else if(soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals)) { 
  //습한 상태 - 코드 작성
  dirt = 1;
  //Serial.println("SoilStaus : Wet");
  } else if(soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals)) {
  //건조한 상태 - 코드 작성
  dirt = 2;
  //Serial.println("SoilStaus : Dry");
  }
  //Serial.println("Soil Done");

  checkTemperandHumi();
  //Serial.println("Air temphumi done");

  checkUV();
  //Serial.println("UV Done");

  checkLux(); 
  //Serial.println("Lux Done");

  rain = analogRead(RainPin);
  
  // ===== 센서 값 읽기 종료 ===== //

  
  // ===== 물펌프 작동 시작 ===== //


  if(dirt == 2 && rain >= 1000 ){
    digitalWrite(7, HIGH);
    delay(5000);
    digitalWrite(7, LOW);
  } else {
    digitalWrite(7, LOW);
  }

  // ===== 물펌프 작동 종료 ===== //

  // ===== 정보 전달 문자열 전송 시작 ===== //

  delay(500);

  String txstring = "";

  txstring += 'S';
  txstring += airTemp;
  txstring += ',';
  txstring += airHumi;
  txstring += ',';
  txstring += Uvi;
  txstring += ',';
  txstring += LuxAnalog;
  txstring += ',';
  txstring += dirt;
  txstring += ',';
  txstring += SoilTemp;
  txstring += 'E';
  

  Serial.println(txstring);
  delay(5000);
  
  
  // ===== 정보 전달 문자열 전송 종료 ===== //

  delay(2000);

  // ===== 절전 모드 시작 ===== //
  
  for(int k = 0; k < SLEEP_CYCLE; k++) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }

  // ===== 절전 모드 종료 ===== //
}
