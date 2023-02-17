/**************************************
 * 부품 핀아웃
 * 
 * 온도센서 - D2
 * 서보모터 - D3
 * 송신기 - D12
 * pH핀 - A1
 * EC핀 - A2
 * 
 **************************************/

// 포함하기

#include "DFRobot_PH.h"            // pH센서 라이브러리
#include "DFRobot_EC.h"            // EH센서 라이브러리
#include <EEPROM.h>               // ROM 읽고 쓰기 라이브러리
#include <OneWire.h>               // 온도계, 칩셋에 한 핀으로만 접속하게 하는 라이브러리
#include <DallasTemperature.h>     // DS18B20 온도계 계산 라이브러리
#include <Wire.h>                  // I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>   // LCD 라이브러리
#include <RCSwitch.h>             // RF 스위치 라이브러리
#include <Servo.h>

// 정의하기

#define PH_PIN A1  // pH센서 핀
#define EC_PIN A2  // EC센서 핀
#define TX_PIN 15  // RF 통신 모듈 핀

#define ONE_WIRE_BUS 14          // DS18B20 연결 핀
OneWire oneWire(ONE_WIRE_BUS);  // 객체 설정

DallasTemperature temper(&oneWire);  // 설정 객체로 DallasTemperature 설정
LiquidCrystal_I2C lcd(0x27,20,4);  // 2004 LCD 설정
RCSwitch mySwitch = RCSwitch();    // RF 스위치 설정
DFRobot_PH ph;                      // pH 객체 설정
DFRobot_EC ec;                      // EC 객체 설정
Servo myservo;

 
// 변수 지정

uint8_t arrow[8] = {0x0, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00, 0x00};  // LCD 사용자 문자 '우측 화살표'(→)

float  voltagePH,voltageEC,phValue,ecValue, temperature = 25;        // pH, EC 보정, 저장을 위한 변수 지정 (기본 25)

int feedPin = 2;                     // 로터리 엔코더 Clk 핀 (D3)
int airPin = 3;                      // 로터리 엔코더 Dt 핀 (D18)
int filterPin = 18;                      // 로터리 엔코더 스위치 핀 (D19)

//int feedLED;
//int airLED;
//int filterLED;

int MotorRunDuration = 1000;        // 모터 작동 주기 : 1초
int pinOutMotor = 6;                // 모터 핀 (3번)

bool ON = true;                     // ON 정의 : TRUE
bool OFF = false;                   // OFF 정의 : FALSE

bool totalPlugStatus[3] = {false, false, false};
bool buttonState = false;

static int oldA = HIGH;
static int oldB = HIGH;

int menu = 1;                       // 메뉴 번호
int pos = 0;

void startPump(bool staters) {      // 펌프 작동 함수

  if(staters) sendCommand(1, ON);
  else sendCommand(1, OFF);
  
}

void startFilter(bool staters) {     // 필터 작동 함수

  if(staters) sendCommand(2, ON);
  else sendCommand(2, OFF);
  
}


void startFeed(bool staters) {        // 공급 작동 함수 

  if(staters) {
    myservo.write(0);
  }
  else myservo.write(94);
  
}


void sendCommand(int number, bool onOff){
  switch(number){
    case 1:
      if(onOff)
        mySwitch.send("110111001001001101001100");
      else
        mySwitch.send("110111001001001101000100");
      break;
    case 2:
      if(onOff)
        mySwitch.send("110111001001001101001010");
      else
        mySwitch.send("110111001001001101000010");
      break;
    case 3:
      if(onOff)
        mySwitch.send("110111001001001101001001");
      else
        mySwitch.send("110111001001001101000001");
      break;
    case 4:
      if(onOff)
        mySwitch.send("110111001001001101001101");
      else
        mySwitch.send("110111001001001101000101");
      break;
    case 5:
      if(onOff)
        mySwitch.send("110111001001001101001011");
      else
        mySwitch.send("110111001001001101000011");
      break;
    default:
      break;
  }
}


void setup() {
  Serial.begin(9600);
  temper.begin();
  ph.begin();
  ec.begin();
  myservo.attach(6);
  

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrow);

  pinMode (feedPin, INPUT_PULLUP);
  pinMode (airPin, INPUT_PULLUP);
  pinMode (filterPin, INPUT_PULLUP);
  //pinMode (feedLED, OUTPUT);
  //pinMode (airLED, OUTPUT);
  //pinMode (filterLED, OUTPUT);
  pinMode(pinOutMotor, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(feedPin), doFeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(airPin), doFilter, FALLING);
  attachInterrupt(digitalPinToInterrupt(filterPin), doAir, FALLING);

  mySwitch.enableTransmit(TX_PIN);
  mySwitch.setPulseLength(154);
  mySwitch.setProtocol(1);

  
}

void doFeed() {
  totalPlugStatus[0] = !totalPlugStatus[0];
      
}

void doFilter() {
  totalPlugStatus[1] = !totalPlugStatus[1];
      
}

void doAir() {
  totalPlugStatus[2] = !totalPlugStatus[2];
}

void printTemperature()
{
  temperature = temper.getTempCByIndex(0);
}

void loop() {
    static unsigned long timepoint = millis();

    lcd.setCursor(0,0);
    lcd.print(" pH  : ");
    lcd.setCursor(7,0);
    lcd.print(phValue, 2);
    if(phValue > 6 && phValue < 8) lcd.print(" (good)");
    else lcd.print(" (bad)");
    lcd.setCursor(0,1);
    lcd.print(" EC  : ");
    lcd.setCursor(7,1);
    lcd.print(ecValue, 2);
    lcd.print(" mS/cm");
    lcd.setCursor(0,2);
    lcd.print("Temp : ");
    lcd.print(temperature, 2);
    lcd.setCursor(1,3);
    if(totalPlugStatus[0]) lcd.print("Feeder");
    if(totalPlugStatus[1]) {
      lcd.setCursor(8,3);
      lcd.print("Filter");
    }
    if(totalPlugStatus[2]) {
      lcd.setCursor(15, 3);
      lcd.print("Air");
    }

    if(totalPlugStatus[0]) {
      startFeed(ON);
    } else {
      startFeed(OFF);
    }
    
    if(totalPlugStatus[1]) {
      startFilter(ON);
    } else {
      startFilter(OFF);
    }
    
    if(totalPlugStatus[2]) {
      startPump(ON);
    } else {
      startPump(OFF);
    }
      
        timepoint = millis();
        temper.requestTemperatures();
        printTemperature();              // read your temperature sensor to execute temperature compensation
        voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
        phValue    = ph.readPH(voltagePH,temperature);       // convert voltage to pH with temperature compensation
        
        Serial.print("pH:");
        Serial.print(phValue,2);
        
        voltageEC = analogRead(EC_PIN)/1024.0*5000;
        ecValue    = ec.readEC(voltageEC,temperature);       // convert voltage to EC with temperature compensation
        
        Serial.print(", EC:");
        Serial.print(ecValue,2);
        Serial.println("ms/cm");

        delay(500);

        lcd.clear();

}

