#include "DFRobot_PH.h"            // pH센서 라이브러리
#include "DFRobot_EC.h"            // EH센서 라이브러리
#include <EEPROM.h>               // ROM 읽고 쓰기 라이브러리
#include <OneWire.h>               // 온도계, 칩셋에 한 핀으로만 접속하게 하는 라이브러리
#include <DallasTemperature.h>     // DS18B20 온도계 계산 라이브러리
#include <Wire.h>                  // I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>   // LCD 라이브러리
#include <RCSwitch.h>             // RF 스위치 라이브러리
LiquidCrystal_I2C lcd(0x3F,20,4);

//Input & Button Logic
const int numOfInputs = 3;                             // 버튼의 수
const int inputPins[numOfInputs] = {8,9,10};        // 핀 설정
int inputState[numOfInputs];                           // 핀들의 상태
int lastInputState[numOfInputs] = {LOW,LOW,LOW};   // 이전 핀들의 상태
bool inputFlags[numOfInputs] = {LOW,LOW,LOW};      // 현재 핀들의 상태 (채터링 방지 이후)
long lastDebounceTime[numOfInputs] = {0,0,0,};        // 채터링 현상 방지
long debounceDelay = 5;                                // 채터링 현상 방지

//LCD Menu Logic
const int numOfScreens = 6;   // 화면의 수
int currentScreen = 0;        // 현재 화면 번호
String screens[numOfScreens][2] = {{"pH Value",""}, {"EC Value", "ms/cm"}, 
{"Temperature","C"},{"Air Pump",""}, {"Filter", ""}, {"Feeder",""}}; // 화면의 내용들
int parameters[numOfScreens]; // 화면에서 변하는 값들

#define PH_PIN A1  // pH센서 핀
#define EC_PIN A2  // EC센서 핀
#define TX_PIN 12  // RF 통신 모듈 핀

#define ONE_WIRE_BUS 2          // DS18B20 연결 핀
OneWire oneWire(ONE_WIRE_BUS);  // 객체 설정

DallasTemperature temper(&oneWire);  // 설정 객체로 DallasTemperature 설정
LiquidCrystal_I2C lcd(0x3F,16,2);  // 2004 LCD 설정
RCSwitch mySwitch = RCSwitch();    // RF 스위치 설정
DFRobot_PH ph;                      // pH 객체 설정
DFRobot_EC ec;                      // EC 객체 설정
 
// 변수 지정

uint8_t arrow[8] = {0x0, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00, 0x00};  // LCD 사용자 문자 '우측 화살표'(→)

float  voltagePH,voltageEC,phValue,ecValue, temperature = 25;        // pH, EC 보정, 저장을 위한 변수 지정 (기본 25)

int feedPin = 3;                     // 로터리 엔코더 Clk 핀 (D3)
int airPin = 18;                      // 로터리 엔코더 Dt 핀 (D18)
int filterPin = 19;                      // 로터리 엔코더 스위치 핀 (D19)

int feedLED;
int airLED;
int filterLED;

int MotorRunDuration = 1000;        // 모터 작동 주기 : 1초
int pinOutMotor = 6;                // 모터 핀 (3번)

bool ON = true;                     // ON 정의 : TRUE
bool OFF = false;                   // OFF 정의 : FALSE

bool totalPulgStatus[3] = {false, false, false};
bool buttonState = false;

static int oldA = HIGH;
static int oldB = HIGH;

int menu = 1;                       // 메뉴 번호
int i = 0;                          // 그냥 있는 녀석

void startPump(bool staters) {      // 펌프 작동 함수

  if(staters) sendCommand(1, ON);
  else sendCommand(1, OFF);
  
}

void startFilter(bool staters) {     // 필터 작동 함수

  if(staters) sendCommand(2, ON);
  else sendCommand(2, OFF);
  
}

void RunMotor(){                      // 모터 작동 함수
    analogWrite(pinOutMotor, 0);
    delay(MotorRunDuration);
    analogWrite(pinOutMotor, 255);
}

void startFeed(bool staters) {        // 공급 작동 함수 

  if(staters) RunMotor();
  else return;
  
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
  for(int i = 0; i < numOfInputs; i++) {
    pinMode(inputPins[i], INPUT);
    digitalWrite(inputPins[i], HIGH); // pull-up 20k
  }
  Serial.begin(9600);
  temper.begin();
  ph.begin();
  ec.begin();

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrow);

  pinMode (feedPin, INPUT_PULLUP);
  pinMode (airPin, INPUT_PULLUP);
  pinMode (filterPin, INPUT_PULLUP);
  pinMode (feedLED, OUTPUT);
  pinMode (airLED, OUTPUT);
  pinMode (filterLED, OUTPUT);
  pinMode(pinOutMotor, OUTPUT);

  attachInterrupt(1, doFeed, FALLING);
  attachInterrupt(4, doFilter, FALLING);
  attachInterrupt(5, doAir, FALLING);

  mySwitch.enableTransmit(TX_PIN);
  mySwitch.setPulseLength(154);
  mySwitch.setProtocol(1);
}

float readTemperature()
{
  temperature = temper.getTempC(0);
}
void doFeed() {
  if(feedState){
        startFeed(OFF);
        Serial.println("Do Feed OFF");
        totalPlugStatus[0] = false;
      }
      else {
        startFeed(ON);
        Serial.println("Do Feed ON");
        totalPlugStatus[0] = true;
      }
      
}

void doFilter() {
  if(filterState){
        startFilter(OFF);
        Serial.println("Do Filter OFF");
        totalPlugStatus[1] = false;
      } else {
        startFilter(ON);
        Serial.println("Do Filter ON");
         totalPlugStatus[1] = true;
      }
      
}

void doAir() {
  if(airPumpState){
        startPump(OFF);
        Serial.println("Do Air OFF");
         totalPlugStatus[2] = false;
      } else {
        startPump(ON);
        Serial.println("Do Air ON");
         totalPlugStatus[2] = true;
      }
      
}


void loop() {
  static unsigned long timepoint = millis();
  setInputFlags();
  resolveInputFlags();
  if(millis() - timepoint > 1000U) {
    command();
  }
}

void command() {
      
        timepoint = millis();
        temperature = readTemperature();                   // read your temperature sensor to execute temperature compensation
        voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
        phValue    = ph.readPH(voltagePH,temperature);       // convert voltage to pH with temperature compensation
        
        Serial.print("pH:");
        Serial.print(phValue,2);
        
        voltageEC = analogRead(EC_PIN)/1024.0*5000;
        ecValue    = ec.readEC(voltageEC,temperature);       // convert voltage to EC with temperature compensation
        
        Serial.print(", EC:");
        Serial.print(ecValue,2);
        Serial.println("ms/cm");

}


void setInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    int reading = digitalRead(inputPins[i]);
    if (reading != lastInputState[i]) {
      lastDebounceTime[i] = millis();
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != inputState[i]) {
        inputState[i] = reading;
        if (inputState[i] == HIGH) {
          inputFlags[i] = HIGH;
        }
      }
    }
    lastInputState[i] = reading;
  }
}

void resolveInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    if(inputFlags[i] == HIGH) {
      inputAction(i);
      inputFlags[i] = LOW;
      printScreen();
    }
  }
}

void inputAction(int input) {
  if(input == 0) {
    if (currentScreen == 0) {
      currentScreen = numOfScreens-1;
    }else{
      currentScreen--;
    }
  }else if(input == 1) {
    if (currentScreen == numOfScreens-1) {
      currentScreen = 0;
    }else{
      currentScreen++;
    } 
  }else if(input == 2) {
    parameterChange(0);
}

void parameterChange(int key) {
  if(key == 0) {
    parameters[currentScreen]++;
  }else if(key == 1) {
    parameters[currentScreen]--;
  }
}

void printScreen() {
  lcd.clear();
  lcd.print(screens[currentScreen][0]);
  lcd.setCursor(0,1);
  lcd.print(parameters[currentScreen]);
  lcd.print(" ");
  lcd.print(screens[currentScreen][1]);
}

