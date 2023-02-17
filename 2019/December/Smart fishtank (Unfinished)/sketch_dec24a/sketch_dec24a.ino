#include <Wire.h>                  // I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>   // LCD 라이브러리
LiquidCrystal_I2C lcd(0x27,20,4);  // 2004 LCD 설정

int ClkPin = 2;                     // 로터리 엔코더 Clk 핀 (D4)
int DtPin = 3;                      // 로터리 엔코더 Dt 핀 (D5)
int SWPin = 4;                      // 로터리 엔코더 스위치 핀 (D6)

uint8_t arrow[8] = {0x0, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00, 0x00};  // LCD 사용자 문자 '우측 화살표'(→)

int menu = 0;                       // 메뉴 번호

int getEncoderTurn() {                  // 로터리 엔코더 변화율 측정
  // -1,0,+1의 값중 하나를 반환합니다.
  static int oldA = LOW;
  static int oldB = LOW;
  int result = 0;
  int newA = digitalRead(ClkPin);
  int newB = digitalRead(DtPin);
  if (newA != oldA || newB != oldB) {
    // 바뀐 값이 있을 때
    if (oldA == LOW && newA == HIGH) {
      result = -(oldB * 2 - 1);
    }
  }
  oldA = newA;
  oldB = newB;
  
  return result;
} 

void setState() {
  
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrow);
  pinMode(ClkPin, INPUT);
  pinMode(DtPin, INPUT);
  digitalWrite(SWPin, HIGH);
  
}

void loop() {
  int change = getEncoderTurn();
  if(change == 0){
  } else {
    menu = menu + change;
  if(menu < 0) menu = 0;
  else if(menu > 3) menu = 3;
  Serial.println(menu);
  if (menu == 0) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.write(0);
    lcd.print("MONITORING");
    lcd.setCursor(1,1);
    lcd.print("AIR MAKER");
    lcd.setCursor(1,2);
    lcd.print("FILTER");
    lcd.setCursor(1,3);
    lcd.print("FEEDING");
  }
  else if (menu == 1) {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("MONITORING");
    lcd.setCursor(0,1);
    lcd.write(0);
    lcd.print("AIR MAKER");
    lcd.setCursor(1,2);
    lcd.print("FILTER");
    lcd.setCursor(1,3);
    lcd.print("FEEDING");
  }
  else if (menu == 2) {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("MONITORING");
    lcd.setCursor(1,1);
    lcd.print("AIR MAKER");
    lcd.setCursor(0,2);
    lcd.write(0);
    lcd.print("FILTER");
    lcd.setCursor(1,3);
    lcd.print("FEEDING");
  }
  else if (menu == 3) {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("MONITORING");
    lcd.setCursor(1,1);
    lcd.print("AIR MAKER");
    lcd.setCursor(1,2);
    lcd.print("FILTER");
    lcd.setCursor(0,3);
    lcd.write(0);
    lcd.print("FEEDING");
  }
  }

}
