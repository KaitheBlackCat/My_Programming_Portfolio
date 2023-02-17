/*
  Based on the SendDemo example from the RC Switch library
  https://github.com/sui77/rc-switch/
*/

#include <RCSwitch.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);
RCSwitch mySwitch = RCSwitch();

const int clkPin = 2;
const int dtPin = 3;
const int swPin = 4;
int plugNumber = 1;
bool state[5] = {false, false, false, false, false};
int previousChange = 0;

void setup() {
  Serial.begin(9600);
  
  // Transmitter is connected to Arduino Pin #10  
  mySwitch.enableTransmit(10);

  // Optional set pulse length.
  mySwitch.setPulseLength(154);
  
  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);
  
  // Optional set number of transmission repetitions.
  // mySwitch.setRepeatTransmit(15);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Plug ");
  lcd.print(plugNumber);
  pinMode(clkPin, INPUT);
  pinMode(dtPin, INPUT);
  pinMode(swPin, INPUT);
  digitalWrite(swPin, HIGH);
}

void loop() {

  const int norm = 0;
  
  int change = getEncoderTurn();

  if(norm != change) lcd.clear();
  
  plugNumber = plugNumber + change;
  if(plugNumber >= 6) plugNumber = 5;
  else if(plugNumber <= 0) plugNumber = 1;
  
  if(digitalRead(swPin) == LOW)
  {
    if(state[plugNumber] == true) state[plugNumber] = false;
    else state[plugNumber] = true;
    
    controlPlugs(plugNumber, state[plugNumber]); 
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Plug ");
    lcd.print(plugNumber);
    
  }
  
  lcd.setCursor(0,0);
  lcd.print("Plug ");
  lcd.print(plugNumber);
  lcd.setCursor(0,1);
  
  if(state[plugNumber] == false) {
    lcd.print("OFF");
  }
  else {
    lcd.print("ON");
  }
  

}

int getEncoderTurn(void)
{
  static int oldA = HIGH; //set the oldA as HIGH
  static int oldB = HIGH; //set the oldB as HIGH
  int result = 0;
  int newA = digitalRead(dtPin);//read the value of clkPin to newA
  int newB = digitalRead(clkPin);//read the value of dtPin to newB
  if (newA != oldA || newB != oldB) //if the value of clkPin or the dtPin has changed
  {
    // something has changed
    if (oldA == HIGH && newA == LOW)
    {
      result = (oldB * 2 - 1);
    }
  }
  oldA = newA;
  oldB = newB;
  return result;
}

int controlPlugs(int plugnum, bool state) {
  switch(plugnum) {
    case 1:
    if(state == 1) mySwitch.send("110111001001001101001100");
    else mySwitch.send("110111001001001101000100");
    break;
    case 2:
    if(state == 1) mySwitch.send("110111001001001101001010");
    else mySwitch.send("110111001001001101000010");
    break;
    case 3:
    if(state == 1) mySwitch.send("110111001001001101001001");
    else mySwitch.send("110111001001001101000001");
    break;
    case 4:
    if(state == 1) mySwitch.send("110111001001001101001101");
    else mySwitch.send("110111001001001101000101");
    break;
    case 5:
    if(state == 1) mySwitch.send("110111001001001101001011");
    else mySwitch.send("110111001001001101000011");
    break;
    default:
    break;
  }
  
}

