#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

static const int RXPin = 11, TXPin = 10;
static const uint32_t GPSBaud = 9600;

LiquidCrystal_I2C lcd(0x3F,16,2);
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
//  Serial.begin(115200);
  ss.begin(GPSBaud);
     lcd.init();                    
     lcd.backlight();
     lcd.setCursor(0,0);
     lcd.print("ArduGPS Ublox 6M");
     lcd.setCursor(0,1);
     lcd.print(" tukangsapu.net ");
     delay(8000);
     lcd.clear();
}

void loop()
{
  while (ss.available() > 0)
  if (gps.encode(ss.read()))
  if (gps.altitude.isUpdated())
  if (gps.satellites.isUpdated())
     displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    lcd.setCursor(0,1);
    lcd.print(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()

{
  Serial.println();
  lcd.setCursor(0,0);
  lcd.print("Satelitte :    "+String(gps.satellites.value()));
  lcd.setCursor(0,1);
  lcd.print("hdop      :   "+String(gps.hdop.value()));
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Altitude:  "+String(gps.altitude.meters()));
  lcd.setCursor(0,1);
  lcd.print("Speed   :   "+String(gps.speed.kmph()));
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Lat :   "+String(gps.location.lat(),6));
  lcd.setCursor(0,1);
  lcd.print("Long: "+String(gps.location.lng(),6));
  delay(8000);
  lcd.clear();
 }
