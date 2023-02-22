
#include "RTClib.h"
#include "M42.h"
#include "Superstar.h"
#include "MyImages.h"

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define RST 8
#define CS 10
#define DC 9
#define CLOCK_INTERRUPT_PIN 2
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define ENABLE_SECOND true

U8G2_SH1107_PIMORONI_128X128_1_4W_HW_SPI u8g2(U8G2_R0, CS, DC, RST);

float Voltage = 3.71; 
float Hum = 0.0;
float Tem = 0.0;

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

void setup() {

    Serial.begin(9600);

    rtc.begin();

    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    rtc.disable32K();

    u8g2.begin();
    u8g2.setDrawColor(1);
    u8g2.setBitmapMode(0);
    u8g2.setFontMode(0);
    u8g2.setFontDirection(0);
    u8g2.setFontPosBottom();
}

void VoltageLine(float vol) {

    // Equation From https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery

    float vol_P = (123.0 - (123.0 / pow(1.0 + pow(vol / 3.7, 80), 0.165))) / 100.0;
    int lineLength = 58 * vol_P;

    // From middle to left
    u8g2.drawLine(58, 79, 58 - lineLength, 79);
    u8g2.drawLine(58, 80, 58 - lineLength, 80);

    // From middle to right
    u8g2.drawLine(69, 79, 69 + lineLength, 79);
    u8g2.drawLine(69, 80, 69 + lineLength, 80);
}


void loop() {

    DateTime now = rtc.now();

    char currentTime[] = "hh:mm";
    char currentDate[] = "YYYY.MM.DD";
    char currentSecond[] = "ss";

    now.toString(currentTime);
    now.toString(currentDate);
    now.toString(currentSecond);
    
    // picture loop
    u8g2.firstPage();  
    do {

        // Get all the font width first
        u8g2.setFont(Superstar12);
        int cDateWidth = u8g2.getStrWidth(currentDate);
        int cDaysWidth = u8g2.getStrWidth(daysOfTheWeek[now.dayOfTheWeek()]);

        u8g2.setFont(M42_12);
        int cTimeWidth = u8g2.getStrWidth(currentTime);

        u8g2.setFont(M42_6);
        int cSecondsWidth = u8g2.getStrWidth(currentSecond);

        // Date in upper side
        u8g2.setFont(Superstar12);
        int cDateX = (DISPLAY_WIDTH - cDateWidth) / 2;
        u8g2.drawStr(cDateX, 20, currentDate);

        if (ENABLE_SECOND) {
            // Time in middle side
            u8g2.setFont(M42_12);
            int cTimeX = (DISPLAY_WIDTH - (cTimeWidth + cSecondsWidth + 2)) / 2;
            u8g2.drawStr(cTimeX, 48, currentTime);

            // Second in middle side
            u8g2.setFont(M42_6);
            int cSecdX = cTimeX + cTimeWidth + 2;
            u8g2.drawStr(cSecdX, 48, currentSecond);
        } else {
            // Time in middle side
            u8g2.setFont(M42_12);
            int cTimeX = (DISPLAY_WIDTH - cTimeWidth) / 2;
            u8g2.drawStr(cTimeX, 48, currentTime);
        }

        // Day of week in bottom side
        u8g2.setFont(Superstar12);
        int cDaysX = (DISPLAY_WIDTH - cDaysWidth) / 2;
        u8g2.drawStr(cDaysX, 69, daysOfTheWeek[now.dayOfTheWeek()]);

        VoltageLine(Voltage);

        // Draw All Symbols
        u8g2.setDrawColor(0);
        u8g2.drawXBM(59, 75, Clock_Battery_simbol_width, Clock_Battery_simbol_height, Clock_Battery_simbol);
        u8g2.drawXBM(30, 89, Clock_Degree_width, Clock_Degree_height, Clock_Degree);
        u8g2.drawXBM(85, 91, Clock_Percentage_width, Clock_Percentage_height, Clock_Percentage);
        u8g2.setDrawColor(1);
        
    } while( u8g2.nextPage() );
  
    // rebuild the picture after some delay
    delay(50);

}
