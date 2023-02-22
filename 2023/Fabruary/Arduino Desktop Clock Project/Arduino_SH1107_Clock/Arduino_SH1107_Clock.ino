
#include "RTClib.h"
#include "M42.h"
#include "Superstar.h"

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

U8G2_SH1107_PIMORONI_128X128_1_4W_HW_SPI u8g2(U8G2_R0, CS, DC, RST);

#define CLOCK_INTERRUPT_PIN 2
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define ENABLE_SECOND true

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

void setup() {

    Serial.begin(9600);

    rtc.begin();

    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    rtc.disable32K();

    u8g2.begin();
    u8g2.setDrawColor(1);
    u8g2.setFontMode(0);
    u8g2.setFontDirection(0);
    u8g2.setFontPosBottom();
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
        //u8g2.setFont(M42_6);
        //u8g2.setFont(M42_12);
        //u8g2.setFont(Superstar19);

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
        
    } while( u8g2.nextPage() );
  
    // rebuild the picture after some delay
    delay(50);

}
