
#include "U8glib.h"
#include "RTClib.h"
#include "MyNewFonts.h"

#define CLOCK_INTERRUPT_PIN 2
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define ENABLE_SECOND true

//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);
U8GLIB_NHD27OLED_BW u8g(13, 11, 10, 9);
RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

void setup() {

    Serial.begin(9600);

    rtc.begin();

    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    rtc.disable32K();

    if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
        u8g.setColorIndex(255);     // white
    }
    else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
        u8g.setColorIndex(3);         // max intensity
    }
    else if ( u8g.getMode() == U8G_MODE_BW ) {
        u8g.setColorIndex(1);         // pixel on
    }
    else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
        u8g.setHiColorByRGB(255,255,255);
    }

    u8g.setFontPosBottom();
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
    u8g.firstPage();  
    do {
        //u8g.setFont(M42_6);
        //u8g.setFont(M42_12);
        //u8g.setFont(Superstar19);

        // Get all the font width first
        u8g.setFont(Superstar12);
        int cDateWidth = u8g.getStrWidth(currentDate);
        int cDaysWidth = u8g.getStrWidth(daysOfTheWeek[now.dayOfTheWeek()]);

        u8g.setFont(M42_12);
        int cTimeWidth = u8g.getStrWidth(currentTime);

        u8g.setFont(M42_6);
        int cSecondsWidth = u8g.getStrWidth(currentSecond);

        // Date in upper side
        u8g.setFont(Superstar12);
        int cDateX = (DISPLAY_WIDTH - cDateWidth) / 2;
        u8g.drawStr(cDateX, 20, currentDate);

        if (ENABLE_SECOND) {
            // Time in middle side
            u8g.setFont(M42_12);
            int cTimeX = (DISPLAY_WIDTH - (cTimeWidth + cSecondsWidth + 2)) / 2;
            u8g.drawStr(cTimeX, 48, currentTime);

            // Second in middle side
            u8g.setFont(M42_6);
            int cSecdX = cTimeX + cTimeWidth + 2;
            u8g.drawStr(cSecdX, 48, currentSecond);
        } else {
            // Time in middle side
            u8g.setFont(M42_12);
            int cTimeX = (DISPLAY_WIDTH - cTimeWidth) / 2;
            u8g.drawStr(cTimeX, 48, currentTime);
        }

        // Day of week in bottom side
        //u8g.setFont(Superstar12);
        //int cDaysX = (DISPLAY_WIDTH - cDaysWidth) / 2;
        //u8g.drawStr(cDaysX, 69, daysOfTheWeek[now.dayOfTheWeek()]);
        
    } while( u8g.nextPage() );
  
    // rebuild the picture after some delay
    delay(50);

}
