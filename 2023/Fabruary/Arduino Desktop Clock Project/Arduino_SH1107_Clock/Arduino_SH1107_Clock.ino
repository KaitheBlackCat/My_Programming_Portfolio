/***************************************************************************************************************************************************
 * Arduino Desktop Clock With SH1107 128*128 OLED Display                                                                                          *
 *                                                                                                                                                 *
 * Connection                                                                                                                                      *
 *    The Module -- WeMos D1 Mini                                                                                                                  *
 *                                                                                                                                                 *
 * - DHT22                                                                                                                                         *
 *        Output -- GPIO2                                                                                                                          *
 *                                                                                                                                                 *
 * - RTC(DS1307)                                                                                                                                   *
 *           SDA -- GPIO4                                                                                                                          *
 *           SCL -- GPIO5                                                                                                                          *
 *                                                                                                                                                 *
 * - SH1107 OLED (https://ko.aliexpress.com/item/4000080365758.html?spm=a2g0o.order_list.order_list_main.35.21ef140fjuPiAt&gatewayAdapt=glo2kor)   *
 *     MOSI(SDA) -- GPIO13                                                                                                                         *
 *      CLK(SCL) -- GPIO14                                                                                                                         *
 *            DC -- GPIO0                                                                                                                          *
 *            CS -- GPIO15                                                                                                                         *
 *           RST -- GPIO16                                                                                                                         *
 *                                                                                                                                                 *
 * ⓒ 2023. Young-Chan Jung all rights reserved.                                                                                                   *
 *                                                                                                                                                 *
 * If you have any questions, ask me freely to piggy77567@gmail.com.                                                                               *
 *                                                                                                                                                 *
 ***************************************************************************************************************************************************/


#include "M42.h"
#include "Superstar.h"
#include "MyImages.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>

// RTClib has a compile error when it works on esp8266(Wemos, NodeMCU etc.)
// So change the RTC library made by Makuna 
// Changed at 2023.02.23
#include <RtcDS1307.h>

#define DHTPIN 2
#define DHTTYPE DHT22
#define ENABLE_SECOND true

U8G2_SH1107_SEEED_128X128_1_4W_HW_SPI u8g2(U8G2_R0, 15, 0, 16);
RtcDS1307<TwoWire> Rtc(Wire);
DHT dht(DHTPIN, DHTTYPE);

unsigned long startMillis = 0;

char Hum_char[6];
char Tem_char[6];
char currentTime[6];
char currentDate[11];
char currentSecond[3];

char daysOfTheWeek[7][12] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

void setup() {

    Serial.begin(9600);

    // RTC module setup routine by Makuna
    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) Rtc.SetDateTime(compiled); 
    }
    // RTC setup routine ends

    dht.begin();

    u8g2.begin();
    u8g2.setDrawColor(1);
    u8g2.setBitmapMode(0);
    u8g2.setFontMode(0);
    u8g2.setFontDirection(0);
    u8g2.setFontPosBottom();

    startMillis = millis();

}

void loop() {
   
    unsigned long currentMillis = millis();

    if (currentMillis - startMillis >= 2000L) {
        
        float Tem = dht.readTemperature();
        float Hum = dht.readHumidity();

        memset(Tem_char, 0, sizeof(Tem_char));
        memset(Hum_char, 0, sizeof(Hum_char));
    
        snprintf_P(Tem_char, sizeof(Tem_char), PSTR("%.1f"), Tem);
        snprintf_P(Hum_char, sizeof(Hum_char), PSTR("%.1f"), Hum);

        startMillis = currentMillis;
    }
    
    RtcDateTime now = Rtc.GetDateTime();

    memset(currentTime, 0, sizeof(currentTime));
    memset(currentDate, 0, sizeof(currentDate));
    memset(currentSecond, 0, sizeof(currentSecond));

    snprintf_P(currentTime, sizeof(currentTime), PSTR("%02u:%02u"), now.Hour(), now.Minute());
    snprintf_P(currentDate, sizeof(currentDate), PSTR("%04u.%02u.%02u"), now.Year(), now.Month(), now.Day());
    snprintf_P(currentSecond, sizeof(currentSecond), PSTR("%02u"), now.Second());

    u8g2.setFont(M42_12);
    int cTimeWidth = u8g2.getStrWidth(currentTime);

    u8g2.setFont(M42_6);
    int cSecondsWidth = u8g2.getStrWidth(currentSecond);
    
    // picture loop
    u8g2.firstPage();  
    do {

        // Date in upper side
        u8g2.setFont(Superstar12);
        int cDateX = (128 - u8g2.getStrWidth(currentDate)) / 2;
        u8g2.drawStr(cDateX, 20, currentDate);

        // Time in middle side
        u8g2.setFont(M42_12);
        int cTimeX = (128 - (cTimeWidth + cSecondsWidth + 2)) / 2;
        u8g2.drawStr(cTimeX, 48, currentTime);

        // Second in middle side
        u8g2.setFont(M42_6);
        int cSecdX = cTimeX + cTimeWidth + 2;
        u8g2.drawStr(cSecdX, 48, currentSecond);

        // Day of week in bottom side
        u8g2.setFont(Superstar12);
        int cDaysX = (128 - u8g2.getStrWidth(daysOfTheWeek[now.DayOfWeek()])) / 2;
        u8g2.drawStr(cDaysX, 69, daysOfTheWeek[now.DayOfWeek()]);

        // Show voltage line
        float Voltage = (((analogRead(A0) * 3.3) / 1024 ) * 2.0 + 0.0);

        Serial.print(F("Voltage : ")); Serial.println(Voltage);
        
        // Equation From https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery
        float vol_P = (123.0 - (123.0 / pow(1.0 + pow(Voltage / 3.7, 80), 0.165))) / 100.0;
        if (vol_P > 1.0) vol_P = 1.0;
        int lineLength = 58 * vol_P;
    
        // From middle to left
        u8g2.drawLine(58, 79, 58 - lineLength, 79);
        u8g2.drawLine(58, 80, 58 - lineLength, 80);
    
        // From middle to right
        u8g2.drawLine(69, 79, 69 + lineLength, 79);
        u8g2.drawLine(69, 80, 69 + lineLength, 80);

        // Show Temperatuer value
        u8g2.setFont(Superstar19);
        u8g2.drawStr(36 - (int)(u8g2.getStrWidth(Tem_char) / 2), 123, Tem_char);

        // Show Humidity value
        u8g2.drawStr(92 - (int)(u8g2.getStrWidth(Hum_char) / 2), 123, Hum_char);
        
        // Draw All Symbols
        u8g2.setDrawColor(0);
        u8g2.drawXBM(59, 75, Clock_Battery_simbol_width, Clock_Battery_simbol_height, Clock_Battery_simbol);
        u8g2.drawXBM(30, 89, Clock_Degree_width, Clock_Degree_height, Clock_Degree);
        u8g2.drawXBM(85, 91, Clock_Percentage_width, Clock_Percentage_height, Clock_Percentage);
        u8g2.setDrawColor(1);
        
    } while( u8g2.nextPage() );

}
