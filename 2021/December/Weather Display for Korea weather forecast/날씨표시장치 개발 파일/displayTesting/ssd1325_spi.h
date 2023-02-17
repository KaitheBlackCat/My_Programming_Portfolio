/**The MIT License (MIT)

Copyright (c) 2015 by Daniel Eichhorn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

See more at http://blog.squix.ch

Credits for parts of this code go to Mike Rankin. Thank you so much for sharing!
*/
/*
 * 이 코드는 본래 SSD1306 I2C OLED를 위해 설계된 UI 라이브러리 입니다.
 * 제가 이 코드를 SPI를 사용하는 SSD1325에 맞도록 코드를 바꾸었고, 그 과정에서
 * Adafruit SSD1325 라이브러리에서 사용하는 코드도 포함했습니다. 
 * 
 * 혹시 라이브러리가 이해되지 않으면 제 블로그에 질문해주세요.
 * 사실 저도 이해가 잘 가지 않습니다.
 * 
 * 기존 주석은 모두 한글로 바꾸어놓겠습니다.
 */

 
#pragma once

#include <Arduino.h>
#include "SSD1325Fonts.h"


#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3
#define CHAR_WIDTH_START_POS 4

#define TEXT_ALIGN_LEFT 0
#define TEXT_ALIGN_CENTER 1
#define TEXT_ALIGN_RIGHT 2

#define active_width 8
#define active_height 8
const char active_bits[] PROGMEM = {
   0x00, 0x18, 0x3c, 0x7e, 0x7e, 0x3c, 0x18, 0x00 }; //화면 맨 아래에 표시되는 조그마한 원입니다. 현재 화면이 몇 번째인지 표시하는 UI죠. active는 좀 큰 원을, inactive는 좀 작은 원을 표시합니다.  

#define inactive_width 8
#define inactive_height 8
const char inactive_bits[] PROGMEM = {
   0x00, 0x0, 0x0, 0x18, 0x18, 0x0, 0x0, 0x00 };

#define SSD1325_128_64 // ssd1325 드라이버로 128x32 크기의 OLED를 쓰신다면 64를 32로 고치세요

#if defined SSD1325_128_64 && defined SSD1325_128_32
#error "Only one SSD1325 display can be specified at once in SSD1325.h"
#endif
#if !defined SSD1325_128_64 && !defined SSD1325_128_32
#error "At least one SSD1325 display must be specified in SSD1325.h"
#endif

#if defined SSD1325_128_64
#define SSD1325_LCDWIDTH 128 //!< Width w/SSD1325_128_64 화면 너비 크기 정의
#define SSD1325_LCDHEIGHT 64 //!< Height w/SSD1325_128_64 화면 높이 크기 정의
#endif
#if defined SSD1325_128_32
#define SSD1325_LCDWIDTH 128 //!< Width w/SSD1325_128_32 화면 너비 크기 정의
#define SSD1325_LCDHEIGHT 32 //!< Height w/SSD1325_128_32 화면 높이 크기 정의
#endif


//아래의 주소값은 Adafruit SSD1325 라이브러리에서 따왔습니다.
//주소값 기준은 아무래도 ssd1325 데이터시트에서 온 듯합니다.

#define SSD1325_SETCOLADDR 0x15  //!< 열 주소 설정
#define SSD1325_SETROWADDR 0x75  //!< 행 주소 설정
#define SSD1325_SETCONTRAST 0x81 //!< 현재 대조값 설정
#define SSD1325_SETCURRENT 0x84  //!< 전류값 범위 설정

#define SSD1325_SETREMAP 0xA0      //!< re-map 설정
#define SSD1325_SETSTARTLINE 0xA1  //!< 디스플레이 시작 라인 설정
#define SSD1325_SETOFFSET 0xA2     //!< 디스플레이 오프셋 설정
#define SSD1325_NORMALDISPLAY 0xA4 //!< 디스플레이 모드 설정(보통모드)
#define SSD1325_DISPLAYALLON 0xA5  //!< 디스플레이 on으로 하는 값
#define SSD1325_DISPLAYALLOFF 0xA6 //!< 디스플레이 off로 하는 값
#define SSD1325_INVERTDISPLAY 0xA7 //!< 디스플레이 반전
#define SSD1325_SETMULTIPLEX 0xA8  //!< 멀티플렉스 비율
#define SSD1325_MASTERCONFIG 0xAD  //!< 마스터 콘피그
#define SSD1325_DISPLAYOFF 0xAE    //!< 디스플레이 off
#define SSD1325_DISPLAYON 0xAF     //!< 디스플레이 on

#define SSD1325_SETPRECHARGECOMPENABLE                                         \
  0xB0                                   //!< 사전 충전 보상 활성화 설정
#define SSD1325_SETPHASELEN 0xB1         //!< 페이스 길이 설정
#define SSD1325_SETROWPERIOD 0xB2        //!< 행 주기 설정
#define SSD1325_SETCLOCK 0xB3            //!< 디스플레이 클럭 분할 비율 설정
#define SSD1325_SETPRECHARGECOMP 0xB4    //!< 사전 충전 설정
#define SSD1325_SETGRAYTABLE 0xB8        //!< 그레이스케일 표 설정
#define SSD1325_SETPRECHARGEVOLTAGE 0xBC //!< 사전 충전 전압 설정
#define SSD1325_SETVCOMLEVEL 0xBE        //!< Vcomh 전압 설정
#define SSD1325_SETVSL 0xBF              //!< segment low voltage (VSL) 설정

#define SSD1325_GFXACCEL 0x23 //!< 그래픽 가속 명령 옵션
#define SSD1325_DRAWRECT 0x24 //!< 직사각형 그리기
#define SSD1325_COPY 0x25     //!< 복사


class SSD1325 {

private:
   uint8_t buffer[128 * 64 / 8];
   int myFrameState = 0;
   int myFrameTick = 0;
   int myCurrentFrame = 0;
   int myFrameCount = 0;
   int myFrameWaitTicks = 100;
   int myFrameTransitionTicks = 25;
   int myTextAlignment = TEXT_ALIGN_LEFT;
   int myColor = WHITE;
   void spixfer(uint8_t x);
   int8_t sid, sclk, dc, rst, cs;
   const char *myFontData = ArialMT_Plain_10;
   void (**myFrameCallbacks)(int x, int y);



public:
   // Create the display object // 디스플레이 객체 설정
   SSD1325(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS);

   // Initialize the display // 시작
   void init();

   // Cycle through the initialization // 리셋
   void resetDisplay(void);

   // Connect again to the display // 재연결
   void reconnect(void);

   // Turn the display on // 화면 켜기
   void displayOn(void);

   // Turn the display offs // 화면 끄기
   void displayOff(void);

   // Clear the local pixel buffer // 화면 지우기
   void clear(void);

   // Write the buffer to the display memory // 화면 띄우기
   void display(void);

   // 화면 반전시키기 (검 -> 흰, 흰 -> 검)
   void invertDisplay(uint8_t i);

   // Turn the display upside down // 화면 뒤집기
   void flipScreenVertically();

   // Send a command to the display (low level function) // SPI 통신으로 명령 내리기
   void command(uint8_t c);

   // Send all the init commands // 시작 명령 내리기. init 하나로 충분하니 안써도 됩니다.
   void sendInitCommands(void);

   // Draw a pixel at given position // 픽셀 띄우기
   void setPixel(int x, int y);

   // Draw 8 bits at the given position // 글자 하나 띄우기
   void setChar(int x, int y, unsigned char data);

   // Draw the border of a rectangle at the given location // 사각형 그리기
   void drawRect(int x, int y, int width, int height);

   // Fill the rectangle // 원 그리기
   void fillRect(int x, int y, int width, int height);

   // Draw a bitmap with the given dimensions // 비트맵 이미지 띄우기
   void drawBitmap(int x, int y, int width, int height, const char *bitmap);

   // Draw an XBM image with the given dimensions // xbm 이미지 띄우기
   void drawXbm(int x, int y, int width, int height, const char *xbm);

   // Sets the color of all pixel operations // 흰색과 검정색 중 하나 고를 수 있음.
   void setColor(int color);

   // Draws a string at the given location // 문자열 띄우기
   void drawString(int x, int y, String text);

   // 지정된 위치에 최대 너비의 문자열을 그립니다.
   // 만약에 주어진 문자열이 설정된 길이보다 길다면
   // 공백(' ')이 나오거나 대시('-')가 나오는 곳에서 텍스트가 다음 줄로 줄바꿈 합니다.
   void drawStringMaxWidth(int x, int y, int maxLineWidth, String text); 

   // 현재 설정된 폰트를 기준으로 문자열의 길이를 반환합니다.
   // 이때 길이는 문자 수가 아니라 픽셀 수 입니다.
   int getStringWidth(String text);

   // 전체 문자열에서 어느 위치를 기준으로 텍스트를 배열하는지를 결정합니다.
   // 한글에서 문서 정렬 선택하는 것과 똑같습니다. 가능한 것은 왼쪽 정렬, 중앙 정렬, 오른쪽 정렬입니다.
   // TEXT_ALIGN_LEFT, TEXT_ALIGN_CENTER, TEXT_ALIGN_RIGHT
   void setTextAlignment(int textAlignment);

   // 폰트를 설정합니다. 이것 외에도 다른 폰트를 사용하고 싶다면
   // SSD1306Fonts.h에 더 추가하면 됩니다. 일단 가능한 폰트는 다음과 같습니다.:
   // ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
   void setFont(const char *fontData);

   // void method(x,y) 형식의 콜백 메서드를 설정합니다.
   void setFrameCallbacks(int frameCount, void (*frameCallbacks[])(int x, int y));

   // 다음 눈금으로 이동하도록 프레임워크에 지시합니다.
   // 현재 볼 수 있는 프레임 콜백은 눈금당 한 번씩 호출됩니다.
   void nextFrameTick(void);

   // 프레임 표시기를 그립니다. 일반적인 설정에서는 프레임워크가 이 작업을 수행합니다.
   void drawIndicators(int frameCount, int activeFrame);

   // 한 프레임에 얼마나 머무를 것인지 결정합니다. 10이 1초를 의미합니다.
   // 전환 효과는 여기 없습니다.
   void setFrameWaitTicks(int frameWaitTicks);

   // 전환하는데 얼마나 걸릴 것인지 결정합니다. 10이 1초를 의미합니다.
   void setFrameTransitionTicks(int frameTransitionTicks);

   // 현재 프레임 전환중인지 아닌지 나타내줍니다.
   // Possible values: FRAME_STATE_FIX, FRAME_STATE_TRANSITION
   // 이를 통해서 현재 프레임이 전환 중인지 아닌지를 판별할 수 있습니다.
   int getFrameState();


   const int FRAME_STATE_FIX = 0;
   const int FRAME_STATE_TRANSITION = 1;

};
