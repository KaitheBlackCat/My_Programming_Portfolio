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

#include "ssd1325_spi.h"
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <stdlib.h>

#ifdef SPI_HAS_TRANSACTION
SPISettings oledspi = SPISettings(4000000, MSBFIRST, SPI_MODE0);
#else
#define ADAFRUIT_SSD1325_SPI SPI_CLOCK_DIV2 //!< Spi clock
#endif


SSD1325::SSD1325(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS) {
  sid = SID;
  sclk = SCLK;
  dc = DC;
  rst = RST;
  cs = CS;
}

void SSD1325::init() {
	// set pin directions
	if (sclk != -1) {
		pinMode(sid, OUTPUT);
		pinMode(sclk, OUTPUT);
#ifdef __AVR__
		clkport = portOutputRegister(digitalPinToPort(sclk));
		clkpinmask = digitalPinToBitMask(sclk);
		mosiport = portOutputRegister(digitalPinToPort(sid));
		mosipinmask = digitalPinToBitMask(sid);
#endif
	}
	else {
		// hardware SPI
		SPI.begin();
	}

	pinMode(dc, OUTPUT);
	pinMode(rst, OUTPUT);
	pinMode(cs, OUTPUT);

	digitalWrite(rst, HIGH);
	// VDD (3.3V) goes high at start, lets just chill for a ms
	delay(1);
	// bring reset low
	digitalWrite(rst, LOW);
	// wait 10ms
	delay(10);
	// bring out of reset
	digitalWrite(rst, HIGH);

	// Serial.println("reset");
	delay(500);
	command(SSD1325_DISPLAYOFF);   /* display off */
	command(SSD1325_SETCLOCK);     /* set osc division */
	command(0xF1);                 /* 145 */
	command(SSD1325_SETMULTIPLEX); /* multiplex ratio */
	command(0x3f);                 /* duty = 1/64 */
	command(SSD1325_SETOFFSET);    /* set display offset --- */
	command(0x4C);                 /* 76 */
	command(SSD1325_SETSTARTLINE); /*set start line */
	command(0x00);                 /* ------ */
	command(SSD1325_MASTERCONFIG); /*Set Master Config DC/DC Converter*/
	command(0x02);
	command(SSD1325_SETREMAP); /* set segment remap------ */
	command(0x56);
	// command(0x52);
	command(SSD1325_SETCURRENT + 0x2); /* Set Full Current Range */
	command(SSD1325_SETGRAYTABLE);
	command(0x01);
	command(0x11);
	command(0x22);
	command(0x32);
	command(0x43);
	command(0x54);
	command(0x65);
	command(0x76);

	command(SSD1325_SETCONTRAST); /* set contrast current */
	command(0x7F);                // max!

	command(SSD1325_SETROWPERIOD);
	command(0x51);
	command(SSD1325_SETPHASELEN);
	command(0x55);
	command(SSD1325_SETPRECHARGECOMP);
	command(0x02);
	command(SSD1325_SETPRECHARGECOMPENABLE);
	command(0x28);
	command(SSD1325_SETVCOMLEVEL); // Set High Voltage Level of COM Pin
	command(0x1C);                 //?
	command(SSD1325_SETVSL);       // set Low Voltage Level of SEG Pin
	command(0x0D | 0x02);

	command(SSD1325_NORMALDISPLAY); /* set display mode */
	/* Clear Screen */
	// command(0x23); /*set graphic acceleration commmand */
	// command(SSD1325_GFXACCEL);
	// command(SSD1325_DRAWRECT); /* draw rectangle */
	// command(0x00); /* Gray Scale Level 1 */
	// command(0x00); /* Gray Scale Level 3 & 2 */
	// command(0x3f); /* Gray Scale Level 3 & 2 */
	// command(0x3f); /* Gray Scale Level 3 & 2 */
	// command(0x00); /* Gray Scale Level 3 & 2 */

	command(SSD1325_DISPLAYON); /* display ON */
}

void SSD1325::resetDisplay(void)
{
  displayOff();
  clear();
  display();
  displayOn();
}


void SSD1325::displayOn(void)
{
    command(0xaf);        //display on
}

void SSD1325::displayOff(void)
{
  command(0xae);		//display off
}

//void SSD1325::setContrast(char contrast) {
//  sendCommand(0x81);
//  sendCommand(contrast);  
//}
//void SSD1325::flipScreenVertically() {
//  sendCommand(0xA0 | 0x1);      //SEGREMAP   //Rotate screen 180 deg
//  
// sendCommand(0xC8);            //COMSCANDEC  Rotate screen 180 Deg
//}


void SSD1325::invertDisplay(uint8_t i) {
	if (i) {
		command(SSD1325_INVERTDISPLAY);
	}
	else {
		command(SSD1325_NORMALDISPLAY);
	}
}

void SSD1325::clear(void) {
    memset(buffer, 0, (128*64 / 8));
}


void SSD1325::spixfer(uint8_t x) {
	if (sclk == -1) {
		SPI.transfer(x);
		return;
	}
	// software spi
	// Serial.println("Software SPI");

	for (uint8_t bit = 0x80; bit; bit >>= 1) {
#if defined(AVR)
		*clkport &= ~clkpinmask;
		if (x & bit)
			*mosiport |= mosipinmask;
		else
			*mosiport &= ~mosipinmask;
		*clkport |= clkpinmask;
#else
		digitalWrite(sclk, LOW);
		if (x & bit)
			digitalWrite(sid, HIGH);
		else
			digitalWrite(sid, LOW);
		digitalWrite(sclk, HIGH);
#endif
	}
}


void SSD1325::display(void) {

	command(0x15); /* set column address */
	command(0x00); /* set column start address */
	command(0x3f); /* set column end address */
	command(0x75); /* set row address */
	command(0x00); /* set row start address */
	command(0x3f); /* set row end address */

	if (sclk == -1) {
#ifdef SPI_HAS_TRANSACTION
		SPI.beginTransaction(oledspi);
#else
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(ADAFRUIT_SSD1325_SPI);
#endif
	}

	digitalWrite(cs, HIGH);
	digitalWrite(dc, HIGH);
	digitalWrite(cs, LOW);
	delay(1);

	for (uint16_t x = 0; x < 128; x += 2) {
		for (uint16_t y = 0; y < 64; y += 8) { // we write 8 pixels at once
			uint8_t left8 = buffer[y * 16 + x];
			uint8_t right8 = buffer[y * 16 + x + 1];
			for (uint8_t p = 0; p < 8; p++) {
				uint8_t d = 0;
				if (left8 & (1 << p))
					d |= 0xF0;
				if (right8 & (1 << p))
					d |= 0x0F;
				spixfer(d);
			}
		}
	}

	digitalWrite(cs, HIGH);

#ifdef SPI_HAS_TRANSACTION
	if (sclk == -1)
		SPI.endTransaction(); // release the SPI bus
#endif


}

void SSD1325::setPixel(int x, int y) {
  if (x >= 0 && x < 128 && y >= 0 && y < 64) {
     
     switch (myColor) {
      case WHITE:   buffer[x+ (y/8)*128] |=  (1 << (y&7)); break;
      case BLACK:   buffer[x+ (y/8)*128] &= ~(1 << (y&7)); break; 
      case INVERSE: buffer[x+ (y/8)*128] ^=  (1 << (y&7)); break; 
    }
  }
}

void SSD1325::setChar(int x, int y, unsigned char data) {
  for (int i = 0; i < 8; i++) {
    if (bitRead(data, i)) {
     setPixel(x,y + i);
    }   
  }   
}

void SSD1325::drawString(int x, int y, String text) {
  unsigned char currentByte;
  int charX, charY;
  int currentBitCount;
  int charCode;
  int currentCharWidth;
  int currentCharStartPos;
  int cursorX = 0;
  int numberOfChars = pgm_read_byte(myFontData + CHAR_NUM_POS);
  // iterate over string
  int firstChar = pgm_read_byte(myFontData + FIRST_CHAR_POS);
  int charHeight = pgm_read_byte(myFontData + HEIGHT_POS);
  int currentCharByteNum = 0;
  int startX = 0;
  int startY = y;

  if (myTextAlignment == TEXT_ALIGN_LEFT) {
    startX = x;
  } else if (myTextAlignment == TEXT_ALIGN_CENTER) {
    int width = getStringWidth(text);
    startX = x - width / 2;
  } else if (myTextAlignment == TEXT_ALIGN_RIGHT) {
    int width = getStringWidth(text);
    startX = x - width;
  }

  for (int j=0; j < text.length(); j++) {

    charCode = text.charAt(j)-0x20;

    currentCharWidth = pgm_read_byte(myFontData + CHAR_WIDTH_START_POS + charCode);
    // Jump to font data beginning
    currentCharStartPos = CHAR_WIDTH_START_POS + numberOfChars;
    
    for (int m = 0; m < charCode; m++) {

      currentCharStartPos += pgm_read_byte(myFontData + CHAR_WIDTH_START_POS + m)  * charHeight / 8 + 1;
    }

    currentCharByteNum = ((charHeight * currentCharWidth) / 8) + 1;
    // iterate over all bytes of character
    for (int i = 0; i < currentCharByteNum; i++) {
      
      currentByte = pgm_read_byte(myFontData + currentCharStartPos + i);
      //Serial.println(String(charCode) + ", " + String(currentCharWidth) + ", " + String(currentByte));
      // iterate over all bytes of character
      for(int bit = 0; bit < 8; bit++) {
         //int currentBit = bitRead(currentByte, bit);
         
         currentBitCount = i * 8 + bit;

         charX = currentBitCount % currentCharWidth;
         charY = currentBitCount / currentCharWidth;

         if (bitRead(currentByte, bit)) {
          //Serial.println(String(charX) + ", " + String(charY));
          setPixel(startX + cursorX + charX, startY + charY); 
          //setPixel(charX, charY);
         }

      } 
      yield();     
    }
    cursorX += currentCharWidth;

  }
}

void SSD1325::drawStringMaxWidth(int x, int y, int maxLineWidth, String text) {
  int currentLineWidth = 0;
  int startsAt = 0;
  int endsAt = 0;
  int lineNumber = 0;
  char currentChar = ' ';
  int lineHeight = pgm_read_byte(myFontData + HEIGHT_POS);
  String currentLine = "";
  for (int i = 0; i < text.length(); i++) {
    currentChar = text.charAt(i);
    if (currentChar == ' ' || currentChar == '-') {
      String lineCandidate = text.substring(startsAt, i);
      if (getStringWidth(lineCandidate) <= maxLineWidth) {
        endsAt = i;
      } else {

        drawString(x, y + lineNumber * lineHeight, text.substring(startsAt, endsAt));
        lineNumber++;
        startsAt = endsAt + 1;
      }
    }
    
  }
  drawString(x, y + lineNumber * lineHeight, text.substring(startsAt));
}

int SSD1325::getStringWidth(String text) {
  int stringWidth = 0;
  char charCode;
  for (int j=0; j < text.length(); j++) {
    charCode = text.charAt(j)-0x20;
    stringWidth += pgm_read_byte(myFontData + CHAR_WIDTH_START_POS + charCode);
  }
  return stringWidth;
}

void SSD1325::setTextAlignment(int textAlignment) {
  myTextAlignment = textAlignment;
}

void SSD1325::setFont(const char *fontData) {
  myFontData = fontData;
}

void SSD1325::drawBitmap(int x, int y, int width, int height, const char *bitmap) {
  for (int i = 0; i < width * height / 8; i++ ){
    unsigned char charColumn = 255 - pgm_read_byte(bitmap + i);
    for (int j = 0; j < 8; j++) { 
      int targetX = i % width + x;
      int targetY = (i / (width)) * 8 + j + y;
      if (bitRead(charColumn, j)) {
        setPixel(targetX, targetY);  
      }
    }
  }  
}

void SSD1325::setColor(int color) {
  myColor = color;  
}

void SSD1325::drawRect(int x, int y, int width, int height) {
  for (int i = x; i < x + width; i++) {
    setPixel(i, y);
    setPixel(i, y + height);    
  }
  for (int i = y; i < y + height; i++) {
    setPixel(x, i);
    setPixel(x + width, i);  
  }
}

void SSD1325::fillRect(int x, int y, int width, int height) {
  for (int i = x; i < x + width; i++) {
    for (int j = 0; j < y + height; j++) {
      setPixel(i, j);
    }
  }
}

void SSD1325::drawXbm(int x, int y, int width, int height, const char *xbm) {
  if (width % 8 != 0) {
    width =  ((width / 8) + 1) * 8;
  }
  for (int i = 0; i < width * height / 8; i++ ){
    unsigned char charColumn = pgm_read_byte(xbm + i);
    for (int j = 0; j < 8; j++) { 
      int targetX = (i * 8 + j) % width + x;
      int targetY = (8 * i / (width)) + y;
      if (bitRead(charColumn, j)) {
        setPixel(targetX, targetY);  
      }
    }
  }    
}

void SSD1325::command(uint8_t c) {
	digitalWrite(cs, HIGH);
	digitalWrite(dc, LOW);
	delay(1);
	if (sclk == -1) {
#ifdef SPI_HAS_TRANSACTION
		SPI.beginTransaction(oledspi);
#else
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(ADAFRUIT_SSD1325_SPI);
#endif
	}

	digitalWrite(cs, LOW);
	spixfer(c);
	digitalWrite(cs, HIGH);

#ifdef SPI_HAS_TRANSACTION
	if (sclk == -1)
		SPI.endTransaction(); // release the SPI bus
#endif
}


void SSD1325::sendInitCommands(void)
{
  // Init sequence from https://github.com/adafruit/Adafruit_SSD1325/blob/master/Adafruit_SSD1325.cpp
  command(0xae);		        //display off
  command(0xa6);            //Set Normal Display (default)
                                // Init sequence for 128x64 OLED module
  command(0xAE);            //DISPLAYOFF
  command(0xD5);            //SETDISPLAYCLOCKDIV
  command(0x80);            // the suggested ratio 0x80
  command(0xA8);            //SSD1325_SETMULTIPLEX
  command(0x3F);
  command(0xD3);            //SETDISPLAYOFFSET
  command(0x0);             //no offset
  command(0x40 | 0x0);      //SETSTARTLINE
  command(0x8D);            //CHARGEPUMP
  command(0x14);
  command(0x20);             //MEMORYMODE
  command(0x00);             //0x0 act like ks0108
  
  //sendCommand(0xA0 | 0x1);      //SEGREMAP   //Rotate screen 180 deg
  command(0xA0);
  
  //sendCommand(0xC8);            //COMSCANDEC  Rotate screen 180 Deg
  command(0xC0);
  
  command(0xDA);            //0xDA
  command(0x12);           //COMSCANDEC
  command(0x81);           //SETCONTRAS
  command(0xCF);           //
  command(0xd9);          //SETPRECHARGE 
  command(0xF1); 
  command(0xDB);        //SETVCOMDETECT                
  command(0x40);
  command(0xA4);        //DISPLAYALLON_RESUME        
  command(0xA6);        //NORMALDISPLAY             

  command(0x2e);            // stop scroll
  //----------------------------REVERSE comments----------------------------//
  //  sendCommand(0xa0);		//seg re-map 0->127(default)
  //  sendCommand(0xa1);		//seg re-map 127->0
  //  sendCommand(0xc8);
  //  delay(1000);
  //----------------------------REVERSE comments----------------------------//
  // sendCommand(0xa7);  //Set Inverse Display  
  // sendCommand(0xae);		//display off
  command(0x20);            //Set Memory Addressing Mode
  command(0x00);            //Set Memory Addressing Mode ab Horizontal addressing mode
  //sendCommand(0x02);         // Set Memory Addressing Mode ab Page addressing mode(RESET)  
  
}

void SSD1325::nextFrameTick() {
    myFrameTick++;
    if (myFrameTick==myFrameWaitTicks && myFrameState == 0 || myFrameTick==myFrameTransitionTicks && myFrameState == 1) {
      myFrameState = (myFrameState + 1) %  2;
      if (myFrameState==FRAME_STATE_FIX) {
        myCurrentFrame = (myCurrentFrame + 1) % myFrameCount; 
      } 
      myFrameTick = 0; 
    }
    drawIndicators(myFrameCount, myCurrentFrame);
    
    switch(myFrameState) {
    case 0:
      (*myFrameCallbacks[myCurrentFrame])(0, 0);
      break;
    case 1:
      (*myFrameCallbacks[myCurrentFrame])(-128 * myFrameTick / myFrameTransitionTicks, 0);
      (*myFrameCallbacks[(myCurrentFrame + 1) % myFrameCount])(-128 * myFrameTick / myFrameTransitionTicks + 128, 0);
      break;
    }
    
}
void SSD1325::drawIndicators(int frameCount, int activeFrame) {
  for (int i = 0; i < frameCount; i++) {
    const char *xbm;
    if (activeFrame == i) {
       xbm = active_bits;
    } else {
       xbm = inactive_bits;  
    }
    drawXbm(64 - (12 * frameCount / 2) + 12 * i,56, 8, 8, xbm);
  }  
}
void SSD1325::setFrameCallbacks(int frameCount, void (*frameCallbacks[])(int x, int y)) {
  myFrameCount = frameCount;
  myFrameCallbacks = frameCallbacks;
}

void SSD1325::setFrameWaitTicks(int frameWaitTicks) {
  myFrameWaitTicks = frameWaitTicks; 
}
void SSD1325::setFrameTransitionTicks(int frameTransitionTicks) {
  myFrameTransitionTicks = frameTransitionTicks; 
}
int SSD1325::getFrameState() {
  return myFrameState; 
}
