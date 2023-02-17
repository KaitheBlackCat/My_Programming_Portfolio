#include "U8glib.h"


U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);    // Fast I2C / TWI 


void drawGauge(int center_x = 0, int center_y = 0, int radius = 30, int theta = 45, int numOfMarks = 2, int numOfSmallMarks = 1, int min_val = 0, int max_val = 100) {
  
  int calculated_x = radius * cos(radians(90 - theta));
  int calculated_y = radius * sin(radians(90 - theta));

  int lengthOfMarks = radius * 2 / 15;
  int lengthOfSmallMarks = lengthOfMarks / 2;

  float angle_between_marks = (360 - theta * 2) / (numOfMarks - 1);
  float angle_between_small_marks = angle_between_marks / numOfSmallMarks;
  
  for (int i = 0; i < numOfSmallMarks * (numOfMarks - 1) + numOfMarks; i++) {
    u8g.drawLine(center_x, center_y, center_x - int((radius + lengthOfSmallMarks + 2) * sin(radians(theta + angle_between_small_marks * i))), center_y + int((radius + lengthOfSmallMarks + 2) * cos(radians(theta + angle_between_small_marks * i))));
  }

  u8g.setColorIndex(0);
  u8g.drawDisc(center_x, center_y, radius + 1);
  u8g.setColorIndex(1);

  u8g.drawCircle(center_x, center_y, radius);

  u8g.setColorIndex(0);
  u8g.drawBox(center_x - calculated_x, center_y + calculated_y, 2 * calculated_x + 1, radius - calculated_y + 1);
  u8g.setColorIndex(1);

  u8g.setFont(u8g_font_04b_24);

  for (int i = 0; i < numOfMarks; i++) {
    u8g.drawLine(center_x, center_y, center_x - int(radius * sin(radians(theta + angle_between_marks * i))), center_y + int(radius * cos(radians(theta + angle_between_marks * i))));
  }

  u8g.setColorIndex(0);
  u8g.drawDisc(center_x, center_y, radius - lengthOfMarks - 1);
  u8g.setColorIndex(1);

  for (int i = 0; i < numOfMarks; i++) {
    char MarkNum_char[3] = {};
    int MarkNum = min_val + i * (max_val - min_val)/(numOfMarks - 1);
    float T_ANGLE = theta + angle_between_marks * i;
    int text_x = center_x - int((radius - 9) * sin(radians(T_ANGLE)));
    int text_y = center_y + int((radius - 9) * cos(radians(T_ANGLE)));
    itoa(MarkNum, MarkNum_char, 10);
    u8g.drawStr(text_x - 3, text_y + 3, MarkNum_char);
  }

}

void drawNiddle(int center_x = 0, int center_y = 0, int radius = 30, int outVal = 0, int theta = 45, int min_val = 0, int max_val = 100) {
  int angle = map(outVal, min_val, max_val, theta, 360 - theta);
  
  int shortNiddleLength = radius / 3;
  int longNiddleLength = radius * 0.6;

  int start_x = 0;
  int start_y = 0;
  int end_x = 0;
  int end_y = 0;

  start_x = longNiddleLength  * -sin(radians(angle)) + center_x;
  start_y = longNiddleLength  *  cos(radians(angle)) + center_y;
  end_x   = shortNiddleLength * -sin(radians(angle + 180)) + center_x;
  end_y   = shortNiddleLength *  cos(radians(angle + 180)) + center_y;

  u8g.drawLine(start_x, start_y, end_x, end_y);

  u8g.setColorIndex(0);
  u8g.drawDisc(center_x, center_y, 3);
  u8g.setColorIndex(1);
  u8g.drawCircle(center_x, center_y, 2);
}

void setup(void) {
  u8g.setColorIndex(1);         // pixel on
  u8g.setFontPosCenter();
}

int testValue = 25;

void loop(void) {
  // picture loop
  u8g.firstPage();  
  do {
    drawGauge(36, 36, 30, 45, 7, 4, -20, 40);
    drawNiddle(36, 36, 30, testValue, 45, -20, 40);
  } while( u8g.nextPage() );
  
  // rebuild the picture after some delay
  delay(500);
}
