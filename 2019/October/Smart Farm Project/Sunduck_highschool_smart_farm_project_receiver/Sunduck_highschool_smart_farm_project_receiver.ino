#include <U8glib.h>
#include <SoftwareSerial.h>

SoftwareSerial LORA(2,3);

U8GLIB_NHD27OLED_BW u8g(13, 11, 10, 9);

const int NUMBER_OF_FIELDS = 6;
int fieldIndex = 0;
float values[NUMBER_OF_FIELDS];


char forPrint[10];

String rxstring = "";
boolean bRxStart = false;

String temP = "";
String humI = "";
String Uvi = "";
String Lux = "";
String soilState = "";
String soilTemp = "";

void deleteIt() {
  for (int i = 0; i < 10; i++) {
    forPrint[i] = NULL;
  }
}

void drawAir(void) {
  u8g.setFont(u8g_font_courB10r);
  u8g.drawStr( 21, 10, "Air Status");
  
  u8g.setFont(u8g_font_6x12);
  u8g.drawStr( 0, 20, "Temperature : ");
  deleteIt();
  temP.toCharArray(forPrint, temP.length());
  u8g.drawStr(80, 20, forPrint);
  
  u8g.drawStr(0, 30, "Humidity : ");
  deleteIt();
  humI.toCharArray(forPrint, humI.length());
  u8g.drawStr(62, 30, forPrint);
  
  u8g.drawStr(0, 40, "UV index : ");
  deleteIt();
  Uvi.toCharArray(forPrint, Uvi.length());
  u8g.drawStr(62, 40, forPrint);

  u8g.drawStr(0, 50, "Light : ");
  deleteIt();
  Lux.toCharArray(forPrint, Lux.length()+1);
  u8g.drawStr(43, 50, forPrint);

}

void drawSoil(void) {
  u8g.setFont(u8g_font_courB10r);
  u8g.drawStr( 16, 10, "Soil Status");
  
  u8g.setFont(u8g_font_6x12);
  u8g.drawStr( 0, 20, "Temperature : ");
  deleteIt();
  soilTemp.toCharArray(forPrint, soilTemp.length());
  u8g.drawStr(80, 20, forPrint);
  
  u8g.drawStr(0, 30, "Humidity : ");
  deleteIt();
  int index = soilState.toInt();
  if(index == 0) u8g.drawStr(62, 30, "Watered");
  else if(index == 1) u8g.drawStr(62, 30, "Wet");
  else if(index == 2) u8g.drawStr(62, 30, "Dry");
  else u8g.drawStr(62, 30, "?Error?");

}

void setup(void) {
    u8g.setColorIndex(1);
    Serial.begin(9600);
    LORA.begin(9600);
}

void dataParsing(String str) {

  int f = str.indexOf(",");
  int s = str.indexOf(",", f + 1);
  int t = str.indexOf(",", s + 1);
  int four = str.indexOf(",", t + 1);
  int five = str.indexOf(",", four + 1);
  int six = str.indexOf(",", five + 1);
  
  temP = str.substring(0, f);
  humI = str.substring(f+1, s);
  Uvi = str.substring(s+1, t);
  Lux = str.substring(t+1, four);
  soilState = str.substring(four+1, five);
  soilTemp = str.substring(five+1, six);
  
}

void loop(void) {
  u8g.firstPage();
  do {
    drawAir();
  } while( u8g.nextPage() );
  delay(2000);
  u8g.firstPage();
  do {
    drawSoil();
  } while( u8g.nextPage() );
  delay(2000);
  
  while(LORA.available()){
    char c = LORA.read();
    Serial.print(c);
    if(bRxStart == false) {
      if(c == 'S') bRxStart = true;
     } else {
        if (c == 'E') {
          dataParsing(rxstring);
          bRxStart = false;
          rxstring = "";
          c = "";
          Serial.println();
          
         } else {
          rxstring += c;
         }
    }
    
  }
  



  
  
  
}
