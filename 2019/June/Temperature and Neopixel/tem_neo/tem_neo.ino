#define TXDEBUG
#include <Adafruit_NeoPixel.h>
#include "DHT.h"
 
#define PIN 6
#define DHTPIN 2
#define DHTTYPE DHT22
#define NUMPIXELS 16

#define MODE 1

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

DHT dht(DHTPIN, DHTTYPE);

typedef enum {
  Low,
  Nomal,
  High,
  Very_High
} state;

boolean check = false;

float IhateThisWeather;

state State;

void showLED(int red, int green, int blue) {
  for(int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels.show();
    delay(50);
  }
  return;
}

void setup() {
  Serial.begin(9600);
  pixels.begin();
  pixels.setBrightness(100);
  pixels.show();
}

void loop() {
  
  if(check == false) {
    delay(3000);
    check = true;
  }
  float temp = dht.readTemperature();
  float humi = dht.readHumidity();

  if(isnan(temp) || isnan(humi)) {
    #if defined(TXDEBUG)
    Serial.println("I Can't Find DHT22!!");
    return;
    #endif
  }else {
    #if defined(TXDEBUG)
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.print(" *C ");
    #endif 
  
  IhateThisWeather = 1.8 * temp -0.55*(1 - humi/100)*(1.8 * temp - 26) + 32;
  #if defined(TXDEBUG)
  Serial.print("  ");
  Serial.print("Uncomfortable level: ");
  Serial.print(IhateThisWeather);


  if(MODE == 1) {
    justShowClearColor();
  } else if(MODE == 2) {
    justShowDimColor();
  }
 
}
}

void justShowClearColor() {
  #endif
  if(IhateThisWeather >= 80) State = Very_High;
  else if(IhateThisWeather < 80 && IhateThisWeather >= 75) State = High;
  else if(IhateThisWeather < 75 && IhateThisWeather >= 68) State = Nomal;
  else if(IhateThisWeather < 68) State = Low;
  
  #if defined(TXDEBUG)
  Serial.print("  ");
  Serial.print("State: ");
  if(State == Low) Serial.println("Low");
  else if(State == Nomal) Serial.println("Nomal");
  else if(State == High) Serial.println("High");
  else if(State == Very_High) Serial.println("Very_High");
  #endif

  if(State == Low) showLED(0, 255, 0);
  else if(State == Nomal) showLED(0, 0, 255);
  else if(State == High) showLED(125, 0, 125);
  else if(State == Very_High) showLED(255, 0, 0);
}

void justShowDimColor() {
  showLED(clearByte(IhateThisWeather * 2),0 ,255 - clearByte(IhateThisWeather*2));
}

int clearByte(float Unfon) {
  if(Unfon > 255) {
    return 255;
  } else {
    return Unfon;
  }
}
