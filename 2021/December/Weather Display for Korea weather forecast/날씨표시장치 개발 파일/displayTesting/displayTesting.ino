#include <SPI.h>              //디스플레이와의 통신을 위한 라이브러리
#include <ESP8266WiFi.h>      //와이파이 연결을 위한 라이브러리
#include <ESP8266HTTPClient.h> //HTTP 클라이언트
#include <ArduinoJson.h>      //서버에서 받아온 Json 정보를 처리하는 라이브러리
#include <NTPClient.h>       //NTP 서버에서 시간을 받아오도록 하는 라이브러리
#include <WiFiUdp.h>         //NTPClient 라이브러리의 동작을 돕는 라이브러리
#include <TimeLib.h>          //epoch 타임으로 날짜 계산하려구요
#include <math.h>             //수학계산 라이브러리
#include "ssd1325_spi.h"      //직접 수정한 ssd1325 디스플레이용 라이브러리
#include "fonts.h"            //OLED에 표기할 글자의 폰트를 지정한 라이브러리
#include "MyFonts.h"
#include "myWeather.h"
#include "moonPhaseicon.h"

// 소프트웨어로 SPI핀을 지정하려면 CLK와 MOSI에 해당하는 핀 번호를 적어주시고
// 하드웨어 SPI로 연결하시려면 둘 다 -1로 지정해주세요.
// 하드웨어 SPI로 연결할 경우 화면 전환 속도가 더 부드럽고 빨라집니다.
// 다만 연결할 수 있는 핀에 제약이 생기죠.
#define OLED_CLK D3
#define OLED_MOSI D4

// 이것들은 하드웨어 연결 시에도 지정이 필요합니다.
#define OLED_CS D1
#define OLED_RESET D0
#define OLED_DC D2

// 변수 지정 구간 ****************************

const String WDAY_NAMES[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
current cur[8];
Uforecast Ufore[24];
moon luna;
RS riseSet;
bool firstTime = true;

//********************************************


// 객체 생성 구간 *************************************************

SSD1325 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);  // 디스플레이 객체 생성
WiFiUDP ntpUDP;                                                      // UDP 객체 생성
NTPClient timeClient(ntpUDP, "kr.pool.ntp.org", 32400, 3600000);    // UDP 객체를 통한 timeClient 객체 생성, 
                                                                     // 한국 표준시 GMT+9에 맞춰 9시간(32400초) 보정, 1시간(3600000ms) 간격으로 시간 갱신

// ****************************************************************


int numberOfFrames = 2;  // 프레임 수 입니다. OLED에 화면을 총 몇 개를 띄울지 결정합니다.
bool DoINeedUpdate = false; // 날씨 정보를 업데이트 할 것인지를 결정하도록 합니다. 

String giveFormedTime(String Time) {
  return Time.toInt() < 10 ? "0" + Time : Time;
}

//프레임 함수 지정 구간********
void drawFrame1(int x, int y){
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(Open_Sans_Hebrew_Bold_24);
  display.drawString(64 + x, 5 + y, timeClient.getFormattedTime());
  display.setFont(ArialMT_Plain_10);
  String M = String(month(timeClient.getEpochTime()));
  M = M.toInt() < 10 ? "0"+M : M;
  String D = String(day(timeClient.getEpochTime()));
  D = D.toInt() < 10 ? "0"+D : D;
  
  String Date = String(year(timeClient.getEpochTime())) + "." + M + "." + D + " " + WDAY_NAMES[timeClient.getDay()];
  display.drawString(64 + x, 35 + y, Date);
}

void drawFrame3(int x, int y){ // 1시간, 2시간, 3시간 후의 날씨 예보(초단기예보 활용)

  char weather[] = {')', ')', ')'};
  double temper[3];
  String T[] = {giveFormedTime(String(timeClient.getHours() + 1)).toInt() >= 24 ? giveFormedTime(String(giveFormedTime(String(timeClient.getHours() + 1)).toInt() - 24)) : giveFormedTime(String(timeClient.getHours() + 1)), 
                giveFormedTime(String(timeClient.getHours() + 2)).toInt() >= 24 ? giveFormedTime(String(giveFormedTime(String(timeClient.getHours() + 2)).toInt() - 24)) : giveFormedTime(String(timeClient.getHours() + 2)),
                giveFormedTime(String(timeClient.getHours() + 3)).toInt() >= 24 ? giveFormedTime(String(giveFormedTime(String(timeClient.getHours() + 3)).toInt() - 24)) : giveFormedTime(String(timeClient.getHours() + 3)) 
               };
  for(int k = 0; k < 60; k++) {
    for(int j = 0; j < 3; j++) {
      if(Ufore[k].fcstTime == T[j]+"00") {
        if(Ufore[k].category == "SKY") {
          Serial.println("Ufore[k].category == \"SKY\"");
          if(Ufore[k].fcstValue == "1") weather[j] = 'B';
          else if(Ufore[k].fcstValue == "3") weather[j] = 'H';
          else if(Ufore[k].fcstValue == "4") weather[j] = 'N';
          else weather[j] = ')';
        }
        if(Ufore[k].category == "T1H") {
          Serial.println(Ufore[k].fcstValue);
          temper[j] = Ufore[k].fcstValue.toDouble();
          Serial.println(temper[j]);
        }  
      }
    }
  }

  
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  
  display.setFont(ArialMT_Plain_10);
  display.drawString(20 + x, y - 2, T[0] + ":00");
  display.drawString(64 + x, y - 2, T[1] + ":00");
  display.drawString(108 + x, y - 2, T[2] + ":00");
  
  display.setFont(Meteocons_0_30);
  display.drawString(20 + x, 8 + y, String(weather[0]));
  display.drawString(64 + x, 8 + y, String(weather[1]));
  display.drawString(108 + x, 8 + y, String(weather[2]));

  display.setFont(ArialMT_Plain_10);
  display.drawString(20 + x, 40 + y, String(temper[0]));
  display.drawString(64 + x, 40 + y, String(temper[1]));
  display.drawString(108 + x, 40 + y, String(temper[2]));
  
}

void drawOverlay() {
  timeClient.update();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  String H = String(timeClient.getHours());
  H = H.toInt() < 10 ? "0"+H : H;
  String M = String(timeClient.getMinutes());
  M = M.toInt() < 10 ? "0"+M : M;
  display.drawString(0, 53, H + ":" + M);
}

//*****************************

void (*frameCallbacks[])(int x, int y) = {drawFrame1, drawFrame3}; // 프레임을 추가하셨다면 여기에 똑같이 추가해야 합니다.

HTTPClient http;

void setup() {

  Serial.begin(9600);

  WiFi.begin(ssid, password);

  while( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }

  timeClient.begin();
  display.init();
  
  display.clear();
  display.display();

  display.setFrameCallbacks(numberOfFrames, frameCallbacks);

  display.setFrameTransitionTicks(10);

  display.setFrameWaitTicks(100);

  timeClient.update();

  weatherUpdate();
  
}

void loop() {

  if((WiFi.status() == WL_CONNECTED)) {
    if((timeClient.getMinutes() % 10 == 0 || timeClient.getMinutes() == 0) && timeClient.getSeconds() == 0 ) weatherUpdate();  //매 시간 10분 째가 될 때마다(10분, 20분, 30분, 40분, 50분) 정보 업데이트. 이걸 사용하려면 아래 구문을 주석처리
    //if((timeClient.getMinutes() % 5 == 0 || timeClient.getMinutes() == 0) && timeClient.getSeconds() == 0) weatherUpdate();  //매 시간 5분 째가 될 때마다 정보 업데이트. 이걸 사용하려면 위 구분을 주석처리
    display.clear();
    drawOverlay();
    display.nextFrameTick();
    display.display();  
  }

}


//날짜만으로 요일을 계산해주는 함수
//출처 : https://create.arduino.cc/projecthub/erikuchama/day-of-the-week-calculator-cde704
int calculateWD(int yyyy, int m, int d) {
    int yy, c, mTable, SummedDate, DoW, leap, cTable = 0;

    if((fmod(yyyy,4) == 0 && fmod(yyyy,100) != 0) || (fmod(yyyy,400) == 0))
    { leap = 1; }
    else 
    { leap = 0; }

    // Limit results to year 1900-2299 (to save memory)
    while(yyyy > 2299)
    { yyyy = yyyy - 400; }
    while(yyyy < 1900)
    { yyyy = yyyy + 400; }

    // Calculating century
    c = yyyy/100;

    // Calculating two digit year
    yy = fmod(yyyy, 100);

    // Century value based on Table
    if(c == 19) { cTable = 1; }
    if(c == 20) { cTable = 0; }
    if(c == 21) { cTable = 5; }
    if(c == 22) { cTable = 3; }

    // Jan and Feb calculations affected by leap years
    if(m == 1)
    { if(leap == 1) { mTable = 6; }
      else          { mTable = 0; }}
    if(m == 2)
    { if(leap == 1) { mTable = 2; }
      else          { mTable = 3; }}
    // Other months not affected and have set values
    if(m == 10) { mTable = 0; }
    if(m == 8) { mTable = 2; }
    if(m == 3 || m == 11) { mTable = 3; }
    if(m == 4 || m == 7) { mTable = 6; }
    if(m == 5) { mTable = 1; }
    if(m == 6) { mTable = 4; }
    if(m == 9 || m == 12) { mTable = 5; }

    // Enter the data into the formula
    SummedDate = d + mTable + yy + (yy/4) + cTable;
    
    // Find remainder
    DoW = fmod(SummedDate,7);  

    if (DoW == 0) return 6;
    if (DoW == 1) return 0;
    if (DoW == 2) return 1;
    if (DoW == 3) return 2;
    if (DoW == 4) return 3;
    if (DoW == 5) return 4;
    if (DoW == 6) return 5;
  
}

String httpGETRequest(const char* serverName) {

  // Your IP address with path or Domain name with URL path 
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}"; 

  if (httpResponseCode>0) {
    //Serial.print("HTTP Response code: ");
    //Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

int decodeLabel(String category) {
  if (category == "POP") return 0;
  else if (category == "PTY") return 1;
  else if (category == "PCP") return 2;
  else if (category == "REH") return 3;
  else if (category == "SNO") return 4;
  else if (category == "SKY") return 5;
  else if (category == "TMP") return 6;
  else if (category == "TMN") return 7;
  else if (category == "TMX") return 8;
  else if (category == "UUU") return 9;
  else if (category == "VVV") return 10;
  else if (category == "WAV") return 11;
  else if (category == "VEC") return 12;
  else if (category == "WSD") return 13;
  else if (category == "T1H") return 14;
  else if (category == "RN1") return 15;
  else if (category == "LGT") return 16;
  else return -1;
}

double getContentInDouble(const char* Name, String Line) { //Json 문자열에서 얻은 숫자 정보 등을 정수형으로 저장
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, Line);
  JsonObject obj = doc.as<JsonObject>();

  double payload = obj[Name].as<double>();
  
  return payload;
}

String getContentInString(const char* Name, String Line) { //Json 문자열에서 얻은 정보를 문자열로 저장
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, Line);
  JsonObject obj = doc.as<JsonObject>();

  String payload = obj[Name];
  
  return payload;
}

String getContentInComplex(const char* Name, String Line, int num) { // Json 문자열에서 얻은 정보 중 행렬에 포함된 정보를 해당 번호에 해당하는 정보 저장
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, Line);
  JsonObject obj = doc.as<JsonObject>();

  String payload = obj[Name][num];

  return payload;
}

void getWeatherInfo(String Y, String Mon, String Date, String H, String M) { // 일기 현황 정보를 가져옵니다.

  for (int t = 0; t < 8; t++) {
    // 서버네임 예시
    // http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtNcst?serviceKey=키네임&pageNo=1&numOfRows=1&dataType=Json&base_date=20211220&base_time=2300&nx=55&ny=127
    
    String server = Wendpoint + infoType[0] + "?serviceKey=" + key + "&pageNo=1" + "&numOfRows=1000&dataType=Json" + "&base_date=" + Y + Mon + Date + "&base_time=" + H + M + nx + ny;
    String content = httpGETRequest(server.c_str());
    String response = getContentInString("response", content);
    String body = getContentInString("body", response);
    String items = getContentInString("items", body);

    String item = getContentInComplex("item", items, t);

    cur[t].category = getContentInString("category", item);

    cur[t].obserValue = getContentInString("obsrValue", item);
  }

}

//메모리 효율성을 위해 필요한 정보만 가져오는 방식으로 코드를 변경합니다. (20211222)
//현재 표시를 위해 필요한 정보는 하늘 상태, 온도, 예보 시간, 강수 상태, 낙뢰 여부 입니다.
//각각의 코드는 SKY, T1H, fcstTime, PTY, LGT입니다.
void getUForecastInfo(String Y, String Mon, String Date, String H, String M) { //초단기 예보 정보를 저장합니다.

  int settingN = 0;

  for (int t = 0; t < 60; t++) {
    // 서버네임 예시
    // http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtFcst?serviceKey=키네임&pageNo=1&numOfRows=1000&dataType=JSON&base_date=20211220&base_time=2330&nx=55&ny=127
    
    String server = Wendpoint + infoType[1] + "?serviceKey=" + key + "&pageNo=" + String(t+1) + "&numOfRows=1&dataType=Json" + "&base_date=" + Y + Mon + Date + "&base_time=" + H + M + nx + ny;
    String content = httpGETRequest(server.c_str());
    String response = getContentInString("response", content);
    String body = getContentInString("body", response);
    String items = getContentInString("items", body);

    String item = getContentInComplex("item", items, 0);

    if(getContentInString("category", item) == "SKY" || getContentInString("category", item) == "T1H" || getContentInString("category", item) == "PTY" || getContentInString("category", item) == "LGT") {
      Ufore[settingN].category = getContentInString("category", item);

      Ufore[settingN].fcstValue = getContentInDouble("fcstValue", item);

      Ufore[settingN].fcstDate = getContentInString("fcstDate", item);

      Ufore[settingN].fcstTime = getContentInString("fcstTime", item);

      settingN++;
    }
  }
}

void getMoonInfo(String Y, String Mon, String Date) { //월령 정보 받아오기
  
  // 서버네임 예시
  // http://apis.data.go.kr/B090041/openapi/service/LunPhInfoService/getLunPhInfo?serviceKey=키네임&solYear=2021&solMonth=12&solDay=21&_type=json
    
  String server = Mendpoint + infoType[4] + "/getLunPhInfo?serviceKey=" + key + "&solYear=" + Y + "&solMonth=" + Mon + "&solDay=" + Date + "_type=json";
  String content = httpGETRequest(server.c_str());
  String response = getContentInString("response", content);
  String body = getContentInString("body", response);
  String items = getContentInString("items", body);

  String item = getContentInString("item", items);

  luna.lunAge = getContentInDouble("lunAge", item);

  luna.weekDay = getContentInString("solWeek", item);
}

void getRiseSetInfo(String Y, String Mon, String Date) { //일출몰 정보 받아오기
  
  // 서버네임 예시
  // http://apis.data.go.kr/B090041/openapi/service/RiseSetInfoService/getAreaRiseSetInfo?serviceKey=키네임&locdate=20211221&location=지역명
    
  String server = Mendpoint + infoType[3] + "/getAreaRiseSetInfo?serviceKey=" + key + "&locdate=" + Y + Mon + Date + "&location=" + locationN + "_type=json";
  String content = httpGETRequest(server.c_str());
  String response = getContentInString("response", content);
  String body = getContentInString("body", response);
  String items = getContentInString("items", body);

  String item = getContentInString("item", items);

  riseSet.civil_twilight_set = getContentInString("civile", item);
  riseSet.civil_twilight_rise = getContentInString("civilm", item);
  riseSet.moonrise = getContentInString("moonrise", item);
  riseSet.moonset = getContentInString("moonset", item);
  riseSet.sunrise = getContentInString("sunrise", item);
  riseSet.sunset = getContentInString("sunset", item);
}

void weatherUpdate() {

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "GET CURRENT WEATHER");
  display.display();

  Serial.print("Month = "); Serial.println(month(timeClient.getEpochTime()));
  Serial.print("Day = "); Serial.println(day(timeClient.getEpochTime()));

  Serial.print("Month(S) = "); Serial.println(String(month(timeClient.getEpochTime())));
  Serial.print("Day(S) = "); Serial.println(String(day(timeClient.getEpochTime())));

  String MONTH = giveFormedTime(String(month(timeClient.getEpochTime())));
  String DAY = giveFormedTime(String(day(timeClient.getEpochTime())));
  String HOUR = giveFormedTime(String(timeClient.getHours()));
  String MINUTES = giveFormedTime(String(timeClient.getMinutes()));

  Serial.print("Formed Month = "); Serial.println(MONTH);
  Serial.print("Formed Day = "); Serial.println(DAY);
  
  //현재 날씨 정보 받아오기
  getWeatherInfo(String(year(timeClient.getEpochTime())), MONTH, DAY, HOUR, MINUTES);

  display.clear();
  display.drawString(0, 0, "[ == DONE == ]");
  display.drawString(0, 10, "GET ULTRA FORECAST");
  display.display();
  

  //초단기 예보 정보 받아오기
  if(timeClient.getMinutes() <= 30) { //만약 현재 시간이 30분보다 전이라면
    if(timeClient.getHours() == 0) getUForecastInfo(String(year(timeClient.getEpochTime())), MONTH, String(DAY.toInt() - 1), "23", "30"); // 지금 시간이 자정이라면
    else getUForecastInfo(String(year(timeClient.getEpochTime())), MONTH, DAY, String(HOUR.toInt() - 1), "30"); // 자정이 아니라면
  } else {
    getUForecastInfo(String(year(timeClient.getEpochTime())), MONTH, DAY, HOUR, "30");
  } 

  display.clear();
  display.drawString(0, 0, "[ == DONE == ]");
  display.drawString(0, 10, "[ == DONE == ]");
  display.drawString(0, 20, "GET MOON INFO");
  display.display();


  getMoonInfo(String(year(timeClient.getEpochTime())), MONTH, DAY); //월령정보 받기

  display.clear();
  display.drawString(0, 0, "[ == DONE == ]");
  display.drawString(0, 10, "[ == DONE == ]");
  display.drawString(0, 20, "[ == DONE == ]");
  display.drawString(0, 30, "GET RISE SET INFO");
  display.display();

  getRiseSetInfo(String(year(timeClient.getEpochTime())), MONTH, DAY); //일출몰 정보 받기

  display.clear();
  display.drawString(0, 0, "[ == DONE == ]");
  display.drawString(0, 10, "[ == DONE == ]");
  display.drawString(0, 20, "[ == DONE == ]");
  display.drawString(0, 30, "[ == DONE == ]");
  display.display();
    
}
