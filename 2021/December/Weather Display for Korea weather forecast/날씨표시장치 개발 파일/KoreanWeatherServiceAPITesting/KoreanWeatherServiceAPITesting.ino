#include <ESP8266WiFi.h> // ESP 8266 와이파이 라이브러리
#include <ESP8266HTTPClient.h> // HTTP 클라이언트


String url = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtFcst?serviceKey=oHI3hBRHVfohKwG1H1iPsOShPt37QRjaL5C9u6%2BwUcCrYeSIBesLvm8ZNoav9tCfCdURdOdyP%2BWWtT4z7gSNbQ%3D%3D&pageNo=1&numOfRows=1000&dataType=XML&base_date=20211211&base_time=2300&nx=55&ny=127";

void setup()
{
  // 시리얼 세팅
  Serial.begin(115200);
  Serial.println();

  // 와이파이 접속
  WiFi.begin("KT_WLAN_7A8F", "000000c8f3"); // 공유기 이름과 비밀번호

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) // 와이파이 접속하는 동안 "." 출력
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
}

void Print(String category) {
  if (category == "POP") Serial.println("강수확률");
  else if (category == "PTY") Serial.println("강수형태");
  else if (category == "PCP") Serial.println("1시간 강수량");
  else if (category == "REH") Serial.println("습도");
  else if (category == "SNO") Serial.println("1시간 신적설");
  else if (category == "SKY") Serial.println("하늘상태");
  else if (category == "TMP") Serial.println("1시간 기온");
  else if (category == "TMN") Serial.println("일 최저기온");
  else if (category == "TMX") Serial.println("일 최고기온");
  else if (category == "UUU") Serial.println("풍속(동서성분)");
  else if (category == "VVV") Serial.println("풍속(남북성분)");
  else if (category == "WAV") Serial.println("파고");
  else if (category == "VEC") Serial.println("풍향");
  else if (category == "WSD") Serial.println("풍속");
  else if (category == "T1H") Serial.println("기온");
  else if (category == "RN1") Serial.println("1시간 강수량");
  else if (category == "LGT") Serial.println("낙뢰");
  else Serial.println("Error");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) // 와이파이가 접속되어 있는 경우
  {
    WiFiClient client; // 와이파이 클라이언트 객체
    HTTPClient http; // HTTP 클라이언트 객체

    if (http.begin(client, url)) {  // HTTP
      // 서버에 연결하고 HTTP 헤더 전송
      int httpCode = http.GET();

      // httpCode 가 음수라면 에러
      if (httpCode > 0) { // 에러가 없는 경우
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = http.getString(); // 받은 XML 데이터를 String에 저장

          int start_point = payload.indexOf("<category>");
          int end_point = payload.indexOf("</category>");
          String category=payload.substring(start_point+10,end_point); //더하기11은 <pm10Value>글자수

          int start_date = payload.indexOf("<fcstDate>");
          int end_date = payload.indexOf("</fcstDate>");
          String Date = payload.substring(start_date + 10, end_date);

          int start_time = payload.indexOf("<fcstTime>");
          int end_time = payload.indexOf("</fcstTime>");
          String Time = payload.substring(start_time + 10, end_time);

          int start_point_value = payload.indexOf("<fcstValue>");
          int end_point_value = payload.indexOf("</fcstValue>");
          String Value = payload.substring(start_point_value + 11, end_point_value);

          while(start_point < payload.length()){
            
            Serial.println("category code : " + category);
            Serial.print("Category : ");
            Print(category);
            Serial.println("Forcast Date = " + Date + ", Forcast Time = " + Time);
            Serial.println("Value = " + Value);
            Serial.println();
            
            start_point = payload.indexOf("<category>", start_point+10);
            end_point = payload.indexOf("</category>", end_point+11);
            category=payload.substring(start_point+10,end_point);

            start_date = payload.indexOf("<fcstDate>", start_date + 10);
            end_date = payload.indexOf("</fcstDate>", end_date + 11);
            Date = payload.substring(start_date + 10, end_date);

            start_time = payload.indexOf("<fcstTime>", start_time + 10);
            end_time = payload.indexOf("</fcstTime>", end_time + 11);
            Time = payload.substring(start_time + 10, end_time);

            start_point_value = payload.indexOf("<fcstValue>", start_point_value + 11);
            end_point_value = payload.indexOf("</fcstValue>", end_point_value + 12);
            Value = payload.substring(start_point_value + 11, end_point_value);
            
          }
        }
      } else {
        Serial.printf("[HTTP] GET... 실패, 에러코드: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    } else {
      Serial.printf("[HTTP] 접속 불가\n");
    }
    Serial.println();
    delay(600000);
  }
}
