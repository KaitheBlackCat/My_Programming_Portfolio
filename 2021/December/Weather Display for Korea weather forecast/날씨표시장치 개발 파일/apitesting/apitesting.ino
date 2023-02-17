#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
 
const char* ssid = "KT_WLAN_7A8F";
const char* password =  "000000c8f3";
 
const String endpoint = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtNcst?serviceKey=";
const String key = "oHI3hBRHVfohKwG1H1iPsOShPt37QRjaL5C9u6%2BwUcCrYeSIBesLvm8ZNoav9tCfCdURdOdyP%2BWWtT4z7gSNbQ%3D%3D";
int pageNo = 1;
const String dataType = "&dataType=Json";
const String base_date = "&base_date=20211216";
const String base_time = "&base_time=2200";
const String nx = "&nx=55";
const String ny = "&ny=127";

const String LabelName[] = {"Rain%", "Rain Form", "mm/h", "humidity", "mm/h (sn)", "Sky", "C", "Lowest C", "Highes C", "Wind Speed(WE)", "Wind Speed(NS)", "Wave", "Wind Direction", "Wind Speed", "C", "mm/h", "Thunder"};

typedef struct CURRENT_WEATHER {
  String baseDate;
  String baseTime;
  String category;
  String obserValue;
} current;

HTTPClient http;
 
void setup() {
 
  Serial.begin(115200);

  
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
 
}
 
void loop() {
  
  
 
  if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status


    current datas[8];

    for(int t = 0; t< 8; t++) {

      String objNum = "&pageNo=" + String(pageNo + t) + "&numOfRows=1";
      String serv = endpoint + key + objNum + dataType + base_date + base_time + nx + ny;
      String payload = httpGETRequest(serv.c_str());
      String response = getContent("response", payload);
      String body = getContent("body", response);
      String items = getContent("items", body);
      //Serial.println("items = " + items);

      DynamicJsonDocument k(1024);
      deserializeJson(k, items);
      JsonObject ob = k.as<JsonObject>();
      
      String item = ob["item"][0];
      //Serial.println("item " + String(t+1) + " = " + item);

      datas[t].baseDate = getContent("baseDate", item);

      datas[t].baseTime = getContent("baseTime", item);

      datas[t].category = getContent("category", item);

      datas[t].obserValue = getContent("obsrValue", item);

      if (decodeLabel(datas[t].category) != -1) {
        Serial.println("category " + String(t+1) + " = " + LabelName[decodeLabel(datas[t].category)]);
      } else {
        Serial.println("category " + String(t+1) + " = " + datas[t].category);
      }

      Serial.println("baseDate " + String(t+1) + " = " + datas[t].baseDate);
      Serial.println("baseTime " + String(t+1) + " = " + datas[t].baseTime);
      Serial.println("obserValue " + String(t+1) + " = " + datas[t].obserValue);
    }



      }
 
    else {
      Serial.println("Error on HTTP request");
    }
 
    delay(30000000);
  }

String getContent(const char* Name, String Line) {
  DynamicJsonDocument doc(4096);
  deserializeJson(doc, Line);
  JsonObject obj = doc.as<JsonObject>();

  String payload = obj[Name];
  
  return payload;
}



String httpGETRequest(const char* serverName) {

  // Your IP address with path or Domain name with URL path 
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}"; 

  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
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
