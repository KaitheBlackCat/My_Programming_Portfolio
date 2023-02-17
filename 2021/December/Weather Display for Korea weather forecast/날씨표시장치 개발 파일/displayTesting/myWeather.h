//사전 설정은 여기서 이루어집니다.

const char* ssid = "KT_WLAN_7A8F";
const char* password =  "000000c8f3";
 
const String Wendpoint = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/"; //엔드 포인트
const String Mendpoint = "http://apis.data.go.kr/B090041/openapi/service/";//달 관련 앤드 포인트
const String Dendpoint = "http://apis.data.go.kr/B552584/ArpltnInforInqireSvc"; //미세먼지 엔드 포인트

const String key = "oHI3hBRHVfohKwG1H1iPsOShPt37QRjaL5C9u6%2BwUcCrYeSIBesLvm8ZNoav9tCfCdURdOdyP%2BWWtT4z7gSNbQ%3D%3D";

const String infoType [] = {"getUltraSrtNcst", "getUltraSrtFcst", "getVilageFcst", "RiseSetInfoService", "LunPhInfoService"}; //초단기현황, 초단기예보, 단기예보, 일출정보, 월령정보, 

const String base_date = "&base_date=";

const String base_time = "&base_time=";

/************************************************
 * 여기는 지역마다 값이 다릅니다.
 * 대한민국 내 유명 지역의 값을 넣어놓을 것이지만
 * 제가 블로그에 올려놓은 엑셀파일을 참고하여
 * 값을 수정하세요. 구와 동 단위로 자세합니다.
 * 
 * 서울 nx = 60, ny = 127
 * 부산 nx = 98, ny = 76
 * 대구 nx = 89, ny = 90
 * 인천 nx = 55, ny = 124
 * 광주 nx = 58, ny = 74
 * 대전 nx = 67, ny = 100
 * 울산 nx = 102, ny = 84
 * 세종 nx = 66, ny = 103
 * 제주 nx = 52, ny = 38
 * 울릉군 nx = 127, ny = 127
 * 독도 nx = 144, ny = 123
 * 
 ************************************************/
const String nx = "&nx=60";
const String ny = "&ny=127";
const String locationN = "울산";

const String LabelName[] = {"Rain%", "Rain Form", "mm/h", "humidity", "mm/h (sn)", "Sky", "C", "Lowest C", "Highes C", "Wind Speed(WE)", "Wind Speed(NS)", "Wave", "Wind Direction", "Wind Speed", "C", "mm/h", "Thunder"};

typedef struct CURRENT_WEATHER {
  String baseDate;
  String baseTime;
  String category;
  String obserValue;
} current;


typedef struct ULTRA_FORECAST_WEATHER { //초단기 예보. 초단기예보는 향후 6시간까지의 시간별 예보
  String category;
  double fcstValue;
  String fcstDate;
  String fcstTime;
} Uforecast;


typedef struct MOON_INFORMATION { //월령정보
  double lunAge;
  String weekDay;
} moon;

typedef struct RISE_SET_INFOR { //일출일몰 + 시민박명 정보
  String civil_twilight_rise;
  String civil_twilight_set;
  String sunrise;
  String sunset;
  String moonrise;
  String moonset;
} RS;
