void setup() {
  Serial.begin(9600);                               // 시리얼 통신을 시작하며, 통신속도는 9600
}
void loop() {
  int X = analogRead(1);                           // 변수 X에 아날로그 1번핀에 입력되는 신호를 대입
  int Y = analogRead(0);                           // 변수 Y에 아날로그 0번핀에 입력되는 신호를 대입
  Serial.print("   ");                                   // 시리얼 모니터에 출력 - 띄어쓰기 3칸
  Serial.print("X: ");                                  // 시리얼 모니터에 출력 - X:
  Serial.print(X);                                      // 시리얼 모니터에 출력 - (X 좌표 신호)
  Serial.print("   ");                                   // 시리얼 모니터에 출력 - 띄어쓰기 3칸
  Serial.print("Y: ");                                  // 시리얼 모니터에 출력 - Y:
  Serial.println(Y);                                    // 시리얼 모니터에 출력 - (Y 좌표 신호)
  delay(300);                                           // 0.3초 대기
}
