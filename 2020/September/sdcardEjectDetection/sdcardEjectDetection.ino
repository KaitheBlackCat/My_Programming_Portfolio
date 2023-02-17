// include the SD library:
#include <SPI.h>
#include <SD.h>

Sd2Card card;

const int chipSelect = 4;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
}
void loop(void) {
  delay(50);

  // 카드 작동 여부 판단
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    digitalWrite(7, HIGH);
    digitalWrite(6, LOW);
  } else {
    digitalWrite(7, LOW);
    digitalWrite(6, HIGH);
  }
}
