int pin = 8;
long duration;
long starttime;
long sampletime_ms = 30000;
long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
float ugm3 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
  starttime = millis();
  Serial.println("Start");
}
void loop() {
  duration = pulseIn(pin, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if((millis() - starttime) > sampletime_ms)
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
    ugm3 = concentration * 100 / 13000;
    Serial.print(ugm3);
    Serial.println("ug/m3");
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}
