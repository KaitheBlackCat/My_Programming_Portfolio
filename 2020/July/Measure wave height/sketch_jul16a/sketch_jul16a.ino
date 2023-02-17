#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h" //  ^
#include <MS5611.h> // 

MPU6050 mpu;
MS5611 baro;

long pressure;
double altitude, min_height, max_height, wave_height;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  while (!baro.begin(MS5611_ULTRA_HIGH_RES)) {
    Serial.println("MS5611 connection unsuccessful, check wiring!");
    delay(500);
  }
}

void loop() {
  unsigned long start_time = millis();
  pressure = baro.readPressure(true);
  altitude = baro.getAltitude(pressure);
  max_height = altitude;
  min_height = altitude;

  //  for 15 seconds
  while(millis() - start_time < 15000){
    pressure = baro.readPressure(true);
    altitude = baro.getAltitude(pressure);
    if (altitude < min_height) min_height = altitude;
    if (altitude > max_height) max_height = altitude;
  }
  wave_height = (max_height - min_height)/2.0;
  Serial.print("Max Height m: ");
  Serial.print(max_height);
  Serial.print(" Min Height m: ");
  Serial.print(min_height);
  Serial.print(" Wave Height m: "); 
  Serial.println(wave_height);
} 
