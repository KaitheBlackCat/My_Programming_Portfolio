#include <RCSwitch.h>
#include <Ultrasonic.h>

Ultrasonic ultrasonic(11,10);
RCSwitch mySwitch = RCSwitch();

int i;

void setup() {
  mySwitch.enableTransmit(4);  
}

void loop() {  
  i = ultrasonic.Ranging(CM);
  mySwitch.send(i, 24);
  delay(100); 
}
