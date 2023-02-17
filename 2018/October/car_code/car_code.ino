#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

int pos = 0;
int A_1A = 6;
int A_1B = 11;
int B_1A = 3;
int B_1B = 5;
int C_1A = 9;
int C_1B = 10;
int speed = 180;
int speed_b = 255;

void setup() {
    Serial.begin(9600);
    pinMode(A_1A, OUTPUT);
    pinMode(A_1B, OUTPUT);
    pinMode(B_1A, OUTPUT);
    pinMode(B_1B, OUTPUT);
    pinMode(C_1A, OUTPUT);
    pinMode(C_1B, OUTPUT);
    digitalWrite(A_1A, LOW);
    digitalWrite(A_1B, LOW);
    digitalWrite(B_1A, LOW);
    digitalWrite(B_1B, LOW);
    digitalWrite(C_1A, LOW);
    digitalWrite(C_1B, LOW);
    mySwitch.enableReceive(0);
}
void loop() {
    if (mySwitch.available()) {
        int value = mySwitch.getReceivedValue();
        if (value > 0) {
            if (mySwitch.getReceivedValue()==2){  //앞으로
              Serial.println("UP");
               analogWrite(A_1A, 0);
               analogWrite(A_1B, speed);
               analogWrite(B_1A, speed);
               analogWrite(B_1B, 0);
               analogWrite(C_1A, speed_b);
               analogWrite(C_1B, 0);
              }
            else if (mySwitch.getReceivedValue()==1){ //뒤로
              Serial.println("DOWN");
              analogWrite(A_1A, speed);
              analogWrite(A_1B, 0);
              analogWrite(B_1A, 0);
              analogWrite(B_1B, speed);
              analogWrite(C_1A, 0);
              analogWrite(C_1B, speed_b);
              }
            else if (mySwitch.getReceivedValue()==3){ //우현으로
              Serial.println("RIGHT");
              analogWrite(A_1A, speed);
              analogWrite(A_1B, 0);
              analogWrite(B_1A, speed);
              analogWrite(B_1B, 0);
              analogWrite(C_1A, 0);
              analogWrite(C_1B, speed_b);
              }
            else if (mySwitch.getReceivedValue()==4){ //좌현으로
              Serial.println("LEFT");
              analogWrite(A_1A, 0);
              analogWrite(A_1B, speed);
              analogWrite(B_1A, 0);
              analogWrite(B_1B, speed);
              analogWrite(C_1A, 0);
              analogWrite(C_1B, speed_b);
              }
            else if (mySwitch.getReceivedValue()==5){ //멈춤
              Serial.println("DEADZONE");
              analogWrite(A_1A, 0);
              analogWrite(A_1B, 0);
              analogWrite(B_1A, 0);
              analogWrite(B_1B, 0);
              analogWrite(C_1A, 0);
              analogWrite(C_1B, 0);
            }
        }
        mySwitch.resetAvailable();
    }
}
