#include <BlynkSimpleSerialBLE.h>
#define BLYNK_PRINT Serial
#include <SoftwareSerial.h>
SoftwareSerial SerialBLE(0, 1); // RX, TX

int motor1_A=10;
int motor1_B=12;
int motor1_Speed=9;
int motor2=6;
int motor3=5;
int motor4=11;
char auth[]= "0f03a04d88284cb88f9916f8ecce6c64";
int b = 0;

const int pin1 = 7;
const int pin2 = 4;
const int pin3 = 3;
const int pin4 = 2;
long messung1 = 0;
int ergebnis1 = 0;
long messung2 = 0;
int ergebnis2 = 0;
long messung3 = 0;
int ergebnis3 = 0;
long messung4 = 0;
int ergebnis4 = 0;

void setup() {
  Serial.begin(9600);
  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE,auth);
  pinMode(motor1_A,OUTPUT);
  pinMode(motor1_B,OUTPUT);
  
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);
  pinMode(motor4,OUTPUT);

}

void AUTO() {
 int a = 1;
  
  while(a==1){
    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, LOW);
    delayMicroseconds(2);
    digitalWrite(pin1, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin1,LOW);
    pinMode(pin1,INPUT);
    messung1 = pulseIn(pin1,HIGH);
    ergebnis1 = (messung1/2);
    a = 2;
  }
  
  while(a==2){
    pinMode(pin2, OUTPUT);
    digitalWrite(pin2, LOW);
    delayMicroseconds(2);
    digitalWrite(pin2, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin2,LOW);
    pinMode(pin2,INPUT);
    messung2 = pulseIn(pin2,HIGH);
    ergebnis2 = (messung2/2)/29;
    a = 3;
  } 

  while(a==3){
    pinMode(pin3, OUTPUT);
    digitalWrite(pin3, LOW);
    delayMicroseconds(2);
    digitalWrite(pin3, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin3,LOW);
    pinMode(pin3,INPUT);
    messung3 = pulseIn(pin3,HIGH);
    ergebnis3 = (messung3/2)/29;
    a = 4;
  } 

   while(a==4){
    pinMode(pin4, OUTPUT);
    digitalWrite(pin4, LOW);
    delayMicroseconds(2);
    digitalWrite(pin4, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin4,LOW);
    pinMode(pin4,INPUT);
    messung4 = pulseIn(pin4,HIGH);
    ergebnis4 = (messung4/2)/29;
    a = 1;
  } 








  
}
BLYNK_WRITE(V1){
  int pinValue = param.asInt();
  
    if(param.asInt() == 1){
      AUTO();
    }
}
void loop(){
  Blynk.run();
 }
