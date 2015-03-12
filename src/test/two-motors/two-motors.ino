#include "Pulse.h"
#include "Servo.h"

#define INPUT_HIGH 1023
#define MAX_VOLTAGE 5
#define PERIOD 10000

Servo bucketsServo;

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT); // Enable or not
  pinMode(11, OUTPUT); // Direction
  pinMode(A0, INPUT); // Input read
  pinMode(A1, INPUT);
   
  pinMode(3, OUTPUT);
  pinMode(5,OUTPUT);//E2
  pinMode(6,OUTPUT);//D2
  pinMode(2, OUTPUT);
  
  InitPulse(3, 10);
  bucketsServo.attach(11);
}
void loop() {
  static int analogInPin = A0;
  static int transistorAnalogInPin = A1;
  static int outputPin1 = 10;
  static int dirPin1 = 11;
  static int ledOutputPin = 3;
  
  static int outputPin2 = 5;
  static int dirPin2 = 6; 
  static int ledPin = 2;
  
  digitalWrite(ledPin, HIGH);
  
  static char state = 'r';
  int sensorValue = 1000; // analogRead(analogInPin);
  int lengthWrite = map(sensorValue, 0, INPUT_HIGH, 0, 255);
  
  // Module LED at 490 Hz
  if (IsPulseFinished()) {
    Pulse(200);
  }
  
  int transistorSensorValue = analogRead(transistorAnalogInPin);
  Serial.println(transistorSensorValue);
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'f') digitalWrite(dirPin1, HIGH);
    else if (ch == 'b') digitalWrite(dirPin1, LOW); //first motor reverse
    
    else if (ch == 'q') digitalWrite(dirPin2,HIGH);
    else if (ch == 'w') digitalWrite(dirPin2,LOW);//second motor reverse
    
    else if (ch == 'r') state = 'r';
    else if (ch == 's') state = 's';
  }
  switch (state) {
    case 'r':
      analogWrite(outputPin1, lengthWrite);
      analogWrite(outputPin2, lengthWrite);
      break;
    
    case 's':
      analogWrite(outputPin1, 0);
      analogWrite(outputPin2,0);
      break;
  }
}
