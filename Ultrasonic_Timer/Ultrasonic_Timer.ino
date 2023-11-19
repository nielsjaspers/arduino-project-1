#include <Arduino.h>

const int trigPin = 9;
const int echoPin = 10;
const int buzzPin = 4;

const int yellowLedPin = 12;
const int redLedPin = 7;

const float SPEED_OF_SOUND = 0.0343;

unsigned long timepassed;
unsigned long triggertime = 0;

float duration, distance;


void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*SPEED_OF_SOUND)/2;

  Serial.print("distance: ");
  Serial.println(distance);

  if (distance <= 50) {
    digitalWrite(yellowLedPin, HIGH);
    if (triggertime == 0){
      triggertime = millis();
    }
    if (millis() - triggertime > 4000){
        digitalWrite(redLedPin, HIGH);
        tone(buzzPin, 3500, 500);
    }
    else{
      digitalWrite(redLedPin, LOW);
    }
  }
  else{
    triggertime = 0;
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
}
