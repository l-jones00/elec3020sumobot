#include <Arduino.h>

#define TRIG_FRONT 43
#define ECHO_FRONT 18
#define TRIG_LEFT  44
#define ECHO_LEFT  21
#define TRIG_RIGHT 16
#define ECHO_RIGHT 17

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL);  // timeout 30 ms (~5 m range)
  if (duration == 0) return -1;  // no echo
  long distance = duration / 58; // convert to cm
  return distance;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  
}

void loop() {
  long dFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long dLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  long dRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: ");
  if (dFront < 0) Serial.print("No echo"); else { Serial.print(dFront); Serial.print(" cm"); }

  Serial.print("   Left: ");
  if (dLeft < 0) Serial.print("No echo"); else { Serial.print(dLeft); Serial.print(" cm"); }

  Serial.print("   Right: ");
  if (dRight < 0) Serial.print("No echo"); else { Serial.print(dRight); Serial.print(" cm"); }

  Serial.println();
  delay(500);
}
