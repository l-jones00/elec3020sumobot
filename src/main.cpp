#include <Arduino.h>

// ---------- Pin map ----------
#define STBY 13
#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 10

void setup() {
  Serial.begin(115200);
  Serial.println("Motor Test Start");

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH); // enable driver
}

// helper functions
void motorA_forward()  { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
void motorA_reverse()  { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
void motorA_stop()     { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW); }

void motorB_forward()  { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
void motorB_reverse()  { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
void motorB_stop()     { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW); }

void loop() {
  Serial.println("Both FORWARD");
  motorA_forward(); motorB_forward();
  delay(2000);

  Serial.println("Both REVERSE");
  motorA_reverse(); motorB_reverse();
  delay(2000);

  Serial.println("Left only FORWARD");
  motorA_forward(); motorB_stop();
  delay(1500);

  Serial.println("Right only FORWARD");
  motorA_stop(); motorB_forward();
  delay(1500);

  Serial.println("Left only REVERSE");
  motorA_reverse(); motorB_stop();
  delay(1500);

  Serial.println("Right only REVERSE");
  motorA_stop(); motorB_reverse();
  delay(1500);

  Serial.println("STOP");
  motorA_stop(); motorB_stop();
  delay(1500);
}
