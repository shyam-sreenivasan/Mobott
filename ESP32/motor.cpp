#include "Arduino.h"
#include "motor.h"

void motorInit() {
  pinMode(18, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(5,  OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(19, OUTPUT);
}

void motorMove(byte dir, int speed) {
  digitalWrite(16, LOW);
  analogWrite(19, speed);

  digitalWrite(17, LOW);
  shiftOut(5, 18, MSBFIRST, dir);
  digitalWrite(17, HIGH);
}
