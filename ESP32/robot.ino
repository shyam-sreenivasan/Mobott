#include "Arduino.h"
#include "motor.h"
#include "motion.h"
#include "comm.h"

static float v = 0;   // forward/backward speed, -1.0 to 1.0
static float w = 0;   // turning speed,          -1.0 to 1.0
static unsigned long lastMsgTime = 0;

// Safety stop: if no command arrives for this long, stop the motors.
static const unsigned long TIMEOUT_MS = 300;

// How long each loop pauses. Lower = checks for commands more often.
static const unsigned long LOOP_DELAY_MS = 5;

void setup() {
  Serial.begin(115200);
  motorInit();
  motionInit();
  commInit();
}

void loop() {
  if (commRead(v, w)) {
    lastMsgTime = millis();
  }

  if (millis() - lastMsgTime > TIMEOUT_MS) {
    v = 0;
    w = 0;
  }

  motionUpdate(v, w);

  delay(LOOP_DELAY_MS);
}
