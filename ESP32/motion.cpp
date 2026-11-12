#include "Arduino.h"
#include "motion.h"
#include "motor.h"

// Motor direction bitmasks
static const int M1_F = 2,   M1_B = 4;
static const int M2_F = 128, M2_B = 64;
static const int M3_F = 1,   M3_B = 8;
static const int M4_F = 32,  M4_B = 16;

static float v_smooth = 0;
static float w_smooth = 0;
static const float ALPHA = 1.0f;
static const float DEADBAND = 0.05f;

void motionInit() {
  motorMove(0, 0);
}

void motionUpdate(float v, float w) {
  v_smooth += (v - v_smooth) * ALPHA;
  w_smooth += (w - w_smooth) * ALPHA;

  float left  = v_smooth - w_smooth;
  float right = v_smooth + w_smooth;

  int leftCmd  = 0;
  int rightCmd = 0;

  if      (left  >  DEADBAND) leftCmd  = M1_F | M3_F;
  else if (left  < -DEADBAND) leftCmd  = M1_B | M3_B;

  if      (right >  DEADBAND) rightCmd = M2_F | M4_F;
  else if (right < -DEADBAND) rightCmd = M2_B | M4_B;

  int speed = constrain((int)(max(abs(left), abs(right)) * 255), 0, 255);
  motorMove(leftCmd | rightCmd, speed);
}
