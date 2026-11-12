#pragma once

#include "Arduino.h"  // for String

#define COMM_WIFI  // comment out to use Serial

void commInit();
bool commRead(float &v, float &w);

// Send a line of telemetry back to the controlling PC. No-op when nothing
// is connected, so callers don't have to check.
void commWrite(const String &line);
