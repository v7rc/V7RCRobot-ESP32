#pragma once

#include <Arduino.h>
#include "FlightTypes.h"

class DebugPrinter {
public:
  void begin(unsigned long baud = 115200) {
    Serial.begin(baud);
  }

  void printHeader() {
    Serial.println(F("t,thr,auth,stb,air,rollDeg,pitchDeg,dRollRate,dPitchRate,dYawRate,mRollRate,mPitchRate,mYawRate,rollCmd,pitchCmd,yawCmd,sat"));
  }

  void printCsv(unsigned long nowMs, const FlightDebugData& d) {
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(d.throttle, 4);
    Serial.print(',');
    Serial.print(d.attitudeAuthority, 4);
    Serial.print(',');
    Serial.print(d.stabilize ? 1 : 0);
    Serial.print(',');
    Serial.print(d.airborne ? 1 : 0);
    Serial.print(',');
    Serial.print(d.correctedRollDeg, 3);
    Serial.print(',');
    Serial.print(d.correctedPitchDeg, 3);
    Serial.print(',');
    Serial.print(d.desiredRollRateDegPerSec, 3);
    Serial.print(',');
    Serial.print(d.desiredPitchRateDegPerSec, 3);
    Serial.print(',');
    Serial.print(d.desiredYawRateDegPerSec, 3);
    Serial.print(',');
    Serial.print(d.measuredRollRateDegPerSec, 3);
    Serial.print(',');
    Serial.print(d.measuredPitchRateDegPerSec, 3);
    Serial.print(',');
    Serial.print(d.measuredYawRateDegPerSec, 3);
    Serial.print(',');
    Serial.print(d.rollCmd, 4);
    Serial.print(',');
    Serial.print(d.pitchCmd, 4);
    Serial.print(',');
    Serial.print(d.yawCmd, 4);
    Serial.print(',');
    Serial.println(d.motorsSaturated ? 1 : 0);
  }
};
