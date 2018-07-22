#ifndef _IGNITION_SYSTEM_H_
#define _IGNITION_SYSTEM_H_
#include <Arduino.h>

#define ROCKET_LAUNCHED 0
#define IGNITION_SYSTEM_ROCKET_NOT_ARMED -1
#define IGNITION_SYSTEM_ROCKET_STANDDOWN -2

class IgnitionSystem{
public:
  int begin(int firingPin);
  int arm();
  int fire();
  int standDown();
private:
  int firingPin = -1;
  bool rocketIsarmed = false;
  bool rocketStandDown = false;
};

#endif
