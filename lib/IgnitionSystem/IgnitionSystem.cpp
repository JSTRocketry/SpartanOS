#include "IgnitionSystem.h"

int IgnitionSystem::begin(int firingPin){
  this->firingPin = firingPin;
  pinMode(firingPin, OUTPUT);
  digitalWrite(firingPin, LOW);
  return 0;
}

int IgnitionSystem::arm(){
  rocketIsarmed = true;
  return 0;
}

int IgnitionSystem::fire(){
  if(rocketIsarmed && !rocketStandDown){
    digitalWrite(firingPin, HIGH);
    return ROCKET_LAUNCHED;
  }
  if(!rocketIsarmed){
    return IGNITION_SYSTEM_ROCKET_NOT_ARMED;
  }
  if(rocketStandDown)
  {
    return IGNITION_SYSTEM_ROCKET_STANDDOWN;
  }

  }

int IgnitionSystem::standDown(){
  rocketStandDown = true;
  digitalWrite(firingPin, LOW);
  return 0;
}
