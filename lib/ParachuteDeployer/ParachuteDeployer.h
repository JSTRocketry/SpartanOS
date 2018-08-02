#ifndef _PARACHUTE_DEPLOYER_H_
#define _PARACHUTE_DEPLOYER_H_
#include <Arduino.h>
#include <Servo.h>

#define PARACHUTE_DEPLOYER_MILLIS_BETWEEN_SERVO_CHANGES 500
#define PARACHUTE_DEPLOYER_DEPLOYMENT_MILLIS 50000
class ParachuteDeployer{
public:
  int begin(int *chuteServoPins, int *chuteStorePositions, int *chuteFirePositinos, int numChuteServos);
  int lock();
  int deploy();
  int update();
private:
  int numServos;
  Servo *servos;
  int *minPositions;
  int *maxPositions;
  enum ChuteState {LOCKED,FIRING,DEPLOYED};
  ChuteState curState;
  enum ServoPosition {MIN,MAX};
  ServoPosition servoPositions;
  long timeAtLastStateChange = 0;
  long timeAtDeploymentStart;
};



#endif
