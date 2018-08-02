#include <ParachuteDeployer.h>


int ParachuteDeployer::begin(int *chuteServoPins, int *chuteStorePositions, int *chuteFirePositinos, int numChuteServos){
  numServos = numChuteServos;
  servos = new Servo[numChuteServos];
  minPositions = new int[numChuteServos];
  maxPositions = new int[numChuteServos];
  for(int i = 0; i < numChuteServos; i++){
    servos[i].attach(chuteServoPins[i]);
    minPositions[i] = chuteStorePositions[i];
    maxPositions[i] = chuteFirePositinos[i];
    servos[i].write(minPositions[i]);
  }
  curState = LOCKED;
  servoPositions = MIN;
  return 0;
}

int ParachuteDeployer::deploy(){
  curState = FIRING;
  timeAtDeploymentStart = millis();
  update();
  return 0;
}

int ParachuteDeployer::update(){
  switch (curState) {
    case LOCKED:
      break;
    case FIRING:
      if(millis() - timeAtLastStateChange > PARACHUTE_DEPLOYER_MILLIS_BETWEEN_SERVO_CHANGES){
        //switch
        int *newPositions;
        if(servoPositions == MIN){
          newPositions = maxPositions;
          servoPositions = MAX;
        }
        else if(servoPositions == MAX){
          newPositions = minPositions;
          servoPositions = MAX;
        }
        for(int i = 0; i < numServos; i ++){
          servos[i].write(newPositions[i]);
        }
      }
      timeAtLastStateChange = millis();
      if(millis() - timeAtDeploymentStart > PARACHUTE_DEPLOYER_DEPLOYMENT_MILLIS){
        curState = DEPLOYED;
      }
      break;
    case DEPLOYED:
      if(servoPositions == MIN){
        for(int i = 0; i < numServos; i ++){
          servos[i].write(maxPositions[i]);
        }
      }
  }
  return 0;
}
