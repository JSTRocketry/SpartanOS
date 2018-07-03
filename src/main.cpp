#include <Arduino.h>

/* Constants */
#define MAIN_CHUTE_OVERRIDE_TIMER 100
#define MINIMUM_ALTITUDE 20
#define MAIN_CHUTE_MIN_DEPLOY_ALTITUDE 150


/* Class Declerations */
#include <IgnititonSystem.h>
#include <RadioTelemetry.h>
#include <RecoverySystem.h>
#include <SdStorage.h>
#include <SensorPackage.h>
/* Function Prototypes */

/* Flight Time Variables */
float attitude[3] = {};
float acceleration[3] = {};
float angularVelocities[3] = {};
float magData[3] = {};
float pressure;
float pressureAltitude;
//gps data member


/* System Variables */
long systemTime;
long flightTime;
long systemTimeAtLaunch;
bool hasLaunchBeenTriggeredByMaster = false;
float startPressure;
bool hasApogeeBeenDetected = false;
bool hasDrogueChuteBeenDeployed = false;
bool hasMainChuteBeenDeployed = false;

enum FlightMode{
  WAITING_FOR_SYNC, WAITING_FOR_ARM, WAITING_FOR_LAUNCH, STAND_DOWN_CALLED, ABORT_CALLED
};

enum EmergencyModes{

}

bool harnessModeActive = false;



void setup() {
  Serial.begin(115200);
  int
    // put your setup code here, to run once:
}

void updateSystems(){

}

int handleRecovery(){
  if(hasLaunchBeenTriggeredByMaster){
    if(!hasApogeeBeenDetected){
      //check out our new altitude, make sure legit
      hasApogeeBeenDetected = true;
      //deploy drogue
      hasDrogueChuteBeenDeployed = true;
      return DROGUE_CHUTE_DEPLOYMENT;
    }
    else if(hasApogeeBeenDetected){
      if(!hasMainChuteBeenDeployed){
        //check if at altitude to deploy chute
        if(pressureAltitude <= MAIN_CHUTE_MIN_DEPLOY_ALTITUDE){
          //deploy main chute
          hasMainChuteBeenDeployed = true;
          return MAIN_CHUTE_DEPLOYMENT;
        }
      }
    }
  }
  return 0;
}

void loop() {
    // put your main code here, to run repeatedly:
    updateSystems();
    //check for recovery events
    int recoveryEvent = handleRecovery();
    //check for new radio commands and handle
    //check for emergency modes and handle'


}
