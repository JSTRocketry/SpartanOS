5
#include <Arduino.h>

/* Constants */
#define MAIN_CHUTE_OVERRIDE_TIMER 100
#define MINIMUM_ALTITUDE 20
#define MAIN_CHUTE_MIN_DEPLOY_ALTITUDE 150
#define MAX_ANGULAR_VELOCITY_X_Y 60
#define MAX_ANGULAR_VELOCITY_Z 1000


/* Class Declerations */

#include <RadioTelemetry.h>
#include <RecoverySystem.h>
#include <SdStorage.h>
#include <SensorPackage.h>
#include <IgnititonSystem.h>
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
  MINIMUM_ALTITUDE_NOT_REACHED, UNEXPECTED_LOSS_OF_CONTROL, UNEXPECTED_LOSS_OF_POWER
}

bool harnessModeActive = false;



void setup() {
  Serial.begin(115200);
  int
    // put your setup code here, to run once:
}

String serialBuffer;

int checkForSerialCommand(String *line){
  if(Serial.available()){
    while(Serial.available()){
      char c = (char)Serial.read();
      if(c == '\n' || c =='\r'){
        *line = serialBuffer;
        return 0;
        serialBuffer = "";
      }
      serialBuffer += c;
    }
  }
  return -1;
}

bool harnessModeGyroOverride = false;
bool harnessModeAccelOverride = false;
bool harnessModeMagOverride = false;
bool harnessModeAttitudeOverride = false;
bool harnessModePressureAltitudeOverride = false;

#define NEW_HARNESS_MODE_COMMAND -1
#define NEW_HARNESS_MODE_DATA_OVERRIDE -2
#define NO_NEW_HARNESS_COMMAND -3
String serialCommand;
int checkForHarnessCommand(){

  int newCommandStatus = checkForSerialCommand(&serialCommand);
  if(newCommandStatus == 0){
    //save the line here
  }
  else return NO_NEW_HARNESS_COMMAND;
  if(!harnessModeActive){
    if(serialCommand == ENTER_HARNESS_MODE_COMMAND){
      harnessModeActive = true;
      return NEW_HARNESS_MODE_COMMAND;
    }
  }
  //check if appropriate harness mode command has been sent over Serial3
  //if it has, start checking for sensor override commands
  //if a sensor has an override, accept the data given from computer
}

void updateSystems(){
  int harnessCommandState = checkForHarnessCommand();
  if(!harnessModeActive){
    //update sensors normally
  }
  else if()

  //update sensors (unless if in Harness Mode)
  //if in harnessMode, chec
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
