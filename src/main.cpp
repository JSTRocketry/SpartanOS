#include <Arduino.h>

/* Constants */
#define MAIN_CHUTE_OVERRIDE_TIMER 100
#define MINIMUM_ALTITUDE 20
#define MAIN_CHUTE_MIN_DEPLOY_ALTITUDE 150
#define MAX_ANGULAR_VELOCITY_X_Y 60
#define MAX_ANGULAR_VELOCITY_Z 1000

#define HARNESS_MODE_COMMAND "HARNESS_MODE=TRUE"
#define HARNESS_MODE_GYRO_OVERRIDE "GYRO_OVERRIDE=TRUE"
#define HARNESS_MODE_ACCEL_OVERRIDE "ACCEL_OVERRIDE=TRUE"
#define HARNESS_MODE_MAG_OVERRIDE "MAG_OVERRIDE=TRUE"
#define HARNESS_MODE_ATTITUDE_OVERRIDE "ATTITUDE_OVERRIDE=TRUE"
#define HARNESS_MODE_PRESSURE_OVERRIDE "PRESSURE_OVERRIDE=TRUE"
#define HARNESS_MODE_ALTITUDE_OVERRIDE "ALTITUDE_OVERRIDE=TRUE"


#define FLIGHT_DATA_GYRO_UPDATE_MILLIS 500
#define FLIGHT_DATA_ACCEL_UPDATE_MILLIS 500
#define FLIGHT_DATA_MAG_UPDATE_MILLIS 500
#define FLIGHT_DATA_ATTITUDE_UPDATE_MILLIS 100
#define FLIGHT_DATA_TEMPERATURE_UPDATE_MILLIS 500
#define FLIGHT_DATA_GPS_UPDATE_MILLIS 2000
#define FLIGHT_DATA_ALTITUDE_UPDATE_MILLIS 100

#define RADIO_TELEMETRY_GYRO_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_ACCEL_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_MAG_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_ATTITUDE_UPDATE_MILLIS 500
#define RADIO_TELEMETRY_TEMPERATURE_UPDATE_MILLIS 3000
#define RADIO_TELEMETRY_GPS_UPDATE_MILLIS 10000
#define RADIO_TELEMETRY_ALTITUDE_UPDATE_MILLIS 500


#define ALTITUDE_NOISE_REDUCTION_UPDATE_MILLIS 500
#define ALTITUDE_NOISE_REDUCTION_SMOOTHER_VALUE 0.96


#define APOGEE_DETECTION_MIN_ALTITUDE 20
#define APOGEE_DETECTION_MIN_LAUNCH_MILLIS 10000


/* Class Declerations */
#include <RadioTelemetry.h>
#include <RecoverySystem.h>
#include <SdStorage.h>
#include <SensorPackage.h>
#include <IgnitionSystem.h>
/* Function Prototypes */

/* Flight Time Variables */
float attitude[3] = {};
float acceleration[3] = {};
float angularVelocities[3] = {};
float magData[3] = {};
float pressure;
float pressureAltitude;
//gps data member

#define ALTITUDE_BUFFER_SIZE 2048
float runningAltitudeData[ALTITUDE_BUFFER_SIZE] = {}; //store once per second? Or start altitude and then once per second from launch?
float currentRunningAltitude = 0;
int runningAltitudeDataIndex = 0;

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

enum FlightPhase{
  ON_GROUND, BOOST_PHASE, COAST_PHASE, DROGUE_CHUTE_PHASE, MAIN_CHUTE_PHASE, LANDED
};

bool harnessModeActive = false;


IgnititonSystem ignitionSystem;
RadioTelemetry radioTelemetry;
SdStorage flightData;
SensorPackage sensorPackage;


void setup() {
  Serial.begin(115200);
  //int
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
    if(serialCommand == HARNESS_MODE_COMMAND){
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
  //else if()

  //update sensors (unless if in Harness Mode)
  //if in harnessMode, chec
}
#define DROGUE_CHUTE_DEPLOYMENT 1
#define MAIN_CHUTE_DEPLOYMENT 2
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



int checkBaseCommands(){
  //check the radio for base station commands and return with codes
}

long flightDataMillisAtLastGyroUpdate = 0;
long flightDataMillisAtLastAccelUpdate = 0;
long flightDataMillisAtLastMagUpdate = 0;
long flightDataMillisAtLastAttitudeUpdate = 0;
long flightDataMillisAtLastTemperatureUpdate = 0;
long flightDataMillisAtLastGPSUpdate = 0;

int reportFlightData(){
  //check for appropriate time to update flight data and do so
  long tempTime = millis();
  if(tempTime - flightDataMillisAtLastAccelUpdate > FLIGHT_DATA_ACCEL_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - flightDataMillisAtLastGyroUpdate > FLIGHT_DATA_GYRO_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - flightDataMillisAtLastMagUpdate > FLIGHT_DATA_MAG_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - flightDataMillisAtLastAttitudeUpdate > FLIGHT_DATA_ATTITUDE_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - flightDataMillisAtLastTemperatureUpdate > FLIGHT_DATA_TEMPERATURE_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - flightDataMillisAtLastGPSUpdate > FLIGHT_DATA_GPS_UPDATE_MILLIS){
    //update accel to SD
  }
  return 0;
}


long telemetryMillisAtLastGyroUpdate = 0;
long telemetryMillisAtLastAccelUpdate = 0;
long telemetryMillisAtLastMagUpdate = 0;
long telemetryMillisAtLastAttitudeUpdate = 0;
long telemetryMillisAtLastTemperatureUpdate = 0;
long telemetryMillisAtLastGPSUpdate = 0;

int sendDataThroughRadio(){
  long tempTime = millis();
  if(tempTime - telemetryMillisAtLastAccelUpdate > FLIGHT_DATA_ACCEL_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - telemetryMillisAtLastGyroUpdate > FLIGHT_DATA_GYRO_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - telemetryMillisAtLastMagUpdate > FLIGHT_DATA_MAG_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - telemetryMillisAtLastAttitudeUpdate > FLIGHT_DATA_ATTITUDE_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - telemetryMillisAtLastTemperatureUpdate > FLIGHT_DATA_TEMPERATURE_UPDATE_MILLIS){
    //update accel to SD
  }
  if(tempTime - telemetryMillisAtLastGPSUpdate > FLIGHT_DATA_GPS_UPDATE_MILLIS){
    //update accel to SD
  }
  return 0;
}

void loop() {
    // put your main code here, to run repeatedly:
    updateSystems();
    //check for recovery events
    int recoveryEvent = handleRecovery();
    //check for new radio commands and handle
    checkBaseCommands();
    reportFlightData();
}

String printFloat(float toPrint, int length){
  String toReturn = String(toPrint, length);
  return toReturn;
}

long millisAtLastAltitudeUpdate = 0;

int addAltitudeData(float data){
  long tempTime = millis();
  currentRunningAltitude = ALTITUDE_NOISE_REDUCTION_SMOOTHER_VALUE*currentRunningAltitude + data*(1.0-ALTITUDE_NOISE_REDUCTION_SMOOTHER_VALUE);
  if(tempTime - millisAtLastAltitudeUpdate >= ALTITUDE_NOISE_REDUCTION_UPDATE_MILLIS){
    runningAltitudeData[runningAltitudeDataIndex] = currentRunningAltitude;
    runningAltitudeDataIndex++;
    if(runningAltitudeDataIndex >= ALTITUDE_BUFFER_SIZE) runningAltitudeDataIndex = 0;
    millisAtLastAltitudeUpdate = millis();
  }
  return 0;
}

#define APOGEE_REACHED 0
#define APOGEE_NOT_REACHED -1
int checkForApogee(){
  int maxAltitudeIndex = 0;
  for(int i = 1; i < runningAltitudeDataIndex; i++){
    if(runningAltitudeData[i] > runningAltitudeData[maxAltitudeIndex]){
      maxAltitudeIndex = i;
    }
  }
  if(maxAltitudeIndex * ALTITUDE_NOISE_REDUCTION_UPDATE_MILLIS >= APOGEE_DETECTION_MIN_LAUNCH_MILLIS && runningAltitudeData[maxAltitudeIndex] >= APOGEE_DETECTION_MIN_ALTITUDE){
    return APOGEE_REACHED;
  }
  return APOGEE_NOT_REACHED;
}

#define DROGUE_CHUTE_PIN 3
#define MAIN_CHUTE_PIN 4
int numDrogueChuteServos = 2;
int numMainChuteServos = 2;
int *drogueChutePins;
int *mainChutePins;


void initRecoverySystem(int mainChutePins[], double mainChuteDeplomentPositions[], int numMainChutes, int drogueChutePins[], double drogueChuteDeplomentPositions[], int numDrogueChutes){

}

int fireChutes(){
  if(FIRE_DROGUE_CHUTE){
    //servo code
  }
}
