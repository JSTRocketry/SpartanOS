#include <Arduino.h>


/*
Status:
Done:
  RadioTelemetry: Good, outgoing buffer not yet tested

Working on:
  Ignition System: TODO

*/

/* Constants */

#define SERIAL_DEBUG
#define APOGEE_REACHED 0
#define APOGEE_NOT_REACHED -1

#define FIRST_STAGE_FIRING_PIN 10

#define MAIN_CHUTE_OVERRIDE_TIMER 100
#define MINIMUM_ALTITUDE 20
#define MAIN_CHUTE_MIN_DEPLOY_ALTITUDE 150
#define MAX_ANGULAR_VELOCITY_X_Y 60
#define MAX_ANGULAR_VELOCITY_Z 1000

#define HARNESS_MODE_COMMAND "HARNESS=TRUE"
#define HARNESS_MODE_GYRO_OVERRIDE "GYRO_OVERRIDE=TRUE"
#define HARNESS_MODE_ACCEL_OVERRIDE "ACCEL_OVERRIDE=TRUE"
#define HARNESS_MODE_MAG_OVERRIDE "MAG_OVERRIDE=TRUE"
#define HARNESS_MODE_ATTITUDE_OVERRIDE "ATTITUDE_OVERRIDE=TRUE"
#define HARNESS_MODE_PRESSURE_OVERRIDE "PRESSURE_OVERRIDE=TRUE"
#define HARNESS_MODE_ALTITUDE_OVERRIDE "ALTITUDE_OVERRIDE=TRUE"


#define FLIGHT_DATA_GYRO_UPDATE_MILLIS 100
#define FLIGHT_DATA_ACCEL_UPDATE_MILLIS -1
#define FLIGHT_DATA_MAG_UPDATE_MILLIS -1
#define FLIGHT_DATA_ATTITUDE_UPDATE_MILLIS -1
#define FLIGHT_DATA_TEMPERATURE_UPDATE_MILLIS -1
#define FLIGHT_DATA_GPS_UPDATE_MILLIS -1
#define FLIGHT_DATA_ALTITUDE_UPDATE_MILLIS -1

#define RADIO_TELEMETRY_GYRO_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_ACCEL_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_MAG_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_ATTITUDE_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_TEMPERATURE_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_GPS_UPDATE_MILLIS -1
#define RADIO_TELEMETRY_ALTITUDE_UPDATE_MILLIS -1

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
#include <GPS_Handler.h>
/* Function Prototypes */

/* Flight Time Variables */
float attitude[3] = {};
float acceleration[3] = {};
float angularVelocities[3] = {};
float magData[3] = {};
float temperature = 0;
GPS_Handler::GpsLocationData location;
float pressure;
float altitude;
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
  WAITING_FOR_SYNC, WAITING_FOR_ARM, WAITING_FOR_LAUNCH, IN_FLIGHT, STAND_DOWN_CALLED, ABORT_CALLED
};

enum FlightPhase{
  ON_GROUND, BOOST_PHASE, COAST_PHASE, DROGUE_CHUTE_PHASE, MAIN_CHUTE_PHASE, LANDED
};


String flightPhaseToString(FlightPhase phase){
  switch (phase){
    case ON_GROUND:
      return "On Ground";
      break;
    case BOOST_PHASE:
      return "Boost Phase";
      break;
    case COAST_PHASE:
      return "Coast Phase";
      break;
    case DROGUE_CHUTE_PHASE:
      return "Drogue Chute Phase";
      break;
    case MAIN_CHUTE_PHASE:
      return "Main Chute Phase";
      break;
    case LANDED:
      return "Landed";
      break;
  }
  return "ERROR";
}

FlightMode rocketFlightMode = WAITING_FOR_SYNC;
FlightPhase rocketFlightPhase = ON_GROUND;
bool harnessModeActive = false;
bool altitudeOverride = false;
bool gyroOverride = false;
bool accelOverride = false;
bool magOverride = false;
bool gpsOverride = false;
bool attitudeOverride = false;

IgnitionSystem ignitionSystem;
RadioTelemetry radioTelemetry;
RecoverySystem recoverySystem;
SdStorage flightData;
SensorPackage sensorPackage;



String floatToString(float toPrint, int length);
int addAltitudeData(float data);
int checkForApogee();

void serialPrint(String message){
  #ifdef SERIAL_DEBUG
    Serial.println(message);
  #endif
}

void setup() {
  Serial.begin(115200);
  int errors = sensorPackage.begin();
  serialPrint("Sensor Package Errors: " + String(errors));
  //errors = flightData.begin();
  //serialPrint("Flight Data errors: " + String(errors));
  ignitionSystem.begin(FIRST_STAGE_FIRING_PIN);
  recoverySystem.begin();
  radioTelemetry.begin();
  errors = flightData.begin();
  if(errors != 0){
    radioTelemetry.sendData("SD init FAILED!");
    while(true){
      radioTelemetry.update();
      serialPrint("Flight Recording Errors: " + String(errors));
      delay(500);
    }
  }


  //int
    // put your setup code here, to run once:
  serialPrint("Init Done");
}

String serialBuffer;
long timeAtLastWrite = 0;
int checkForSerialCommand(String *line){
  if(millis() - timeAtLastWrite > 20){
    serialBuffer = "";
  }
  serialPrint("Serial Length: " + String(serialBuffer.length()));
  if(Serial.available()){
    while(Serial.available()){
      char c = (char)Serial.read();
      if(c == '\n' || c =='\r'){
        *line = serialBuffer;
        serialBuffer = "";
        return 0;
      }
      timeAtLastWrite = millis();
      serialBuffer += c;
    }
  }
  return -1;
}


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

bool harnessModeGyroOverride = false;
bool harnessModeAccelOverride = false;
bool harnessModeMagOverride = false;
bool harnessModeAttitudeOverride = false;
bool harnessModealtitudeOverride = false;

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
  serialPrint("New Harness Command: " + serialCommand);
  if(!harnessModeActive){
    if(serialCommand.indexOf(HARNESS_MODE_COMMAND) >= 0){
      harnessModeActive = true;
      serialPrint("Harness override command given!");
      return NEW_HARNESS_MODE_COMMAND;
    }
  }
  else if(harnessModeActive){
    /*
    split the message up:
    ALT:1000;
    GYRO_X:10
    */
    String sensor = serialCommand.substring(0,serialCommand.indexOf(":"));
    String sensorValue = serialCommand.substring(serialCommand.indexOf(":") + 1, serialCommand.indexOf(";"));
    Serial.println("Sensor: " + sensor + " Value: " + sensorValue);
    //deal with the values now
    if(sensor.indexOf("ALT") >= 0){
      altitude = atof(sensorValue.c_str());
      serialPrint("New Altitude given over harness: " + sensorValue);
      altitudeOverride = true;
    }
  }
}

void updateSystems(){
  serialPrint("Updating Sensors!");
  serialPrint("Avail Mem: " + String(freeMemory()));
  recoverySystem.update();
  radioTelemetry.update();
  sensorPackage.update();
  checkForHarnessCommand();

  if(!attitudeOverride){
    sensorPackage.getAttitudeData(&attitude[0], &attitude[1], &attitude[2]);
  }
  if(!altitudeOverride){
    sensorPackage.getAltitudeData(&altitude);
  }
  serialPrint("Altitude: " + String(altitude));
  if(!gyroOverride){
    sensorPackage.getGyroData(&angularVelocities[0], &angularVelocities[1], &angularVelocities[2]);
  }
  if(!accelOverride){
    sensorPackage.getAccelData(&acceleration[0], &acceleration[1], &acceleration[2]);
  }
  if(!gpsOverride){
    sensorPackage.getGPSData(&location);
  }
  serialPrint("update Systems done");
  //sensorPackage.getTemperatureData(&temperature);

  //else if()

  //update sensors (unless if in Harness Mode)
  //if in harnessMode, chec
}
#define DROGUE_CHUTE_DEPLOYMENT 1
#define MAIN_CHUTE_DEPLOYMENT 2
int handleRecovery(){
  if(hasLaunchBeenTriggeredByMaster){
    addAltitudeData(altitude);
    if(!hasApogeeBeenDetected){
      //check out our new altitude, make sure legit
      if(checkForApogee() == APOGEE_REACHED){
        hasApogeeBeenDetected = true;
        //deploy drogue
        recoverySystem.deployDrogueChute();
        rocketFlightPhase = DROGUE_CHUTE_PHASE;
        serialPrint("Apogee detection! Drogue Chute!");
        recoverySystem.deployDrogueChute();
        hasDrogueChuteBeenDeployed = true;
        return DROGUE_CHUTE_DEPLOYMENT;
      }

    }
    else if(hasApogeeBeenDetected){
      if(!hasMainChuteBeenDeployed){
        //check if at altitude to deploy chute
        if(altitude <= MAIN_CHUTE_MIN_DEPLOY_ALTITUDE){
          //deploy main chute
          hasMainChuteBeenDeployed = true;
          rocketFlightPhase = MAIN_CHUTE_PHASE;
          recoverySystem.deployMainChute();
          serialPrint("Main Chute Deployment!");
          return MAIN_CHUTE_DEPLOYMENT;
        }
      }
    }
  }
  return 0;
}

void printRocketState(){
  String runningMessage;
  int radioStatus = radioTelemetry.checkRadioStatus();
  runningMessage += "Radio State: ";
  if(radioStatus == SYNC_ACQUIRED_SENDING_REALTIME){
    runningMessage += "ACQUIRED";
  }
  else{
    runningMessage += "Not Acquired";
  }
  runningMessage += "Flight Phase: " + flightPhaseToString(rocketFlightPhase);
  serialPrint(runningMessage);
}

int checkBaseCommands(){
  //check the radio for base station commands and return with codes
  //arm, launch, abort
  if(rocketFlightMode == WAITING_FOR_SYNC && radioTelemetry.checkRadioStatus() == SYNC_ACQUIRED_SENDING_REALTIME){
    rocketFlightMode = WAITING_FOR_ARM;
  }
  int numAvail = radioTelemetry.available();
  if(numAvail > 0){
    String messageBuffer[numAvail];
    radioTelemetry.getCurrentMessages(&messageBuffer[0], &numAvail);
    for(int i = 0; i < numAvail; i ++){
      //look for message type, handle
      if(messageBuffer[i].indexOf("ARM") >= 0){
        //arm command
        rocketFlightMode = WAITING_FOR_LAUNCH;
        radioTelemetry.sendData("ARM Recieved");
        serialPrint("ARM Recieved");
        ignitionSystem.arm();
      }
      if(messageBuffer[i].indexOf("LAUNCH") >= 0){
        if(rocketFlightMode == WAITING_FOR_LAUNCH){
          rocketFlightMode = IN_FLIGHT;
          radioTelemetry.sendData("LAUNCH Recieved, launching");
          serialPrint("LAUNCH Recieved, launching");
          ignitionSystem.fire();
          hasLaunchBeenTriggeredByMaster = true;
        }
        else{
          ignitionSystem.standDown();
          rocketFlightMode = ABORT_CALLED;
          radioTelemetry.sendData("NOT CORRECT LAUNCH ORDER, STANDING DOWN");
          serialPrint("NOT CORRECT LAUNCH ORDER, STANDING DOWN");
        }
      }
      if(messageBuffer[i].indexOf("ABORT") >= 0){
        if(rocketFlightMode == WAITING_FOR_ARM || rocketFlightMode == WAITING_FOR_LAUNCH){
          ignitionSystem.standDown();
          radioTelemetry.sendData("ABORT Recived, ignition system standing down");
          serialPrint("ABORT Recived, ignition system standing down");
        }
        else if(rocketFlightMode == IN_FLIGHT){
          recoverySystem.deployMainChute();
          radioTelemetry.sendData("ABORT Recieved, deploying parachute");
          serialPrint("ABORT Recieved, deploying parachute");
        }
      }
    }
  }
  return 0;
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
  if((FLIGHT_DATA_ACCEL_UPDATE_MILLIS != -1) & (tempTime - flightDataMillisAtLastAccelUpdate > FLIGHT_DATA_ACCEL_UPDATE_MILLIS)){
    //update accel to SD
    flightData.writeData("@{AX:" + String(acceleration[0]) + ";AY:" + String(acceleration[1]) + ";AZ:" + String(acceleration[2]) + ";TS:" + String(systemTime) + ";}@");
    flightDataMillisAtLastAccelUpdate = tempTime;
  }
  if((FLIGHT_DATA_GYRO_UPDATE_MILLIS != -1) & (tempTime - flightDataMillisAtLastGyroUpdate > FLIGHT_DATA_GYRO_UPDATE_MILLIS)){
    //update accel to SD
    flightData.writeData("@{GX:" + String(angularVelocities[0]) + ";GY:" + String(angularVelocities[1]) + ";GZ:" + String(angularVelocities[2]) + ";TS:" + String(systemTime) + ";}@");
    flightDataMillisAtLastGyroUpdate = tempTime;
  }
  if((FLIGHT_DATA_MAG_UPDATE_MILLIS != -1) & (tempTime - flightDataMillisAtLastMagUpdate > FLIGHT_DATA_MAG_UPDATE_MILLIS)){
    //update accel to SD
    flightData.writeData("@{MX:" + String(magData[0]) + ";MY:" + String(magData[1]) + ";MZ:" + String(magData[2]) + ";TS:" + String(systemTime) + ";}@");
    flightDataMillisAtLastMagUpdate = tempTime;
  }
  if((FLIGHT_DATA_ATTITUDE_UPDATE_MILLIS != -1) & (tempTime - flightDataMillisAtLastAttitudeUpdate > FLIGHT_DATA_ATTITUDE_UPDATE_MILLIS)){
    //update accel to SD
    flightData.writeData("@{OX:" + String(attitude[0]) + ";OY:" + String(attitude[1]) + ";OZ:" + String(attitude[2]) + ";TS:" + String(systemTime) + ";}@");
    flightDataMillisAtLastAttitudeUpdate = tempTime;
  }
  if((FLIGHT_DATA_TEMPERATURE_UPDATE_MILLIS != -1) & (tempTime - flightDataMillisAtLastTemperatureUpdate > FLIGHT_DATA_TEMPERATURE_UPDATE_MILLIS)){
    //update accel to SD
    flightData.writeData("@{T:" + String(temperature) + ";TS:" + String(systemTime) + "}@");
    flightDataMillisAtLastTemperatureUpdate = tempTime;
  }
  if((FLIGHT_DATA_GPS_UPDATE_MILLIS != -1) & (tempTime - flightDataMillisAtLastGPSUpdate > FLIGHT_DATA_GPS_UPDATE_MILLIS)){
    //update accel to SD

    flightData.writeData("@{LA:" + floatToString(location.latitude,6) + ";LO:" + floatToString(location.longitude,6) + ";TS:" + String(systemTime) + ";}@");
    flightDataMillisAtLastGPSUpdate = tempTime;
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
  serialPrint("Radio update start");
  if(radioTelemetry.checkForSync() != SYNC_ACQUIRED_SENDING_REALTIME){
    radioTelemetry.attemptSync();
    serialPrint("Attempting radio sync!");
    return -5;
  }
  if((RADIO_TELEMETRY_ACCEL_UPDATE_MILLIS != -1) && (tempTime - telemetryMillisAtLastAccelUpdate > RADIO_TELEMETRY_ACCEL_UPDATE_MILLIS)){
    //update accel to SD
    serialPrint("Updating Accel over Radio");
    radioTelemetry.sendData("@{AX:" + String(acceleration[0]) + ";AY:" + String(acceleration[1]) + ";AZ:" + String(acceleration[2]) + ";TS:" + String(systemTime) + ";}@");
    telemetryMillisAtLastAccelUpdate = tempTime;
  }
  if((RADIO_TELEMETRY_GYRO_UPDATE_MILLIS != -1) && (tempTime - telemetryMillisAtLastGyroUpdate > RADIO_TELEMETRY_GYRO_UPDATE_MILLIS)){
    //update accel to SD
    serialPrint("Updating gyro over radio");
    radioTelemetry.sendData("@{GX:" + String(angularVelocities[0]) + ";GY:" + String(angularVelocities[1]) + ";GZ:" + String(angularVelocities[2]) + ";TS:" + String(systemTime) + ";}@");
    telemetryMillisAtLastGyroUpdate = tempTime;
  }
  if((RADIO_TELEMETRY_MAG_UPDATE_MILLIS != -1) && (tempTime - telemetryMillisAtLastMagUpdate > RADIO_TELEMETRY_MAG_UPDATE_MILLIS)){
    //update accel to SD
    serialPrint("Updating mag over radio");
    radioTelemetry.sendData("@{MX:" + String(magData[0]) + ";MY:" + String(magData[1]) + ";MZ:" + String(magData[2]) + ";TS:" + String(systemTime) + ";}@");
    telemetryMillisAtLastMagUpdate = tempTime;
  }
  if((RADIO_TELEMETRY_ATTITUDE_UPDATE_MILLIS != -1) && (tempTime - telemetryMillisAtLastAttitudeUpdate > RADIO_TELEMETRY_ATTITUDE_UPDATE_MILLIS)){
    //update accel to SD
    serialPrint("Updating attitude over radio");
    radioTelemetry.sendData("@{OX:" + String(attitude[0]) + ";OY:" + String(attitude[1]) + ";OZ:" + String(attitude[2]) + ";TS:" + String(systemTime) + ";}@");
    telemetryMillisAtLastAttitudeUpdate = tempTime;
  }
  if((RADIO_TELEMETRY_TEMPERATURE_UPDATE_MILLIS != -1) && (tempTime - telemetryMillisAtLastTemperatureUpdate > RADIO_TELEMETRY_TEMPERATURE_UPDATE_MILLIS)){
    //update accel to SD
    serialPrint("Updating temperature over radio");
    radioTelemetry.sendData("@{T:" + String(temperature) + ";TS:" + String(systemTime) + "}@");
    telemetryMillisAtLastTemperatureUpdate = tempTime;
  }
  if((RADIO_TELEMETRY_GPS_UPDATE_MILLIS != -1) && (tempTime - telemetryMillisAtLastGPSUpdate > RADIO_TELEMETRY_GPS_UPDATE_MILLIS)){
    //update accel to SD
    serialPrint("Updating gps over radio");
    radioTelemetry.sendData("@{LA:" + floatToString(location.latitude,6) + ";LO:" + floatToString(location.longitude,6) + ";TS:" + String(systemTime) + ";}@");
    telemetryMillisAtLastGPSUpdate = tempTime;
  }
  serialPrint("Radio update end");
  return 0;
}

void loop() {
    // put your main code here, to run repeatedly:
    updateSystems();
    //check for recovery events
    //handleRecovery();
    //check for new radio commands and handle
    checkBaseCommands();
    reportFlightData();
    sendDataThroughRadio();
    printRocketState();
    delay(100);
}

String floatToString(float toPrint, int length){
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
