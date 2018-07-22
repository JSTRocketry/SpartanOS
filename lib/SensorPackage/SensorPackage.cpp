#include "SensorPackage.h"



int SensorPackage::begin(){
  //init all sensors, check for any init fails
  //return an int with all failures recorded
  int errors = 0;
  int imuInitStatus = imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS);
  if(imuInitStatus < 0) errors &= SENSOR_PACKAGE_IMU_SENSOR_INIT_FAILED;
  int bmpInitStatus = pressure.begin();
  if(bmpInitStatus < 0) errors &= SENSOR_PACKAGE_PRESSURE_SENSOR_INIT_FAILED;
  //gps init
  gps = new GPS_Handler(&Serial2);



  zeroAltitudePressure = pressure.readPressure();
  return errors;
}

int SensorPackage::update(){
  //check scheduling and update necessary sensors
  //return code with all new sensors that have been updated
  int updatedSensorCodes = 0;
  long curTime = millis();
  if(curTime - lastPressureUpdate > SENSOR_PACKAGE_PRESSURE_UPDATE_MILLIS){
    curPressureData = pressure.readPressure();
    curAltitude = pressure.readAltitude(zeroAltitudePressure);
  }
  return updatedSensorCodes;
}

int SensorPackage::getAccelData(float *xData, float *yData, float *zData){
  *xData = curAccelData[0];
  *yData = curAccelData[1];
  *zData = curAccelData[2];
  return 0; //idk what the return codes should be here....
}


int SensorPackage::updateGyro(){
  imu.getGyroData(&this->genVector);
  curGyroData[0] = genVector.x;
  curGyroData[1] = genVector.y;
  curGyroData[2] = genVector.z;
  return 0;
}

int SensorPackage::updateAccel(){
  imu.getAccelData(&this->genVector);
  curAccelData[0] = genVector.x;
  curAccelData[1] = genVector.y;
  curAccelData[2] = genVector.z;
  return 0;
}

int SensorPackage::updateGPS(){
  int gpsStatus = gps->update();
  if(gpsStatus == 0){
    gps->getLocationData(&locationData);
  }
  return 0;
}

int SensorPackage::updateMag(){
  //TODO implement?
  return -5;
}

int SensorPackage::updateAttitude(){
  imu.getData(&imuData);
  curAttitudeData[0] = imuData.orientation.x;
  curAttitudeData[1] = imuData.orientation.y;
  curAttitudeData[2] = imuData.orientation.z;
  return 0;
}

int SensorPackage::resetForFlight(){
  zeroAltitudePressure = pressure.readPressure();
  return 0;
}

int SensorPackage::getGyroData(float *xData, float *yData, float *zData){
  *xData = curGyroData[0];
  *yData = curGyroData[1];
  *zData = curGyroData[2];
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getGPSData(GPS_Handler::GpsLocationData *location){
  *location = locationData;
  return 0;
}

int SensorPackage::getMagData(float *xData, float *yData, float *zData){
  *xData = curMagData[0];
  *yData = curMagData[1];
  *zData = curMagData[2];
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getAttitudeData(float *xData, float *yData, float *zData){
  *xData = curAttitudeData[0];
  *yData = curAttitudeData[1];
  *zData = curAttitudeData[2];
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getTemperatureData(float *temperature){
  *temperature = curTemperatureData;
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getPressureData(float *pressure){
  *pressure = curPressureData;
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getAltitudeData(float *deltaAltitude){
  *deltaAltitude = curAltitude;
  return 0; //idk what the return codes should be here....
}
