#include <SensorPackage.h>



int SensorPackage::begin(){
  //init all sensors, check for any init fails
  //return an int with all failures recorded
  int errors = 0;
  int imuInitStatus = imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS);
  if(imuInitStatus < 0) errors &= SENSOR_PACKAGE_IMU_SENSOR_INIT_FAILED;
  int bmpInitStatus = pressure.begin();
  if(bmpInitStatus < 0) errors &= SENSOR_PACKAGE_PRESSURE_SENSOR_INIT_FAILED;
  //gps init

  return errors;
}

int SensorPackage::update(){
  //check scheduling and update necessary sensors
  //return code with all new sensors that have been updated
  int updatedSensorCodes = 0;
  return updatedSensorCodes;
}

int SensorPackage::getAccelData(float *xData, float *yData, float *zData){
  *xData = curAccelData[0];
  *yData = curAccelData[1];
  *zData = curAccelData[2];
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getGyroData(float *xData, float *yData, float *zData){
  *xData = curGyroData[0];
  *yData = curGyroData[1];
  *zData = curGyroData[2];
  return 0; //idk what the return codes should be here....
}

int SensorPackage::getGPSData(){
  //TODO implement
  return -1; //not yet implemented
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
