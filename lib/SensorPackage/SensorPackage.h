#ifndef _SENSOR_PACKAGE_H_
#define _SENSOR_PACKAGE_H_

#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_BMP280.h>

class SensorPackage{
public:
  int begin();
  int update();
  int getAccelData(float *xData, float *yData, float *zData);
  int getGyroData(float *xData, float *yData, float *zData);
  int getMagData(float *xData, float *yData, float *zData);
  int getAttitudeData(float *xData, float *yData, float *zData);
  int getPressureData(float *pressure);
  int getAltitudeData(float *deltaAltitude);
  int getTemperatureData(float *temperature);
  int getGPSData();
private:
  Adafruit_BMP280 pressure;
  MPU9250 imu;
  MPU9250_Data imuData;

};
#endif
