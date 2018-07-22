#ifndef _SENSOR_PACKAGE_H_
#define _SENSOR_PACKAGE_H_

#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include "GPS_Handler.h"

/* error codes */
#define SENSOR_PACKAGE_PRESSURE_SENSOR_INIT_FAILED  0b00000001
#define SENSOR_PACKAGE_IMU_SENSOR_INIT_FAILED       0b00000010
#define SENSOR_PACKAGE_GPS_SENSOR_INIT_FAILED       0b00000100


/* update returns */
#define SENSOR_PACKAGE_NEW_PRESSURE_DATA            0b00000001
#define SENSOR_PACKAGE_NEW_ACCEL_DATA               0b00000010
#define SENSOR_PACKAGE_NEW_GYRO_DATA                0b00000100
#define SENSOR_PACKAGE_NEW_MAG_DATA                 0b00001000
#define SENSOR_PACKAGE_NEW_ATTITUDE_DATA            0b00010000
#define SENSOR_PACKAGE_NEW_GPS_DATA                 0b00100000
#define SENSOR_PACKAGE_NEW_TEMPERATURE_DATA         0b01000000


#define SENSOR_PACKAGE_PRESSURE_UPDATE_MILLIS 100
#define SENSOR_PACKAGE_IMU_UPDATE_MILLIS 5
#define SENSOR_PACKAGE_GPS_UPDATE_MILLIS 2000
#define SENSOR_PACKAGE_TEMPERATURE_UPDATE_MILLIS 1000

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
  int getGPSData(GPS_Handler::GpsLocationData *location);
  int resetForFlight();
private:
  float curPressureData;
  float zeroAltitudePressure;
  float curTemperatureData;
  float curAccelData[3]= {};
  float curGyroData[3] = {};
  float curMagData[3] = {};
  float curAttitudeData[3] = {};
  float curAltitude;

  long lastPressureUpdate = 0;
  long lastGyroUpdate = 0;
  long lastAccelUpdate = 0;
  long lastMagUpdate = 0;
  long lastGPSUpdate = 0;
  long lastTempUpdate = 0;
  GPS_Handler::GpsLocationData locationData;

  int updateGyro();
  int updateAccel();
  int updateMag();
  int updatePressure();
  int updateAttitude();
  int updateGPS();
  Vector3f genVector;
  // cur gps data
  Adafruit_BMP280 pressure;
  MPU9250 imu;
  MPU9250_Data imuData;
  GPS_Handler *gps;



};
#endif
