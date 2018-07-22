#ifndef _GPS_HANDLER_H_
#define _GPS_HANDLER_H_
#include <Arduino.h>
#include "BetterSerial.h"

#define GPS_NO_NEW_DATA -1
#define GPS_NOT_YET_ACQUIRED -2


class GPS_Handler{
public:
  enum FixQuality{
    GPS_FIX_INVALID, GPS_FIX
  };

  struct GpsLocationData{
      float latitude;
      float longitude;
      float altitude;
      int numSatillites;
      long timestamp;
      FixQuality fixQuality;
      long timeSinceLastUpdate;
  };
  GPS_Handler(HardwareSerial *serial);
  int update();
  int getLocationData(GpsLocationData *locationData);
private:
  int parseGPSLine(String newData, GpsLocationData *parsedData);
  float DMStoDecimal(float dms);
  bool goodDataAvailable = false;
  long timeAtLastUpdate = 0;
  GpsLocationData curLocationData;
  BetterSerial gpsSerial;
  String tempLine;
  //HardwareSerial serial;
};

#endif
