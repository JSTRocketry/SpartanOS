#include "GPS_Handler.h"

GPS_Handler::GPS_Handler(HardwareSerial *serial){
  gpsSerial.begin(serial, 9600);
}

int GPS_Handler::getLocationData(GpsLocationData *locationData){
  if(!goodDataAvailable) return -1;
  *locationData = curLocationData;
  return 0;
}

int GPS_Handler::update(){
  int numNewUpdates = 0;
  GpsLocationData tempGpsData;
  while(gpsSerial.readLine(&tempLine) == 0){
    //Serial.println("New Data From GPS");
    numNewUpdates ++;
    if(parseGPSLine(tempLine, &tempGpsData) == 0){
      //Serial.println("Good Line!");
      if(!goodDataAvailable) goodDataAvailable = true;
      //maybe do extra checks?
      tempGpsData.timeSinceLastUpdate = timeAtLastUpdate - millis();
      timeAtLastUpdate = millis();
      curLocationData = tempGpsData;
      numNewUpdates ++;
    }
    /*
    else{
      Serial.println("Bad Line!");
    }
    */
  }
  if(numNewUpdates > 0) return 0;
  if(!goodDataAvailable) return GPS_NOT_YET_ACQUIRED;
  return GPS_NO_NEW_DATA;
}



float GPS_Handler::DMStoDecimal(float dms){
  char brokenDMS[6];
  String(dms).toCharArray(brokenDMS, 6);
  float degree = atof((String(brokenDMS[0]) + String(brokenDMS[1])).c_str());
  float minutes = atof((String(brokenDMS[2]) + String(brokenDMS[3])).c_str());
  float secconds = atof((String(brokenDMS[4]) + String(brokenDMS[5]) + String(brokenDMS[6])).c_str());
  float decimal = degree + (minutes/60) + (secconds/3600);
  return decimal;
}

int GPS_Handler::parseGPSLine(String newData, GpsLocationData *parsedData){
  //Serial.println("New Data:"  + newData);
  if (newData.indexOf("$GPGGA") == -1){
    return -1;
  }
  int commaCount[13];
  commaCount[0] = newData.indexOf(",");
  for (int i = 1; i < 13; i++){
    commaCount[i] = newData.indexOf(",", commaCount[i-1]+1);
    if(commaCount[i] < 0 || commaCount[i] >= (int)newData.length()){
      //Serial.println("Something wrong with gps parsing");
      return -1;
    }
  }
  int addon, sumon;
  if (strcmp(newData.substring(commaCount[2] + 1, commaCount[3]).c_str(),  String("N").c_str()) == 0){
    addon = 1;
  }
  else{
    addon = -1;
  }
  if (strcmp(newData.substring(commaCount[4] + 1, commaCount[5]).c_str(), String("E").c_str()) == 0){
    sumon = 1;
  }
  else{
    sumon = -1;
  }
  parsedData->timestamp = atol(newData.substring((commaCount[0] + 1), commaCount[1]).c_str());
  parsedData->latitude = DMStoDecimal((atof(newData.substring((commaCount[1] + 1), commaCount[2]).c_str()))) * addon;
  parsedData->longitude = DMStoDecimal((atof(newData.substring((commaCount[3] + 1), commaCount[4]).c_str()))) * sumon;
  parsedData->altitude = atof((newData.substring(commaCount[8] + 1, commaCount[9])).c_str());
  switch(atoi((newData.substring(commaCount[5] + 1, commaCount[6])).c_str())){
    case 0:
      parsedData->fixQuality = GPS_FIX_INVALID;
      break;
    case 1:
      parsedData->fixQuality = GPS_FIX;
      break;
    default:
      parsedData->fixQuality = GPS_FIX_INVALID;
      break;
  }
  parsedData->numSatillites = atoi((newData.substring(commaCount[6] + 1, commaCount[7])).c_str());
  //gpsData->heightOfGeoid = atof((line.substring(commaCount[10] + 1, commaCount[11])).c_str());
  return 0;
}
