#include "BetterSerial.h"


void BetterSerial::begin(HardwareSerial *ser, long baud){
  this->ser = ser;
  ser->begin(baud);
}


int BetterSerial::readLine(String *newLine){
  if(ser->available()){
    while(ser->available()){
      tempChar = (char) ser->read();
      if(tempChar == '\n' || tempChar == '\r'){
        if(buff.length() != 0){
          *newLine = buff;
          buff = "";
          return 0;
        }
        return -1;
      }
      buff += tempChar;
    }
  }
  return -1;
}
