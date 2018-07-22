#ifndef _BETTER_SERIAL_H_
#define _BETTER_SERIAL_H_
#include <Arduino.h>

class BetterSerial{
public:
  void begin(HardwareSerial *ser, long baud);
  int readLine(String *newLine);
private:
  String buff;
  char tempChar;
  HardwareSerial *ser;
};

#endif
