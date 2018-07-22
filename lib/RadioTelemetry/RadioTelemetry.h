#ifndef _RADIO_TELEMETRY_H_
#define _RADIO_TELEMETRY_H_

#include <Arduino.h>

#define RADIO_BUFFER_SIZE 64
#define WAITING_FOR_SYNC_BUFFERING_MESSAGES -1
#define SYNC_ACQUIRED_SENDING_REALTIME -2


#define RADIO_BAUD 2400
#define RADIO_SYNC_REQUEST "PING"
#define RADIO_SYNC_RETURN "ECHO"

class RadioTelemetry{
public:
  int begin();
  int update();
  int checkForSync();
  int sendData(String data);
  int checkForBaseCommand();
  int available();
  void getCurrentMessages(String * buffer, int *numberAvailable);
  int checkRadioStatus();
private:
  String runningMessage;
  String recievedMessagesBuffer[16];
  int recievedMessagesBufferIndex = 0;
  String preSyncBuffer[RADIO_BUFFER_SIZE];
  int curBufferIndex = 0;
  bool hasSynced = false;
  void writeBuffer(String * buff, int size);
  void updateIncommingBuffer();

};


#endif
