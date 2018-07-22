#include "RadioTelemetry.h"


int RadioTelemetry::begin(){
  Serial3.begin(RADIO_BAUD);
  Serial3.print(RADIO_SYNC_REQUEST);
  return 0;
}

int RadioTelemetry::update(){
  if(!hasSynced){
    if(checkForSync() == 0){
      writeBuffer(preSyncBuffer, curBufferIndex);
      return SYNC_ACQUIRED_SENDING_REALTIME;
    }
    return WAITING_FOR_SYNC_BUFFERING_MESSAGES;
  }
  else{
    //already synced, update incomming buffer
    updateIncommingBuffer();
    return SYNC_ACQUIRED_SENDING_REALTIME;

  }
}

int RadioTelemetry::checkRadioStatus(){
  if(hasSynced){
    return SYNC_ACQUIRED_SENDING_REALTIME;
  }
  return WAITING_FOR_SYNC_BUFFERING_MESSAGES;
}

void RadioTelemetry::writeBuffer(String * buff, int size){
  //write the buffer!
  for(int i = 0; i < size; i ++){
    Serial3.println(buff[i]);
  }
}

int RadioTelemetry::available(){
  return recievedMessagesBufferIndex;
}

void RadioTelemetry::getCurrentMessages(String * buffer, int * numberAvailable){
  for(int i = 0; i < recievedMessagesBufferIndex; i ++){
    buffer[i] = recievedMessagesBuffer[i];
  }
  buffer = recievedMessagesBuffer;
  *numberAvailable = recievedMessagesBufferIndex;
  //clear the stack
  for(int i = 0; i < 16; i ++){
    recievedMessagesBuffer[i] = "";
  }
  recievedMessagesBufferIndex = 0;
}

void RadioTelemetry::updateIncommingBuffer(){
  if(Serial3.available()){
    while(Serial3.available()){
      char c = (char)Serial3.read();
      if(c == '\n' || c == '\r'){
        if(runningMessage.length() > 0){
          recievedMessagesBuffer[recievedMessagesBufferIndex] = runningMessage;
          if(recievedMessagesBufferIndex == 15){
            recievedMessagesBufferIndex = 0;
          }
          else{
              recievedMessagesBufferIndex ++;
          }
          runningMessage = "";
        }
      }
      else{
          runningMessage += c;
      }
    }
  }
}

int RadioTelemetry::checkForSync(){
  if(Serial3.available()){
    while(Serial3.available()){
      char c = (char)Serial3.read();
      Serial.println(c);
      if(c == '\n' || c == '\r'){
        if(runningMessage.indexOf(RADIO_SYNC_RETURN) >= 0){
          hasSynced = true;
          return 0;
        }
        else{
          //idk what to do here....
        }
      }
      else runningMessage += c;
    }
  }
  return -1;
}
