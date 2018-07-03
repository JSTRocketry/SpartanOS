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

    }
  }
}

void RadioTelemetry::writeBuffer(String * buff, int size){

}



void RadioTelemetry::updateIncommingBuffer(){
  if(Serial3.available()){
    char c = (char)Serial3.read();
    runningMessage += c;
  }
}

int RadioTelemetry::checkForSync(){
  if(Serial3.available()){
    char c = (char)Serial3.read();
    if(c == '\n' || c == '\r'){
      if(runningMessage.equals(RADIO_SYNC_RETURN)){
        hasSynced = true;
        return 0;
      }
      else{
        //idk what to do here....
      }
    }
  }
  return -1;
}
