#ifndef _IGNITION_SYSTEM_H_
#define _IGNITION_SYSTEM_H_

class IgnititonSystem{
public:
  int begin(int firingPin);
  int arm();
  int fire();
  int standDown();
private:
  int firingPin = -1; 
};

#endif
