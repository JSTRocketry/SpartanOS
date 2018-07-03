#ifndef _RECOVERY_SYSTEM_H_
#define _RECOVERY_SYSTEM_H_

#include <Arduino.h>

class RecoverySystem{
public:
  int begin(int mainServo, long mainLockPosition, long mainReleasePosition, int drogueServo, long drogueLockPosition, long drogueReleasePosition);
  int lock();
  int deployDrogueChute();
  int deployMainChute();
private:

};

#endif
