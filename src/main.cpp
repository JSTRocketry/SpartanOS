#include <Arduino.h>

/* Constants */

/* Class Declerations */

/* Function Prototypes */

/* Flight Time Variables */

/* System Variables */


class RecoverySystem{
public:
  int begin(int mainServo, long mainLockPosition, long mainReleasePosition, int drogueServo, long drogueLockPosition, long drogueReleasePosition);
  int lock();
  int deployDrogueChute();
  int deployMainChute();
private:

};

void setup() {
    // put your setup code here, to run once:
}

void loop() {
    // put your main code here, to run repeatedly:
}
