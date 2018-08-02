#ifndef _RECOVERY_SYSTEM_H_
#define _RECOVERY_SYSTEM_H_

#include <Arduino.h>
#include <ParachuteDeployer.h>
class RecoverySystem{
public:
  int begin();
  int deployDrogueChute();
  int deployMainChute();
  int update();
private:
  ParachuteDeployer mainChute;
  ParachuteDeployer drogueChute;
  int numMainChuteServos = 1;
  int numDrogueChuteServos = 1;
  int mainChuteServoPins[1] = {1};
  int mainChuteStorePosition[1] = {0};
  int mainChuteFirePosition[1] = {100};
  int drogueChuteServoPins[1] = {2};
  int drogueChuteStorePosition[1] = {0};
  int drogueChuteFirePosition[1] = {100};
};

#endif
