#include <RecoverySystem.h>



int RecoverySystem::begin(){
  mainChute.begin(&mainChuteServoPins[0], &mainChuteStorePosition[0], &mainChuteFirePosition[0], numMainChuteServos);
  drogueChute.begin(&drogueChuteServoPins[0], &drogueChuteStorePosition[0], &drogueChuteFirePosition[0], numDrogueChuteServos);
  return 0;
}

int RecoverySystem::deployDrogueChute(){
  drogueChute.deploy();
  return 0;
}

int RecoverySystem::deployMainChute(){
  mainChute.deploy();
  return 0;
}

int RecoverySystem::update(){
  drogueChute.update();
  mainChute.update();
  return 0;
}
