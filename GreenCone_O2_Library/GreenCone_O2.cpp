#include "GreenCone_O2.h"

GreenCone_O2::GreenCone_O2(uint8_t sensorName){
  name = sensorName;
}

void GreenCone_O2::Init(){
  switch (name)
  {
  case OXYGEN_SENSOR:
    while (!oxygen.begin(OXYGEN_SENSOR_Address))
    {
      status=0;
    }
    status=1;
    break;
  case MULTIGAS_SENSOR:
    while (!gas.begin())
    {
      status=0;
    }
    status=1;
    gas.changeAcquireMode(gas.PASSIVITY);
    delay(1000);
    gas.setTempCompensation(gas.ON);
    break;
  default:
    status = 99;
    break;
  }
}

float GreenCone_O2::get_O2(){
  switch (name)
  {
  case OXYGEN_SENSOR:
    return oxygen.getOxygenData(COLLECT_NUMBER);
    break;
  case MULTIGAS_SENSOR:
    return gas.readGasConcentrationPPM();
  default:
    status = 99;
    break;
  }
  
}