#ifndef __GreenCone_O2_H__
#define __GreenCone_O2_H__

#include "Wire.h"  // Include Wire library for I2C communication
#include "DFRobot_MultiGasSensor.h"
#include "DFRobot_OxygenSensor.h"

#define OXYGEN_SENSOR 1
#define OXYGEN_SENSOR_Address ADDRESS_3
#define COLLECT_NUMBER  10 

#define MULTIGAS_SENSOR 2
#define I2C_COMMUNICATION
#define I2C_ADDRESS    0X77

class GreenCone_O2
{
  public:
    uint8_t name = 0;
    uint8_t status = 0;
    DFRobot_OxygenSensor oxygen;
    DFRobot_GAS_I2C gas;

    GreenCone_O2(uint8_t sensorName);
    void Init();
    float get_O2();
    
  private:
};

#endif
