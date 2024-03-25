#include "DFRobot_MultiGasSensor.h"
#include "DFRobot_OxygenSensor.h"

#define I2C_COMMUNICATION

#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;

// #ifdef  I2C_COMMUNICATION
// #define I2C_ADDRESS    0x74
//   DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
// #endif

class GreenCone_O2
{
private:
    /* data */
public:

    int I2C_ADDRESS = 0x74

    GreenCone_O2(int I2C_ADDRESS);
    ~GreenCone_O2();
};


