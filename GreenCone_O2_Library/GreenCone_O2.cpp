#include "GreenCone_O2.h"

GreenCone_O2::GreenCone_O2(int I2C_add)
{
    I2C_ADDRESS = I2C_add;
    DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
}

GreenCone_O2::~GreenCone_O2()
{

}