#include "DFRobot_MultiGasSensor.h"
#include "DFRobot_OxygenSensor.h"
//Turn on by default, using I2C communication at the time, switch to serial port communication after turning off
#define I2C_COMMUNICATION

#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x74
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#endif


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);

  while(!gas.begin())
  {
    Serial.println("O2[1] NO Deivces !");
    delay(1000);
  }
  Serial.println("O2[1] connected successfully!");

  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("O2[2] NO Deivces !");
    delay(1000);
  }
  Serial.println("O2[2] connected successfully!");

  //Mode of obtaining data: the main controller needs to request the sensor for data
  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);
  gas.setTempCompensation(gas.ON);

}

void loop() {
  Serial.print("O2[1] : ");
  Serial.print(gas.readGasConcentrationPPM());
  
  Serial.print(" O2[2] : ");
  Serial.println(oxygen.getOxygenData(COLLECT_NUMBER));

}
