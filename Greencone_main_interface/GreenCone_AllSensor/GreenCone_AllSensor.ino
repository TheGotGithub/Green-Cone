/* Pin
  I2C---
    SCL : 22
    SDA : 21
*/

/*variable details---------
  //Temp_CO2_Him_Air
    float tempAir_1 : temperature value Air (SCD41)
    uint16 CO2Air_1 : CO2 value Air (SCD41)
    float humidityAir_1 : humidity Value Air (SCD41)

    float tempAir_2 : temperature value Air (SCD41)
    uint16_t CO2Air_2 : CO2 value Air (SCD41)
    float humidityAir_2 : humidity Value Air (SCD41)

  //O2_Sensor
    float oxygenVal

variable details End--------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DFRobot_OxygenSensor.h"

#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "TCA9548.h"

#include <DFRobot_B_LUX_V30B.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//O2_Sensor
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10 

/* USER CODE END PD */

/* USER CODE BEGIN PV */
//O2_Sensor
DFRobot_OxygenSensor oxygen;
float oxygenVal = 0;

//I2C_Mul
TCA9548 MP(0x70);
uint8_t searchAddress = 0x62;
int sensorIndex=2;

//Temp_Air_CO2
SensirionI2CScd4x scd4x;

float tempAir_1 = 0.0f;
uint16_t CO2Air_1 = 0;
float humidityAir_1 = 0;

float tempAir_2 = 0.0f;
uint16_t CO2Air_2 = 0;
float humidityAir_2 = 0;

//LUX_Sensor
DFRobot_B_LUX_V30B  myLux(13);
float luxVal = 0;
// luxVal

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void getTempAir_CO2(uint8_t index,float* tempVal,uint16_t* co2Val,float* humidityVal);
void printTempAir_CO2_Sensor();
void printO2_Sensor();
/* USER CODE END PFP */

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
  Wire.begin();
  /* USER CODE*/
  //--O2_Sensor CODE-------------------------
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
  //--O2_Sensor CODE END----------------------
  //--TEMPAIR_CO2 CODE
  uint16_t error;
  char errorMessage[256];
  scd4x.begin(Wire);
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }
  if (MP.begin() == false)
  {
    Serial.println("Could not connect to TCA9548 multiplexer.");
  }
  error = scd4x.startPeriodicMeasurement();

  //LUX_Sensor
  myLux.begin();

  /* USER CODE END*/
}

void loop() {
  /* USER CODE*/
  //--O2_Sensor CODE-------------------------
  // getO2_Sensor(&oxygenVal);
  printO2_Sensor();
  //--O2_Sensor CODE END ---------------------
  
  //--Temp_Air CODE-------
  // getTempAir_CO2(2, &tempAir_1, &CO2Air_1, &humidityAir_1);
  printTempAir_CO2_Sensor();
  //--Temp_Air CODE END --

  //LUX_Sensor CODE -----
  getLUX_Sensor(&luxVal);
  Serial.println(luxVal);
  //LUX_Sensor CODE END ----

  /* USER CODE END*/
}

//-USER CODE FUNCTION-------------

//TempAir_CO2---------
void getTempAir_CO2(uint8_t index,float* tempVal,uint16_t* co2Val,float* humidityVal){
  MP.selectChannel(index);
  // MP.disableAllChannels();
  uint16_t error;
  char errorMessage[256];
  delay(100);
  // Read Measurement
  uint16_t co2 = 0;
  float temperature = 0.0f;
  float humidity = 0.0f;
  bool isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
      Serial.print("Error trying to execute getDataReadyFlag(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      return;
  }
  if (!isDataReady) {
      return;
  }
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else if (co2 == 0) {
      Serial.println("Invalid sample detected, skipping.");
  } else {
      *tempVal = temperature;
      *co2Val = co2;
      *humidityVal = humidity;
  }
}
void printTempAir_CO2_Sensor(){
  getTempAir_CO2(2, &tempAir_1, &CO2Air_1, &humidityAir_1);
  Serial.print(1);
  Serial.print("TempAir : ");
  Serial.println(tempAir_1);

  getTempAir_CO2(3, &tempAir_2, &CO2Air_2, &humidityAir_2);
  Serial.print(2);
  Serial.print("TempAir : ");
  Serial.println(tempAir_2);
}
//O2_Sensor_CODE
void printO2_Sensor(){
  getO2_Sensor(&oxygenVal);
  // float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  Serial.print(" oxygen concentration is ");
  Serial.print(oxygenVal);
  Serial.println(" %vol");
  delay(1000);
}
void getO2_Sensor(float *O2Val){
  oxygen.begin(Oxygen_IICAddress);
  *O2Val = oxygen.getOxygenData(COLLECT_NUMBER);
}

void getLUX_Sensor(float *luxVal){
  myLux.begin();
  *luxVal = myLux.lightStrengthLux();
}
