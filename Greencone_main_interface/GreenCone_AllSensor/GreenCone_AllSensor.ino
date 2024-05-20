#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "PlanetCentric";
const char* password = "BearLab!";

unsigned long timerDelay = 10000;

float Moi_1_raw, Moi_1_ADC, Moi_2_raw, Moi_2_ADC, pH, O2_1, O2_2, Temp_Contact_1;

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
#include "DFRobot_MultiGasSensor.h"

#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <TCA9548.h>
#include <SoftwareSerial.h>

#include <DFRobot_B_LUX_V30B.h>
#include <SHT31.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//O2_Sensor
#define Oxygen_IICAddress_1 0x73
#define Oxygen_IICAddress_2 0x74
#define COLLECT_NUMBER  10 

//O2_Sensor (MultiGas)
#define I2C_ADDRESS 0x74


//Methene 
#define Methene 34

//PH
#define RE 2
#define DE 0

//Soil Moisture
#define OUT_SoilMois1 33
#define OUT_SoilMois2 32


/* USER CODE END PD */

/* USER CODE BEGIN PV */
uint32_t lastTime = 0;
//O2_Sensor
DFRobot_OxygenSensor oxygen;
float oxygenVal1 = 0.0f;
float oxygenVal2 = 0.0f;

//O2_Sensor (MultiGas)
DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);

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
float luxVal = 0.0f;
// luxVal

//Methene
float MetheneVal = 0.0f;
float Methene_ADC;

//PH
uint16_t PHVal= 0;
const byte phaddress[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
byte values[11];
SoftwareSerial mod(15, 4);

//Soil Moisture
float Moisture1_ADC;
float Moisture2_ADC;
float Moisture1_Val;
float Moisture2_Val;

//SHT30
SHT31 sht;
float tempContact;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void getTempAir_CO2(uint8_t index,float* tempVal,uint16_t* co2Val,float* humidityVal);
void printTempAir_CO2_Sensor();
void printO2_Sensor();
/* USER CODE END PFP */

void setup() {

  Serial.begin(9600);

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
  while (!Serial) {
    // delay(100);
  }
  Wire.begin();
  Wire.setClock(100000);

  /* USER CODE*/
  //--O2_Sensor CODE-------------------------
  while(!oxygen.begin(Oxygen_IICAddress_1)){
    Serial.println("I2c for 1st O2 device number error !");
  }
  Serial.println("I2c for 1st O2 connect success !");
  
  while(!oxygen.begin(Oxygen_IICAddress_2)){
    Serial.println("I2c for 2nd O2 device number error !");
  }
  Serial.println("I2c for 2nd O2 connect success !");

  while(!gas.begin())
  {
    Serial.println("O2_Sensor (MultiGass)NO Deivces !");
  }
  gas.changeAcquireMode(gas.PASSIVITY);
  gas.setTempCompensation(gas.ON);
  //--O2_Sensor CODE END----------------------

  //--TEMP_AIR_CO2 CODE
  if (MP.begin() == false)
  {
    Serial.println("Could not connect to TCA9548 multiplexer.");
  }

  uint16_t error;
  char errorMessage[256];
  MP.selectChannel(6);
  scd4x.begin(Wire);

  error = scd4x.startPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute startPeriodicMeasurement_CO2_1(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  Serial.println("Waiting for first measurement... (5 sec)");
  delay(5000);

  MP.selectChannel(7);
  scd4x.begin(Wire);

  error = scd4x.startPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute startPeriodicMeasurement_CO2_2(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }
  Serial.println("Waiting for first measurement... (5 sec)");
  //--TEMP_AIR_CO2 CODE END------------

  //LUX_Sensor
  myLux.begin();

  //PH Sensor
  mod.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  //SHT30
  sht.begin(0x44); 

  /* USER CODE END*/
}

void loop() {
    /* USER CODE*/

    //--O2_Sensor CODE-------------------------
    getO2_Sensor(&oxygenVal1, &oxygenVal2);
    
    //--O2_Sensor CODE END ---------------------
    
    //--Temp_Air CODE-------
    getTempAir_CO2(6, &tempAir_1, &CO2Air_1, &humidityAir_1);
    getTempAir_CO2(7, &tempAir_2, &CO2Air_2, &humidityAir_2);
    //--Temp_Air CODE END --

    //LUX_Sensor CODE -----
    getLUX_Sensor(&luxVal);
    //LUX_Sensor CODE END ----

    //Methene Sensor
    GetMethene(&MetheneVal);

    //PH Sensor
    GetPH(&PHVal);

    //Soil Moisture
    GetSoilMoisture(&Moisture1_Val, &Moisture2_Val);

    //SHT30
    GetSHT30(&tempContact);


    /* USER CODE END*/
    if ((millis() - lastTime) > timerDelay){
      lastTime = millis();
      Serial.println("-----------------------------");

      Serial.print("1st_oxygen: ");
      Serial.println(oxygenVal1);
      Serial.print("2nd oxygen: ");
      Serial.print(oxygenVal2);
      Serial.println(" %vol");

      Serial.print(1);
      Serial.print("TempAir : ");
      Serial.println(tempAir_1);

      Serial.print(2);
      Serial.print("TempAir : ");
      Serial.println(tempAir_2);

      Serial.print("LUX : ");
      Serial.println(luxVal);

      Serial.print("Methene: ");
      Serial.println(MetheneVal);

      Serial.print("Methene ADC: ");
      Serial.println(Methene_ADC);

      Serial.print("PH : ");
      Serial.println(PHVal);

      Serial.print("Moisture 1 & 2 : ");
      Serial.print(Moisture1_Val);
      Serial.print(", ");
      Serial.println(Moisture2_Val);

      Serial.print("MoistureADC 1 & 2 : ");
      Serial.print(Moisture1_ADC);
      Serial.print(", ");
      Serial.println(Moisture2_ADC);

      Serial.print("Temp_contact : ");
      Serial.println(tempContact);

      if(WiFi.status()== WL_CONNECTED){
        Send_data("https://books-opening.gl.at.ply.gg:61345/api/v1/greenCone/add");
      }

      else {
        Serial.println("WiFi Disconnected");
      }

    }
}

//-USER CODE FUNCTION-------------

//TempAir_CO2---------
void getTempAir_CO2(uint8_t index,float* tempVal,uint16_t* co2Val,float* humidityVal){
  MP.selectChannel(index);
  // scd4x.begin(Wire);
  // MP.disableAllChannels();
  uint16_t error;
  char errorMessage[256];
  // delay(100);
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
  // Serial.println("before read Measure");
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
  getTempAir_CO2(6, &tempAir_1, &CO2Air_1, &humidityAir_1);
  // Serial.print(1);
  // Serial.print("TempAir : ");
  // Serial.println(tempAir_1);

  // getTempAir_CO2(7, &tempAir_2, &CO2Air_2, &humidityAir_2);
  // Serial.print(2);
  // Serial.print("TempAir : ");
  // Serial.println(tempAir_2);
}

//O2_Sensor_CODE
void printO2_Sensor(){
  getO2_Sensor(&oxygenVal1, &oxygenVal2);
  // float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  // Serial.print("1st_oxygen: ");
  // Serial.print(oxygenVal1);
  // Serial.print("2nd oxygen: ");
  // Serial.print(oxygenVal2);
  // Serial.println(" %vol");
}
void getO2_Sensor(float *O2Val_1, float *O2Val_2){
  oxygen.begin(Oxygen_IICAddress_1);
  *O2Val_1 = oxygen.getOxygenData(COLLECT_NUMBER);

  oxygen.begin(Oxygen_IICAddress_2);
  *O2Val_2 = gas.readGasConcentrationPPM();
}
void getLUX_Sensor(float *luxVal){
  myLux.begin();
  *luxVal = myLux.lightStrengthLux();
}

//Methene Sensor
void GetMethene(float *MetheneVal){
  float slope = -0.3384580239;
  float A = 10.36034152;
  float Rseries = 1000;
  Methene_ADC = (float)analogRead(Methene);
  float ADC = ((float)analogRead(Methene)*5.0)/1023;
  float Rs = ((5-ADC)/ADC)*Rseries; 
  float R0 = 1809.52;
  float Y = Rs/R0;
  float Methane_gas = pow(10,(log10(Y/A)/slope));
  *MetheneVal = Methane_gas;
}

//PH Sensor
void GetPH(uint16_t *PHVal){
  *PHVal = Fphaddress() / 10;
}

byte Fphaddress() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (mod.write(phaddress, sizeof(phaddress)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    for (byte i = 0; i < 7; i++) {
      values[i] = mod.read();
      }
    }
  return values[4];
}

//Soil Moistures
void GetSoilMoisture(float *Moisture1_Val, float *Moisture2_Val){
  uint16_t wetValue = 2700;
  uint16_t dryValue = 1500;
  *Moisture1_Val = constrain(map(analogRead(OUT_SoilMois1), wetValue, dryValue, 100, 0), 0, 100);
  *Moisture2_Val = constrain(map(analogRead(OUT_SoilMois2), wetValue, dryValue, 100, 0), 0, 100);

  Moisture1_ADC = analogRead(OUT_SoilMois1);
  Moisture2_ADC = analogRead(OUT_SoilMois2);
}

//SHT30

void GetSHT30(float *tempContact){
  sht.begin(0x44); 
  sht.read();
  *tempContact = sht.getTemperature();
}

void Send_data(const char* serverName){
    WiFiClient client;
    HTTPClient http;
  
    // Your Domain name with URL path or IP address with path
    http.begin(serverName);
    
    // // Specify content-type header
    http.addHeader("Content-Type", "application/json");

    String jsonData = "";

    jsonData += "{";

    jsonData += "\"methane\" : {";
    jsonData += "\"data_0\" : \""  + String(Methene_ADC) +   "\", ";
    jsonData += "\"data_1\" : \""  + String(MetheneVal) +   "\"";
    jsonData += "},";

    jsonData += "\"moisture_0\" : {";
    jsonData += "\"data_0\" : \""  + String(Moisture1_ADC) +   "\", ";
    jsonData += "\"data_1\" : \""  + String(Moisture1_Val) +   "\"";
    jsonData += "},"; 

    jsonData += "\"moisture_1\" : {";
    jsonData += "\"data_0\" : \""  + String(Moisture2_ADC) +   "\", ";
    jsonData += "\"data_1\" : \""  + String(Moisture2_Val) +   "\"";
    jsonData += "},";

    jsonData += "\"ph\" : {";
    jsonData += "\"data_0\" : \""  + String(PHVal) +   "\" ";
    jsonData += "},";

    jsonData += "\"o2_0\" : {";
    jsonData += "\"data_0\" : \""  + String(oxygenVal1) +   "\" ";
    jsonData += "},";

    jsonData += "\"o2_1\" : {";
    jsonData += "\"data_0\" : \""  + String(oxygenVal2) +   "\" ";
    jsonData += "},";  

    jsonData += "\"ambientLight\" : {";
    jsonData += "\"data_0\" : \""  + String(luxVal) +   "\" ";
    jsonData += "},";

    jsonData += "\"co2_0\" : {";
    jsonData += "\"data_0\" : \""  + String(CO2Air_1) +   "\" ";
    jsonData += "},";

    jsonData += "\"temp_noContact_0\" : {";
    jsonData += "\"data_0\" : \""  + String(tempAir_1) +   "\" ";
    jsonData += "},";  

    jsonData += "\"co2_1\" : {";
    jsonData += "\"data_0\" : \""  + String(CO2Air_2) +   "\" ";
    jsonData += "},";

    jsonData += "\"temp_noContact_1\" : {";
    jsonData += "\"data_0\" : \""  + String(tempAir_2) +   "\" ";
    jsonData += "},";

    jsonData += "\"temp_contact\" : {";
    jsonData += "\"data_0\" : \""  + String(tempContact) +   "\" ";
    jsonData += "}";

    jsonData += "}";

    int httpResponseCode = http.POST(jsonData);
    Serial.print("httpSendStatus: ");
    Serial.println(httpResponseCode);
    http.end();
}