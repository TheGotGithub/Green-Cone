# GreenCone Multi Sensor

## Pin

- I2C
    - SCL : 22
    - SDA : 21

---

## Variable details

| Sensor Name | Variable name | type |
| --- | --- | --- |
| SCD41(TempAir,CO2,Hum) | tempAir_1 | float |
|  | humidityAir_1 | float |
|  | CO2Air_1 | uint16_t |
|  | tempAir_2 | float |
|  | humidityAir_2 | float |
|  | CO2Air_2 | uint16_t |
| O2 Sensor DFRobot | oxygenVal | float |
| LUX_V30B DFRobot | luxVal | float |

---

## Function

| Sensor Name | Function  |
| --- | --- |
| SCD41(TempAir,CO2,Hum) | getTempAir_CO2() |
|  | printTempAir_CO2_Sensor() |
| O2 Sensor DFRobot | getO2_Sensor() |
|  | printO2_Sensor() |
| LUX_V30B DFRobot | getLUX_Sensor() |
|  |  |

## Get Data Function

- SCD41(TempAir,CO2,Hum)
    - getTempAir_CO2()
    
    ```cpp
    getTempAir_CO2(2, &tempAir_1, &CO2Air_1, &humidityAir_1);
    Serial.println(tempAir_1);
    Serial.println(CO2Air_1);
    Serial.println(humidityAir_1);
    ```
    
- O2 Sensor DFRobot
    - getO2_Sensor()
    
    ```cpp
    getO2_Sensor(&oxygenVal);
    Serial.print(oxygenVal);
    ```
    
- LUX_V30B DFRobot
    - getLUX_Sensor()
    
    ```cpp
     getLUX_Sensor(&luxVal);
     Serial.println(luxVal);
    ```
