//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "GravityTDS.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define TdsSensorPin A3
#define TurbiditySensorPin A5
#define OneWireBus 7

GravityTDS gravityTds;
OneWire oneWire(OneWireBus);
DallasTemperature sensors(&oneWire);


RF24 radio(9, 8);  // CE, CSN  - create an RF24 object
const uint64_t esp_addr = 0xE8E8F0F0E1LL; //address to communicate with esp32
float ackData[3] = {0,0,0};
int xValue;
int yValue;

float roundToDp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}

float getAvgVoltage(char sensorPin) {
  float voltage=0;
  float inputVoltage = 5.0;
  int samples=1000;

  for (int i=0; i<samples; i++) {
    voltage += ((float)analogRead(sensorPin)/1024.0)*inputVoltage;
  }

  voltage=voltage/samples;
  voltage=roundToDp(voltage, 1);
  return voltage;
}

float getTemperatureWrapper() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float getTurbidityValue(float voltage_TURBIDITY) {
  float ntu;
  if(voltage_TURBIDITY < 2.5){
      ntu = 3000;
  }
  else{
      ntu = -1120.4*square(voltage_TURBIDITY)+5742.3*voltage_TURBIDITY-4352.9;
    }
  return ntu;
}

float getTdsValueWrapper(float temperature) {

  float tdsValue = 0;
  gravityTds.setTemperature(temperature);
  gravityTds.update(); //sampling & calculating
  tdsValue = gravityTds.getTdsValue();
  return tdsValue;
}

void setup()
{
 
  Serial.begin(9600);

  // for DS18B20 - temperature sensor
  sensors.begin();
  // for SEN0244 - TDS sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, esp_addr);
  radio.enableAckPayload();
  radio.startListening();
  radio.writeAckPayload(1,&ackData,sizeof(ackData));
}

void loop()
{

  if (radio.available())
  {
 
    int dataReceived[4]; 
    radio.read(&dataReceived, sizeof(dataReceived));
    xValue = dataReceived[0];
    yValue = dataReceived[2];
    
    
    Serial.print("Received xValue: ");
    Serial.print(xValue);
    Serial.print(" Received yValue: ");
    Serial.println(yValue);
    // engine output
    // get measurements
    sendMeasurements();
    delay(50);

  }

}

void sendMeasurements(){
   float measurement_TURBIDITY = getTurbidityValue(voltage_TURBIDITY);
   delay(1000);
   float measurement_TEMPERATURE = getTemperatureWrapper();
   delay(1000);
   float measurement_TDS = getTdsValueWrapper(measurement_TEMPERATURE);

   ackData[0] = measurement_TURBIDITY;
   ackData[1] = measurement_TEMPERATURE;
   ackData[2] = measurement_TDS;

   radio.writeAckPayload(1,&ackData,sizeof(ackData));
  
  }
