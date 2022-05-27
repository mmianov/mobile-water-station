//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "GravityTDS.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define TdsSensorPin A4
#define TurbiditySensorPin A3
#define OneWireBus 4
#define enA 3
#define in1 6
#define in2 5 
#define enB 10
#define in3 7
#define in4 2 



GravityTDS gravityTds;
OneWire oneWire(OneWireBus);
DallasTemperature sensors(&oneWire);


RF24 radio(9, 8);  // CE, CSN  - create an RF24 object
const uint64_t esp_addr = 0xE8E8F0F0E1LL; //address to communicate with esp32

int xValue;
int yValue;

float ackData[3] = {0,0,0};
int measurementSendDelay = 3000;
unsigned long timer;
int state;


void setup()
{
  Serial.begin(9600);
  Serial.println("Mobile Water Station Controller - initializing ... ");

  // motors
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  // for DS18B20 - temperature sensor
  sensors.begin();
  // for SEN0244 - TDS sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();
   Serial.println("[*]Water sensors: up and running");

  radio.begin();
  Serial.println("[*]nRF24L01: up and running");
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, esp_addr);
  radio.enableAckPayload();
  Serial.println("[*]ACK Payload: enabled");
  radio.startListening();
  radio.writeAckPayload(1,&ackData,sizeof(ackData));

  timer = millis();
}

void loop()
{

  if (radio.available())
  {
 
    int dataReceived[4]; 
    radio.read(&dataReceived, sizeof(dataReceived));
    xValue = dataReceived[0];
    yValue = dataReceived[2];
    
//    Serial.print("Received xValue: ");
//    Serial.print(xValue);
//    Serial.print(" Received yValue: ");
//    Serial.println(yValue);
    // engine output
    state = controlMotors(xValue,yValue);
    
    // get measurements after specified time and if the boat does not move
    if(millis()-timer >= measurementSendDelay && state == 0){
        sendMeasurements();
        timer = millis();  
      }
  }

}

// Engine functions - controlling the boat 

// xValue - left right movement
// yValue - forward backward movement
int controlMotors(int xValue, int yValue){
    int speedMotorLeft; // A - right motor
    int speedMotorRight;// B - left motor

    // MOTOR LOGIC
    // left and right are from the perspective of the back of the boat
    // stationary position (joystick in the center)
    if(xValue <= 1850 && xValue >= 1650 && yValue <= 1850 && yValue >= 1650){
        // left motor stationary
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        // right motor stationary
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);;
        speedMotorLeft = 0;
        speedMotorRight = 0;  
        //Serial.println("In stationary");  
        return 0; 
      }

      // forward
      else if(xValue <= 1850 && xValue >= 1650 && yValue <= 4095 && yValue >= 1850){
        // left motor: power
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        // right motor: power
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);;
        speedMotorLeft = map(yValue,1850,4095,0,255);
        speedMotorRight = map(yValue,1850,4095,0,255);  
        Serial.println("Forward");   
        return 1;
      }

      // backward
      else if(xValue <= 1850 && xValue >= 1650 && yValue <= 1650 && yValue >= 0){
        // left motor: power
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        // right motor: power 
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);;
        speedMotorLeft = map(yValue,1650,0,0,255);
        speedMotorRight = map(yValue,1650,0,0,255);  
        Serial.println("Backward"); 
        return 2;  
      }

      // spin clockwise
       else if(xValue <= 4095 && xValue >= 1850 && yValue <= 1850 && yValue >= 1650){
        // left motor: power
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        // right motor: stationary
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);;
        speedMotorLeft = map(xValue,1850,4095,0,255);
        speedMotorRight = 0 ;
        Serial.println("Clockwise spin");   
        return 3;
      }

      // spin anticlockwise
       else if(xValue <= 1650 && xValue >= 0 && yValue <= 1850 && yValue >= 1650){
        // left motor: stationary
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        // right motor: power
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);;
        speedMotorLeft = 0;
        speedMotorRight = map(xValue,1650,0,0,255);  
        Serial.println("Anticlockwise spin");   
        return 4;
      }

      // 1 quadrant - moving forward left
      else if (xValue <= 1650 && xValue >= 0 && yValue <= 4095 && yValue >= 1850){
        // left motor: less xValue -> less power -> initiates left turn
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        speedMotorLeft = map(xValue,0,1650,0,255);
        // right motor: more yValue -> more power -> controls angle of the left turn
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        speedMotorRight =  map(yValue,1850,4095,0,255);
        Serial.println("Forward (left)");
        return 5;
        }

      // 2 quadrant - moving forward right
      else if (xValue <= 4095 && xValue >= 1850 && yValue <= 4096 && yValue >= 1850){
        // left motor: more yValue -> more power -> controls angle of the right turn
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        speedMotorLeft = map(yValue,1850,4095,0,255);
        // right motor: more xValue -> less power -> initiates right turn
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        speedMotorRight =  map(xValue,4095,1850,0,255);
        Serial.println("Forward (right)");
        return 6;
        }
      // 3 quadrant - moving backwards left
      else if (xValue <= 1650 && xValue >= 0 && yValue <= 1650 && yValue >= 0){
        // left motor: less xValue -> less power -> initiates left turn
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speedMotorLeft = map(xValue,0,1650,0,255);
        // right motor: less yValue -> more power -> controls angle of the left turn
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        speedMotorRight =  map(yValue,1650,0,0,255);
        Serial.println("Backwards (left)");
        return 7;
        }

        // 4 quadrant - moving backwards right
      else if (xValue <= 4095 && xValue >= 1850 && yValue <= 1650 && yValue >= 0){
        // left motor: more yValue -> more pwoer -> controls angle of the rigth turn
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speedMotorLeft = map(yValue,1650,0,0,255);
        // right motor: more xValue -> less power -> initiaties right turn
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        speedMotorRight =  map(xValue,4095,1850,0,255);
        Serial.println("Backwards (right)");
        return 8;
        }

        analogWrite(enA, speedMotorRight);
        analogWrite(enB, speedMotorLeft);
      
  } 


// Measurements Functions
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

void sendMeasurements(){
   float voltage_TURBIDITY = getAvgVoltage(TurbiditySensorPin);
   float measurement_TURBIDITY = getTurbidityValue(voltage_TURBIDITY);
   float measurement_TEMPERATURE = getTemperatureWrapper();
   float measurement_TDS = getTdsValueWrapper(measurement_TEMPERATURE);
   Serial.print("Turbidity: ");
   Serial.print(measurement_TURBIDITY);
   Serial.print(" Temperature: ");
   Serial.print(measurement_TEMPERATURE);
   Serial.print(" TDS: ");
   Serial.println(measurement_TDS);
   
   ackData[0] = measurement_TURBIDITY;
   ackData[1] = measurement_TEMPERATURE;
   ackData[2] = measurement_TDS;

   radio.writeAckPayload(1,&ackData,sizeof(ackData));
  
  }