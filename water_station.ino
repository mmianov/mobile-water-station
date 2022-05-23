//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 8);  // CE, CSN  - create an RF24 object
const uint64_t esp_addr = 0xE8E8F0F0E1LL; //address to communicate with esp32
char ackData[2] = {100,200};
int xValue;
int yValue;
void setup()
{
 
  Serial.begin(9600);
  
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
   ackData[0] = 1111;
   ackData[1] = 4444;
   radio.writeAckPayload(1,&ackData,sizeof(ackData));
  
  }
