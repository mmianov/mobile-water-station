#include <HttpClient.h>
#include <WiFi.h>
#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define joyX 32 
#define joyY 34

RF24 radio(12, 14, 26, 25, 27); //create an RF24 object
const uint64_t boat_addr = 0xE8E8F0F0E1LL; //address to commmunicate with Arduino nrf24
float ackData[3] = {0,0,0};
  
const char* ssid = "test";
const char* password =  "testpasswd";
  
void setup() {
  Serial.begin(9600);
  Serial.println("ESP32 Controller up and running");
 
  radio.begin(); 
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  Serial.println("ACK Payload: enabled");
  radio.setRetries(5,5);
  radio.openWritingPipe(boat_addr); //set the address
  Serial.print("Opened writing pipe to address: ");
  Serial.println((int)boat_addr);
  
}
  
void loop() {
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);
 
  int joystickValues[2];
  joystickValues[0] = xValue;
  joystickValues[1] = yValue;
  
  bool result;
  result = radio.write(&joystickValues, sizeof(joystickValues));
  // debug
//  Serial.print("X sensor: ");
//  Serial.print(joystickValues[0]);
//  Serial.print(" Y sensor: ");
//  Serial.println(joystickValues[1]);
//  Serial.println(sizeof(joystickValues));
    
   if(result){
      if(radio.isAckPayloadAvailable()){
          radio.read(&ackData,sizeof(ackData));
          Serial.print(ackData[0]);
          Serial.print(" --- ");
          Serial.print(ackData[1]);
          Serial.print(" --- ");
          Serial.println(ackData[2]);
        }
//        else{
//          Serial.println("ACK but no data");
//          }
    }

//    else{
//        Serial.println("TX failed");
//      }

    delay(80);
  
}





//void setup_WiFi(){
//  
//  
//  delay(4000);   //Delay needed before calling the WiFi.begin
//  
//  WiFi.begin(ssid, password); 
//  
//  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
//    delay(1000);
//    Serial.println("Connecting to WiFi..");
//  }
//  
//  Serial.println("Connected to the WiFi network");
//  
//  }



//void loop_testWiFi(){
//  if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status
//  
//   HTTPClient http;   
//  
//   http.begin("https://ptsv2.com/t/w5jk9-1653050943/post");  //Specify destination for HTTP request
//   http.addHeader("Content-Type", "text/plain");             //Specify content-type header
//  
//   int httpResponseCode = http.POST("POSTING from ESP32");   //Send the actual POST request
//  
//   if(httpResponseCode>0){
//  
//    String response = http.getString();                       //Get the response to the request
//  
//    Serial.println(httpResponseCode);   //Print return code
//    Serial.println(response);           //Print request answer
//  
//   }else{
//  
//    Serial.print("Error on sending POST: ");
//    Serial.println(httpResponseCode);
//  
//   }
//  
//   http.end();  //Free resources
//  
//   }else{
//  
//    Serial.println("Error in WiFi connection");   
//  
//  }
//  
//  delay(10000);  //Send a request every 10 seconds
//  }
