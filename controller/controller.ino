#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

#define joyX 32 
#define joyY 34

RF24 radio(12, 14, 26, 25, 27); //create an RF24 object
const uint64_t boat_addr = 0xE8E8F0F0E1LL; //address to commmunicate with Arduino nrf24
float ackData[3] = {0,0,0};

float temp ;
float tds;
float turbidity;


WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void connectAWS()
{
    Serial.println("ESP32 Controller up and running");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("[*]Connected to Wifi");

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    Serial.println("[*]Configured AWS security credentials");

    // Connect to the MQTT broker on the AWS endpoint we defined earlier
    client.setServer(AWS_IOT_ENDPOINT, 8883);

    // Create a message handler
    client.setCallback(messageHandler);

    Serial.println("Connecting to AWS IOT ...");

    while (!client.connect(THINGNAME))
    {
        Serial.print(".");
        delay(100);
    }

    if (!client.connected())
    {
        Serial.println("AWS IoT Timeout!");
        return;
    }

    // Subscribe to a topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

    Serial.println("[*]AWS IoT Connected!");
}

void publishMessage(float temp, float tds,float turbidity)
{
    StaticJsonDocument<200> doc;
    doc["Temperature"] = temp;    
    doc["TDS"] = tds;
     doc["Turbidity"] = turbidity;
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer); // print to client

    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
    Serial.println("Measurements sent to AWS server!");
}

void messageHandler(char* topic, byte* payload, unsigned int length)
{
    Serial.print("incoming: ");
    Serial.println(topic);

    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    const char* message = doc["message"];
    Serial.println(message);
}

void setup() {
  
  Serial.begin(115200);
  // AWS IoT server config
  connectAWS();
  delay(4000);   //Delay needed before calling the WiFi.begin
  // nRF24L01 config
  radio.begin(); 
  Serial.println("[*]nRF24L01 radio up and running");
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  Serial.println("[*}ACK Payload enabled");
  radio.setRetries(5,5);
  radio.openWritingPipe(boat_addr);
  
}
  
void loop() {
  
  // read values from joysticks
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);
 
  int joystickValues[2];
  joystickValues[0] = xValue;
  joystickValues[1] = yValue;

  bool result;
  // send joystick commands to controller
  result = radio.write(&joystickValues, sizeof(joystickValues));

  // get measurements if sent in ACK
  if(result){
      if(radio.isAckPayloadAvailable()){
          radio.read(&ackData,sizeof(ackData));
          Serial.print(ackData[0]);
          turbidity = ackData[0];
          Serial.print(" --- ");
          Serial.print(ackData[1]);
          temp = ackData[1];
          Serial.print(" --- ");
          Serial.println(ackData[2]);
          tds = ackData[2];

          // publish measurements to AWS server
          publishMessage(temp,tds,turbidity);
        }

    client.loop();
    
  
  }
}