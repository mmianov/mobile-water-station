#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "DHT.h"
#define DHTPIN 14     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

float h ;
float t;
const char* ssid = "test";
const char* password =  "testpasswd";

DHT dht(DHTPIN, DHTTYPE);

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void connectAWS()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker on the AWS endpoint we defined earlier
    client.setServer(AWS_IOT_ENDPOINT, 8883);

    // Create a message handler
    client.setCallback(messageHandler);

    Serial.println("Connecting to AWS IOT");

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

    Serial.println("AWS IoT Connected!");
}

void publishMessage()
{
    StaticJsonDocument<200> doc;
    doc["humidity"] = h;    // TODO change this
    doc["temperature"] = t; // TODO change this
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer); // print to client

    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
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
  connectAWS();
  delay(4000);   //Delay needed before calling the WiFi.begin
  dht.begin()
  
}
  
void loop() {
  
// if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status
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
// }else{
//
//    Serial.println("Error in WiFi connection");
//
// }
    h = dht.readHumidity();       // TODO change this
    t = dht.readTemperature();    // TODO change this


    if (isnan(h) || isnan(t) )  // Check if any reads failed and exit early (to try again).
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    Serial.print(F("Humidity: "));          // TODO change this
    Serial.print(h);
    Serial.print(F("%  Temperature: "));    // TODO change this
    Serial.print(t);
    Serial.println(F("°C "));

    publishMessage();
    client.loop();
    delay(10000); //Send a request every 10 seconds
  
}
