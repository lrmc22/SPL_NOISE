#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <math.h>

 //generated pb interface
#include <pb_encode.h>
#include "interface.pb.h"

//Router parameters
const char* ssid        = "......."; // Enter your WiFi name
const char* password    =  "......"; // Enter WiFi password
const char* mqttServer  = "192.168.1.78"; //RPI MQTT Server IP

//PB variables
bool status;

// MQTT Topic
const char* temperatureTopic = "/senso-care/sensors/SPLdB-superesp8266";

//Our two communicating objects in MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//VPP variables 
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz) 
unsigned int sample;
 
 //Sent parameters
 void sendSerialised(float value, const char* topic)
{
  uint8_t buffer[sensocare_messages_Measure_size];
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  //Set protobuf structure values
  measure.timestamp = (uint64_t) time(NULL);
  measure.value = value;

//Encoding Measure fields
  status = pb_encode(&stream, sensocare_messages_Measure_fields, &measure);

  if (!status)
  {
    Serial.println("Encoding failed"); // Fail
  }
  /*
  //Prinf buffer in serial monitor
  for(int i = 0; i < sensocare_messages_Measure_size; i++ )
  {
    Serial.print(buffer[i]);
  }
  Serial.print(" ");
  // Envoyer le buffer
  client.publish(topic, (char*)buffer);
  */
}
void setup() 
{
   Serial.begin(115200);
}
void wifiSetup()
{
  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //Init wifi connection to router Bbox in our case
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

//We wait until connection is done
while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  Serial.print(".");
}

  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  configTime(2*3600, 0, mqttServer);
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected())
  {

    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
     {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    }else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() 
{
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level
 
   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;
 
   // collect data for a 50 mS period --> 20Hz being the minimum audible frequency 
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A0);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = 0.0;
   volts = map(volts,0,2*peakToPeak,0,1024);
   //double volts = (peakToPeak * 3.3) / 1024;  // convert to volts here we use a 3.3v power supply
   //94 db --> une pression acoustique efficace de 1 pA equivaut à 94dB SPL (Sound Pressure Level)
   int dBVal = 94 + 20*log10(volts/1.58); 
   
   Serial.print("voltage : "); 
   Serial.println(volts);
   Serial.print("SPL : "); 
   Serial.println(dBVal);
   sendSerialised(volts,temperatureTopic);
   delay(500);
}