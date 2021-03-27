#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <math.h>

//generated pb interface
#include <pb_encode.h>
#include "interface.pb.h"

//Router parameters
const char *ssid        = "";                      // Enter your WiFi name
const char *password    = ""; // Enter WiFi password
const char *mqttServer  = "";                 //RPI MQTT Server IP

//PB variables
bool status;

// MQTT Topic
const char *splTopic = "/senso-care/sensors/SPLdB-superesp8266";

//Our two communicating objects in MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//VPP variables
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

//PB sent buffer
uint8_t buffer[sensocare_messages_Measure_size];

long startMillis = millis();

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

  //configTime(2 * 3600, 0, mqttServer);
}
void reconnect()
{
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
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void sendSerialised(sensocare_messages_Measure measure, const char *topic)
{
  memset(buffer, '\0', sensocare_messages_Measure_size);
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sensocare_messages_Measure_size);
  status = pb_encode(&stream, sensocare_messages_Measure_fields, &measure);
  if (!status)
  {
    Serial.println("Encoding failed"); // Fail
  }

  //Prinf buffer in serial monitor
  /*for(int i = 0; i < sensocare_messages_Measure_size; i++ )
  {
    Serial.print(buffer[i]);
  }
  Serial.println(" ");*/
  // Envoyer le buffer
  client.publish(topic, (char *)buffer, stream.bytes_written);
}

void createMessageAndSend(float value, const char *topic)
{
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  measure.value.fValue = value;
  measure.which_value = sensocare_messages_Measure_fValue_tag;
  sendSerialised(measure, topic);
}
void createMessageAndSend(int value, const char *topic)
{
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  measure.value.iValue = value;
  measure.which_value = sensocare_messages_Measure_iValue_tag;
  sendSerialised(measure, topic);
}
void setup()
{
  Serial.begin(115200); 
  wifiSetup();
  //link between client and MQTT server
  client.setServer(mqttServer,1883);
}
void loop()
{
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;          // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  if (!client.connected()) {
    reconnect();
  }
  // collect data for a 50 mS period --> 20Hz being the minimum audible frequency
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(A0);
    if (sample < 1024) // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample; // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample; // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin; // max - min = peak-peak amplitude
  double volts = 0.0;
  volts = (peakToPeak * 3.3 / 1024) * 10; // convert to volts here we use a 3.3v power supply according to gain
                                          //94 db --> une pression acoustique efficace de 1 pA equivaut à 94dB SPL(Sound Pressure Level)
                                          //-44 dB sensibilité du micro , 60 est le gain de notre capteur
  double dBVal = 20 * log10(volts / 0.00631);
  float  dBSPL = dBVal + 94 - 44 - 60;

  Serial.print("dbv : ");
  Serial.println(volts);
  Serial.print("SPL : ");
  Serial.println(dBSPL);

  //Send serialised data to Mqtt server 
  createMessageAndSend(dBSPL, splTopic);

  delay(1000);
}