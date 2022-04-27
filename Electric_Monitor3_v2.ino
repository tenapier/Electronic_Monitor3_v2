
// This code is for the First Set of Solar Panels giving voltage, current and power output.



#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

const char* ssid     = "Tim's Network"; // ESP32 and ESP8266 uses 2.4GHZ wifi only
const char* password = "carlsagan"; 

//MQTT Setup Start
#include <PubSubClient.h>
#define mqtt_server "192.168.7.238"
WiFiClient espClient;
PubSubClient client(espClient);

#define mqttVoltsIn "Volts/Vin"
#define mqttVoltsSol "Volts/Vsol"
#define mqttCurrentOut "Current/current"
#define mqttCurrent2Out "Current/current2"
#define mqttPower "Power/power"
#define mqttPower2 "Power/power2"
#define mqttRelay "Solar/Relay"
#define mqttRelay2 "Solar/Relay2"

unsigned long R1 = 6800;  //Top Resistor for Battery Voltage Divider
unsigned long R2 = 1000;    //Bottom Resistor for Battery Voltage Divider
unsigned long R3 = 120000;  // Top Resistor for Solar Voltage Divider
unsigned long R4 = 3300;   // Bottom Resistor for Solar Voltage Divider

float Vin;
float Vsol;
float current;
float current2;
float power;
float power2;

unsigned long millisNow = 0;  // for delay purposes
unsigned int sendDelay = 10000;  // delay before sending info via MQTT

const int Relay = 16;
const int Relay2 = 17;

void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  String messageTemp2;
  for (int i = 0; i < length; i++){
  Serial.print((char)message[i]);
  messageTemp += (char)message[i];
}
Serial.println();

if(String(topic)== mqttRelay){
  Serial.print("Changing Solar Relay to ");
  if(messageTemp == "on"){
    digitalWrite(Relay, HIGH);
    Serial.print("On");
  }
  else if(messageTemp == "off"){
    digitalWrite(Relay, LOW);
    Serial.print("Off");
  }
}
Serial.println();

 Serial.print("Message2 arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message2: ");
  for (int i = 0; i < length; i++){
  Serial.print((char)message[i]);
  messageTemp2 += (char)message[i];
}

Serial.println();
if(String(topic)== mqttRelay2){
  Serial.print("Changing Solar Relay2 to ");
  if(messageTemp2 == "on"){
    digitalWrite(Relay2, HIGH);
    Serial.print("On");
  }
  else if(messageTemp2 == "off"){
    digitalWrite(Relay2, LOW);
    Serial.print("Off");
  }
}
Serial.println();
}

void setup()
{
  pinMode(Relay, OUTPUT);
  pinMode(Relay2, OUTPUT);
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");


 // begin Wifi connect
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(2000);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //end Wifi connect

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}
  void topicsSubscribe(){
  client.subscribe("Solar/Relay");
  client.subscribe("Solar/Relay2");
}

void reconnect() {
  // Loop until we're reconnected
  int counter = 0;
  while (!client.connected()) {
    if (counter==5){
      ESP.restart();
    }
    counter+=1;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
   
    if (client.connect("electricMonitor2")) {
      Serial.println("connected");
      topicsSubscribe();
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  topicsSubscribe();
}

void loop() {
  if (!client.connected()){
    reconnect();
  }

  if (millis() > millisNow + sendDelay) {
    
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);

  
  volts1 = volts1 + 0.005;
  volts1 = volts1 * 100.00;
  int voltsA = (int)volts1;
  float voltsA1 = ((float)voltsA)/100.00;
  volts1 = voltsA1;   // method to bring volts1 to two digits behind the decimal, allows for more accurate current calculations
  

   Vin = (volts0)* (R1+R2)/R2;                    // voltage divider calculations for battery
   Vsol = (volts3)* (R3+R4)/R4;                   // voltage divider calculations for solar input
   current = (1.675-(volts2)) * 10.0/0.12;        // current to/from battery calculation (from voltage change) current sensor
   current2 = (1.675-(volts1)) * 10.0/0.12;       // current from solar input calculation (from voltage change) current2 sensor
   if (current2 > 0.10 && current2 < 0.65) {
    current2 = 0.00;
   }                                              // parameters to correct low voltage for low current(0) reading
   if (current > 0.10 && current < 0.65) {
    current = 0.00;
   }
   if (current2 > -.85 && current2 <= 0.10){
    current2 = 0.00;
   }
   power = Vin * current;
   power2 = Vsol * current2;
   //power = abs(power);

   //if (Vin > 25.6){
   // digitalWrite(Relay, LOW);
   // Serial.print("Relay is now Off");
   // }
   

  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
  Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");
  Serial.print("Vin = "); Serial.println(Vin);
  Serial.print("Vsol = "); Serial.println(Vsol);
  Serial.print("current = "); Serial.println(current);
  Serial.print("current2 = "); Serial.println(current2);
  Serial.print("power = "); Serial.println(power);
  Serial.print("power2 = "); Serial.println(power2);


  client.publish(mqttVoltsIn, String(Vin).c_str(),true);
  client.publish(mqttVoltsSol, String(Vsol).c_str(),true);
  client.publish(mqttCurrentOut, String(current).c_str(),true);
  client.publish(mqttCurrent2Out, String(current2).c_str(),true);
  client.publish(mqttPower, String(power).c_str(),true);
  client.publish(mqttPower2, String(power2).c_str(),true);
  millisNow = millis();

}
  
  client.loop();
}
