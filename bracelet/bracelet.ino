#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

/**
 * LED blinker global variables
 */
const int LED_PIN=2;
long lastMillis =0;
int blinkInterval =0;
int blinkLength =0;

/**
 * WIFI global variables 
 */
const char* ssid = "SENSE_IT";  
const char* password = "12345678";  
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
bool blinkStatus= LOW;
bool pulseStatus= LOW;
WebServer server;

/**
 * Max30102 global variables
 */
 /**
 * MAX20102 settings
 */
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
MAX30105 particleSensor;
bool isParticleSensorConnected = LOW;
long lastMillisParticleBlink =0;

/************************************************** Code Segment ****************************************/
/**
 * LED blinker code segment
 */

void blinkerStart(int interval, int length) {
  lastMillis=millis();
  blinkInterval=interval;
  blinkLength = length;
  digitalWrite(LED_PIN, HIGH);
}

void blinkerStop() {
  digitalWrite(LED_PIN, LOW);
  lastMillis=0;
  blinkInterval=0;
  blinkLength = 0;
}

void blinkerHandle() {
  if (blinkLength == 0) //bloink is stopped
    return;

  long currentMillis = millis()-lastMillis;
  if (currentMillis <= blinkLength) {
    digitalWrite(LED_PIN, HIGH); // turn on led , in the middle of the ON length time
  } else if (currentMillis <= (blinkInterval+blinkLength)) {
    digitalWrite(LED_PIN, LOW); // turn off led,  in the middle of the OFF interval between blinks
  } else {
    lastMillis = millis(); //reset the cycle and start over
    blinkerHandle();
  }
}

/**
 * WIFI code segment
 */
void wifiSetup() {
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", wifiHandle_OnConnect);
  server.on("/blinkOn", wifiHandle_blinkOn);
  server.on("/blinkOff", wifiHandle_blinkOff);
  server.on("/pulseOn", wifiHandle_pulseOn);
  server.on("/pulseOff", wifiHandle_pulseOff);
  server.onNotFound(wifiHandle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void wifiHandle() {
  server.handleClient();

//    if(LED1status)
//    {digitalWrite(LED1pin, HIGH);}
//    else
//    {digitalWrite(LED1pin, LOW);}

}

void wifiHandle_OnConnect() {
  blinkStatus = LOW;
  Serial.println("http: blink Status: OFF");
  server.send(200, "text/html", wifiSendHTML(blinkStatus, pulseStatus));
}

void wifiHandle_blinkOn() {
  blinkStatus = HIGH;
  blinkerStart(1000, 1000);
  Serial.println("http blink Status: ON");
  server.send(200, "text/html", wifiSendHTML(true, pulseStatus));
}

void wifiHandle_blinkOff() {
  blinkStatus = LOW;
  blinkerStop();
  Serial.println("http blink Status: OFF");
  server.send(200, "text/html", wifiSendHTML(false, pulseStatus));
}

void wifiHandle_pulseOn() {
  pulseStatus = HIGH;
  Serial.println("http pulse Status: ON");
  server.send(200, "text/html", wifiSendHTML(blinkStatus, true));
}

void wifiHandle_pulseOff() {
  blinkStatus = LOW;
  Serial.println("http pulse Status: OFF");
  server.send(200, "text/html", wifiSendHTML(blinkStatus, false));
}

void wifiHandle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String wifiSendHTML(uint8_t blinkStat, uint8_t pulseStat){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  if (pulseStat) {
    ptr +="<meta http-equiv=\"refresh\" content=\"5\">\n";
  }
  ptr +="<title>SENSE IT control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>CAMBIAR SENSE IT Web Server</h1>\n";
  
  if (particleSensor.getIR() < 50000)
  {ptr +="<h3>Sensor is NOT connected</h3>\n";}
  else
  {ptr +="<h3>Sensor is connected</h3>\n";}

   if(blinkStat)
  {ptr +="<p>blink Status: ON</p><a class=\"button button-off\" href=\"/blinkOff\">OFF</a>\n";}
  else
  {ptr +="<p>blink Status: OFF</p><a class=\"button button-on\" href=\"/blinkOn\">ON</a>\n";}

   if(pulseStat)
  {ptr +="<p>Heart Beat : ON</p><a class=\"button button-off\" href=\"/pulseOff\">"; ptr +=beatAvg; ptr +="</a>\n";}
  else
  {ptr +="<p>Heart Beat : OFF</p><a class=\"button button-on\" href=\"/pulseOn\">ON</a>\n";}
  
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}
/**
 *  pulse heart rate code segment
 */
 /**
 * heart rate code segment
 */
void particleSensorSetup() {
  int times=5;
  while (times > 0) {
    if ( particleSensor.begin(Wire, I2C_SPEED_FAST)) {
      isParticleSensorConnected = HIGH;
      Serial.println("MAX30105 connected");
      break;
    }
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    delay(1000);
    times--;
  }

  if (isParticleSensorConnected) {  
    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0); //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED    
  }
  
}

void particleSensorBlink() {
  long tmp = millis() - lastMillisParticleBlink;
  if (tmp > 100) {
    particleSensor.setPulseAmplitudeRed(0);

  } if (tmp > 122) {
    particleSensor.setPulseAmplitudeRed(250);
    lastMillisParticleBlink = millis();
  }
  
}

void sampleHeartRate() {
  if (isParticleSensorConnected) {
    long irValue = particleSensor.getIR();
    
    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
    
    //debugHeartate(irValue);
  }
}

void debugHeartate(long irValue) {
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
} 

 
/**
 * arduino code segment
 */

void setup() {
  Serial.begin(115200);
  
  // LED BLINKER
  pinMode (LED_PIN, OUTPUT);

  // init heart rate sensor
  particleSensorSetup();

  // WIFI code
  wifiSetup();
}

void loop() {
  delay(100);
  blinkerHandle();
  wifiHandle();
  sampleHeartRate();
  particleSensorBlink();  
}
