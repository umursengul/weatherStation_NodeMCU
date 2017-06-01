/*********************************************************************/
/*              Multisensor Remote Sensing with NodeMCU              */
/*                                                                   */
/*                                                                   */
/*********************************************************************/

/////////////////////
//    LIBRARIES    //
/////////////////////
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

/////////////////////////////
//    WiFi & ThingSpeak    //
/////////////////////////////
const char* ssid = "";                      // WiFi name
const char* password = "";                  // WiFi password
const char* server = "api.thingspeak.com";  // API server
String apiKey = "";                         // API key from thingspeak

///////////////////////////
//    PIN DEFINITIONS    //
///////////////////////////
#define DHTPIN 2                // Pin # of DHT11
#define P_INT 12                // Pin # of Anemometer
int sensorPin = A0;             // Pin # of LDR and YL-83
int rainSensorEnablePin = 13;   // Enable Pin # of YL-83
int lightSensorEnablePin = 15;  // Enable Pin # of LDR

////////////////////
//  DEFINED VARs  //
////////////////////
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
DHT dht(DHTPIN, DHT11,15);
WiFiClient client;
#define ANEMOMETER_DATA_COUNT 10
unsigned long anemometerData[ANEMOMETER_DATA_COUNT] = { 0 };
void onInterrupt();
int lightSensorVal = 0;
int rainSensorVal = 0;
float windSpeed = 0.0000;
// Must be uncommented for Bluetooth:
/*
// Following string variables are necessary for Bluetooth
// output to WeatherStation app:
static char outstr[15];         // Temporary String 1
String temporaryString;         // Temporary String 2
String temperatureSensorValue;
String pressureSensorValue;
String humiditySensorValue;
String dewSensorValue;
String lightSensorValue;
String windSensorValue;
String rainSensorValue;
String sensorValues;
*/

void setup()
{
  /////////////////
  //  PIN MODES  //
  /////////////////
  pinMode(lightSensorEnablePin, OUTPUT);  // Enables sensor pin for reading
  pinMode(rainSensorEnablePin, OUTPUT);   // Enables sensor pin for reading

  ////////////////////////
  //  SERIAL COMM. INIT //
  ////////////////////////
  Serial.begin(9600);
  delay(10);

  ///////////////
  //  DHT INIT //
  ///////////////
  dht.begin();

  ///////////////////////
  //  ANEMOMETER INIT  //
  ///////////////////////
  // r_sensor ~= 1.75cm
  // circumference ~= 11cm
  attachInterrupt(digitalPinToInterrupt(P_INT), onInterrupt, RISING);

  //////////////////////
  //  WiFi COMM. INIT //
  //////////////////////
  WiFi.begin(ssid, password);
  delay(200);

  // Must be commented out for Bluetooth:
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Serial.print("..........");
  Serial.println();
  //
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  // Must be commented out for Bluetooth:
  Serial.println("WiFi connected");
  Serial.println();
  //
}


void loop()
{
  ///////////////////////////////////////
  //  TEMP. & HUMIDITY SENSOR - DHT11  //
  ///////////////////////////////////////
  float h = dht.readHumidity();
  delay(500);
  float t = dht.readTemperature();
  delay(500);

  if (isnan(h) || isnan(t))
  {
    // Must be commented out for Bluetooth:
    Serial.println("Failed to read from DHT sensor!");
    //
    return;
  }

  // Temperature in Celcius:
  // Must be uncommented for Bluetooth:
  /*
  dtostrf(t,4,2,outstr);
  temporaryString = outstr;
  temperatureSensorValue = temporaryString + ",";
  */
  // Must be commented out for Bluetooth:
  Serial.print("Temperature:      ");
  Serial.print(t);
  Serial.print(" degrees Celcius ");
  Serial.println();
  //
  // Must be uncommented for Bluetooth:
  /*
  dtostrf(h,4,2,outstr);
  temporaryString = outstr;
  humiditySensorValue = temporaryString + ",";
  */
  // Must be commented out for Bluetooth:
  Serial.print("Humidity:         ");
  Serial.print(h);
  Serial.print("%");
  Serial.println();
  //

  // Dew point temperature from temperature and humidity
  double gamma = log(h/100) + ((17.62*t) / (243.5+t));
  double dp = 243.5*gamma / (17.62-gamma);

  // Must be commented out for Bluetooth:
  Serial.print("Dew point:        ");
  Serial.print(dp);
  Serial.print(" degrees Celcius ");
  Serial.println();
  //

  ////////////////////////////////
  //  PRESSURE SENSOR - BMP180  //
  ////////////////////////////////
  if(!bmp.begin())
  {
    // Must be commented out for Bluetooth:
    Serial.print("Failed to read from BMP sensor!!");
    //
    while(1);
  }

  sensors_event_t event;
  bmp.getEvent(&event);

  // Must be uncommented for Bluetooth:
  /*
  dtostrf(event.pressure,6,2,outstr);                           // Pressure in mb
  temporaryString = outstr;
  pressureSensorValue = temporaryString + ",";
  */
  // Must be commented out for Bluetooth:
  Serial.print("Pressure:         ");
  Serial.print(event.pressure);
  Serial.println(" hPa");
  //
  float temperature;
  bmp.getTemperature(&temperature);

  // Must be commented out for Bluetooth:
  Serial.print("Temperature:      ");
  Serial.print(temperature);
  Serial.println(" degrees Celcius ");
  //

  // Altitude from the temperature and the air pressure
  float seaLevelPressure = 1015;
  // Must be commented out for Bluetooth:
  Serial.print("Altitude:         ");
  Serial.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure));
  Serial.println(" m");
  //

  //////////////////////////
  //  LIGHT SENSOR - LDR  //
  //////////////////////////
  digitalWrite(lightSensorEnablePin, HIGH);
  lightSensorVal = analogRead(sensorPin);
  lightSensorVal = constrain(lightSensorVal, 300, 850);
  lightSensorVal = map(lightSensorVal, 300, 850, 0, 1023);
  // Must be uncommented for Bluetooth:
  /*
  dtostrf(lightSensorVal,6,2,outstr);
  temporaryString = outstr;
  lightSensorValue = temporaryString + ",";
  */

  // Must be commented out for Bluetooth:
  Serial.print("Light intensity:  ");
  Serial.println(lightSensorVal);
  //
  digitalWrite(lightSensorEnablePin, LOW);
  delay(100);

  ///////////////////////////
  //  RAIN SENSOR - YL-83  //
  ///////////////////////////
  digitalWrite(rainSensorEnablePin, HIGH);
  delay(500);
  rainSensorVal = analogRead(sensorPin);
  rainSensorVal = constrain(rainSensorVal, 150, 440);
  rainSensorVal = map(rainSensorVal, 150, 440, 1023, 0);
  // Must be uncommented for Bluetooth:
  /*
  int rainSensorRange = map(rainSensorVal, 0, 1023, 0, 3);
  switch (rainSensorRange)
  {
    case 0:
      rainSensorValue = "0,";
      break;
    case 1:
      rainSensorValue = "1,";
      break;
    case 2:
      rainSensorValue = "2,";
      break;
  }
  */

  // Must be commented out for Bluetooth:
  Serial.print("Rain value:       ");
  Serial.println(rainSensorVal);
  Serial.println();
  //
  digitalWrite(rainSensorEnablePin, LOW);
  delay(100);

  ////////////////////////////////
  //  WIND SENSOR - ANEMOMETER  //
  ////////////////////////////////
  if (anemometerData[0] != 0)
  {
    unsigned long now = millis();
    if (now - anemometerData[ANEMOMETER_DATA_COUNT - 1] < 1000)
    {
      unsigned long passedTimes = now - anemometerData[0];
      windSpeed = ((10000/passedTimes)*(0.11));
      // Must be uncommented for Bluetooth:
      /*
      dtostrf(windSpeed,5,2,outstr);
      temporaryString = outstr;
      windSensorValue = temporaryString + ",";
      */
    }
    else
    {
      // Must be uncommented for Bluetooth:
      //windSensorValue = "0,";
    }
  }
  else
  {
    // Must be uncommented for Bluetooth:
    //windSensorValue = "0,";
  }

  ////////////////////////////
  //  THINGSPEAK - NodeMCU  //
  ////////////////////////////
  if (client.connect(server,80))
  {
    // "184.106.153.149" or api.thingspeak.com
    String postStr = apiKey;
    postStr +="&field1=";
    postStr += String(t);
    postStr +="&field2=";
    postStr += String(h);
    postStr +="&field3=";
    postStr += String(dp);
    postStr +="&field4=";
    postStr += String(event.pressure);
    postStr +="&field5=";
    postStr += String(temperature);
    postStr +="&field6=";
    postStr += String(lightSensorVal);
    postStr +="&field7=";
    postStr += String(rainSensorVal);
    //postStr +="&field8=";
    //postStr += String(windSpeed);
    postStr += String(bmp.pressureToAltitude(seaLevelPressure,event.pressure));
    postStr += "\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n\n\n\n\n\n\n");
    client.print(postStr);
  }
  client.stop();

  ////////////////////////
  //  BT SERIAL OUTPUT  //
  ////////////////////////
  // Must be uncommented for Bluetooth:
  /*
  sensorValues = temperatureSensorValue + humiditySensorValue + pressureSensorValue + windSensorValue + rainSensorValue + lightSensorValue;
  Serial.println(sensorValues);
  */

  // ThingSpeak needs minimum 15 sec delay between updates
  delay(30000); // delay(1000000); 17 minutes
}

////////////////////////////
//  WIND SENSOR FUNCTION  //
////////////////////////////
// by MucarTheEngineer
void onInterrupt()
{
  unsigned long newData = millis();
  for (int i = 1; i < ANEMOMETER_DATA_COUNT; i++)
    anemometerData[i - 1] = anemometerData[i];
  anemometerData[ANEMOMETER_DATA_COUNT - 1] = newData;
}
