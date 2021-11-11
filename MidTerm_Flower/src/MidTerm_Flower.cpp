/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/gabi/Documents/IoT/FlowerPot_MidTerm/MidTerm_Flower/src/MidTerm_Flower.ino"
/*
 * Project MidTerm_Flower
 * Description:automatic watering flowerpot
 * Author:Gabriella Hawkins
 * Date:11/10/21
 */

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BMP280.h"
#include "math.h"
#include "Air_Quality_Sensor.h"
#include "cred.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
// #include "Adafruit_MQTT/Adafruit_MQTT.h"

void setup();
void loop();
void showTempDegree(void);
void showMoisture(int _moistureReading);
void publishMoisture(int _moistureReading);
String getAirQuality(int _aqSensor);
void MQTT_connect();
void pumpON(int moistureReading, int buttonvalue);
void pumpOff();
void pingMQTT();
#line 19 "c:/Users/gabi/Documents/IoT/FlowerPot_MidTerm/MidTerm_Flower/src/MidTerm_Flower.ino"
#define AQS_PIN A0
#define OLED_RESET D4
AirQualitySensor aqSensor(AQS_PIN);

Adafruit_SSD1306 display(OLED_RESET);

Adafruit_BMP280 bmp;
const int DUST_SENSOR_PIN = D4;
int moistPin = A2;
int airPin = A0;
int pumpPin = 11;
AirQualitySensor sensor(A0);
const int SENSOR_READING_INTERVAL = 30000;
const byte BMPADDRESS = 0x76;
int val = 0;

int quality = sensor.slope();
int temp;
int airQual;
bool BmpStatus;
const char DEGREE = 0xF8; // Decimal 248 = 0 xF8
float tempC;
float tempF;
float ratio = 0;
float concentration = 0;

void getDustSensorReadings();

String DateTime, TimeOnly;

unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long last_lpo = 0;
unsigned long duration;
unsigned long startPumpTime;

/************ Global State (you don't need to change this!) ***   ***************/
TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
// Setup Feeds to publish or subscribe
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish objDust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Dust");
Adafruit_MQTT_Publish objSoilM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/SoilMoisture");
Adafruit_MQTT_Publish objRoomTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RoomTemp");
Adafruit_MQTT_Publish objAirQ = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQ");

Adafruit_MQTT_Subscribe objPumpOn = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/OnOff");

/************Declare Variables*************/
unsigned long last, lastTime;
float value1, value2;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup()
{
  Serial.begin(9600);
  pinMode(11, OUTPUT);

  waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup
  //Connect to WiFi without going to Particle Cloud
  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf(".");
  }

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&objPumpOn);

  Serial.println("Waiting sensor to init...");
  delay(20000);

  if (sensor.init())
  {
    Serial.println("Sensor ready.");
  }
  else
  {
    Serial.println("Sensor ERROR!");
  }

  if (aqSensor.init())
  {
    Serial.println("Air Quality Sensor ready.");
  }
  else
  {
    Serial.println("Air Quality Sensor ERROR!");
  }

  pinMode(DUST_SENSOR_PIN, INPUT);
  lastInterval = millis();

  pinMode(moistPin, INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();

  if (!bmp.begin(BMPADDRESS))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  Time.zone(-7); // MST = -7, MDT = -6
  Particle.syncTime();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (for the 128x64)
  // init done

  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay(); // clears the screen and buffer
}

void loop()
{
  // Validate connected to MQTT Broker
  MQTT_connect();
  pingMQTT();

  int soilReading=analogRead(moistPin);

 
  int buttonvalue=0;
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000)))
  {
    if (subscription == &objPumpOn)
    {
      buttonvalue = atoi((char *)objPumpOn.lastread);
      Serial.printf("Received %i from Adafruit.io feed FeedNameB \n", buttonvalue);
      pumpON(soilReading,buttonvalue);
    }
  } 
  pumpOff();

  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());

  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  DateTime = Time.timeStr(); // Current Date and Time from Particle Time class
  TimeOnly = DateTime.substring(11, 19);

  tempC = bmp.readTemperature();
  tempF = (tempC * 9.0 / 5.0) + 32;

  //Publish data
  if ((millis() - lastTime > 10000))
  {
    if (mqtt.Update())
    {
      objRoomTemp.publish(tempF);
      Serial.printf("Publishing Temp %0.2f\n", tempF);
      publishMoisture(soilReading);
      objDust.publish(concentration);
      objAirQ.publish(airQual);
      Serial.printf("Publishing quality %i\n", quality);
    }
    lastTime = millis();
  

  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.printf("%s\n", TimeOnly.c_str());
  display.display();
  display.clearDisplay();
  delay(3000);

  showTempDegree();
  
  }
  
void showTempDegree(void)
{

  
 
  display.setTextSize(2);
  display.setCursor(2, 2);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.printf("%0.1f%c", tempF, DEGREE);
  display.display();
 display.clearDisplay();
  
}

void showMoisture(int _moistureReading)
{

  display.clearDisplay();
  display.setCursor(3, 3);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.printf("Soil Moisture %i\n", _moistureReading);
  display.display();
  display.clearDisplay();
}

void publishMoisture(int _moistureReading)
{
  objSoilM.publish(_moistureReading);
  Serial.printf("Publishing Soil %i\n", _moistureReading);
}

void getDustSensorReadings()
{
  if (lowpulseoccupancy == 0)
    lowpulseoccupancy = last_lpo;
  else
    last_lpo = lowpulseoccupancy;

  // Serial.printlnf("LPO: %d", lowpulseoccupancy);
  Serial.printlnf("Ratio: %f%%", ratio);
  Serial.printlnf("Concentration: %f pcs/L", concentration);
}

String getAirQuality(int _aqSensor)
{

  int quality = aqSensor.slope();
  airQual = aqSensor.getValue();
  String qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    qual = "Danger";
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    qual = "High Pollution";
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    qual = "Low Pollution";
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    qual = "Fresh Air";
  }

  return qual;
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}

 void pumpON(int moistureReading, int buttonvalue){

  Serial.printf("pump on\n");
  if( moistureReading > 3446||buttonvalue);{
    digitalWrite(pumpPin,HIGH);
    startPumpTime=(millis());
  }
}

void pumpOff(){
  Serial.printf("pump off\n");
  if (millis()-startPumpTime>2000) {
    digitalWrite(pumpPin,LOW);
  }
}
  
void pingMQTT(){
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis() - last) > 120000)
  {
    Serial.printf("Pinging MQTT \n");
    if (!mqtt.ping())
    {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
}