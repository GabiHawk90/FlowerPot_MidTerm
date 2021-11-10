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

// #include "Adafruit_MQTT.h"

// #include "Adafruit_MQTT/Adafruit_MQTT.h"
// #include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
//#include "Adafruit_MQTT/Adafruit_MQTT.h"

// #include "credentials.h"

#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_BMP280 bmp;
const int DUST_SENSOR_PIN = D4;
const int SENSOR_READING_INTERVAL = 30000;
const byte BMPADDRESS = 0x76;
int moistPin = A2;
int val = 0;
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













// /************ Global State (you don't need to change this!) ***   ***************/
// TCPClient TheClient;

// // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
// Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

// /****************************** Feeds ***************************************/
// // Setup Feeds to publish or subscribe
// // Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
// Adafruit_MQTT_Publish objRandom= Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/random");
// Adafruit_MQTT_Subscribe objButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/button");

/************Declare Variables*************/
unsigned long last, lastTime;
float value1, value2;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup()
{
  Serial.begin(9600);

  pinMode(DUST_SENSOR_PIN, INPUT);
  lastInterval = millis();

  //  waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup
  //   //Connect to WiFi without going to Particle Cloud
  //   WiFi.connect();
  //   while(WiFi.connecting()) {
  //     Serial.printf(".");
  //   }

  //   // Setup MQTT subscription for onoff feed.
  //   mqtt.subscribe(&objButton);

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

  
  int temp, pressure, humidity;

  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - lastInterval) > SENSOR_READING_INTERVAL)
  {
 


 
  }


  // // Validate connected to MQTT Broker
  // MQTT_connect();

  // // Ping MQTT Broker every 2 minutes to keep connection alive
  // if ((millis()-last)>120000) {
  //     Serial.printf("Pinging MQTT \n");
  //     if(! mqtt.ping()) {
  //       Serial.printf("Disconnecting \n");
  //       mqtt.disconnect();
  //     }
  //     last = millis();
  // }

  // // publish to cloud every 30 seconds
  // value1 = random(0,100);
  // if((millis()-lastTime > 30000)) {
  //   if(mqtt.Update()) {
  //    objRandom.publish(value1);
  //     Serial.printf("Publishing %0.2f \n",value1);
  //     }
  //   lastTime = millis();
  // }

  // // this is our 'wait for incoming subscription packets' busy subloop
  // Adafruit_MQTT_Subscribe *subscription;
  // while ((subscription = mqtt.readSubscription(1000))) {
  //   if (subscription == &objButton) {
  //     value2 = atoi((char *)objButton.lastread);
  //         Serial.printf("Received %0.2f from Adafruit.io feed FeedNameB \n",value2);
  //   }
  // }

  // // Function to connect and reconnect as necessary to the MQTT server.
  // void MQTT_connect() {
  //   int8_t ret;

  //   // Stop if already connected.
  //   if (mqtt.connected()) {
  //     return;
  //   }

  //   Serial.print("Connecting to MQTT... ");

  //   while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
  //        Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
  //        Serial.printf("Retrying MQTT connection in 5 seconds..\n");
  //        mqtt.disconnect();
  //        delay(5000);  // wait 5 seconds
  //   }
  //   Serial.printf("MQTT Connected!\n");
  // }

  val = analogRead(moistPin);
  Serial.println(val);

  DateTime = Time.timeStr(); // Current Date and Time from Particle Time class
  TimeOnly = DateTime.substring(11, 19);

  tempC = bmp.readTemperature();
  tempF = (tempC * 9.0 / 5.0) + 32;
  showTempDegree();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.printf("%s\n", TimeOnly.c_str());
  display.display();
  delay(2000);
  display.clearDisplay();
}
void showTempDegree(void)
{

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.setCursor(7, 9);
  display.printf("%0.1f%c", tempF, DEGREE);
  display.display();
  delay(2000);
  display.clearDisplay();
}


void getDustSensorReadings()
{
  if (lowpulseoccupancy == 0)
    lowpulseoccupancy = last_lpo;
  else
    last_lpo = lowpulseoccupancy;

Serial.printlnf("LPO: %d", lowpulseoccupancy);
Serial.printlnf("Ratio: %f%%", ratio);
Serial.printlnf("Concentration: %f pcs/L", concentration);
}


