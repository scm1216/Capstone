/* 
 * Project myProject
 * Author: Sofia Cortes
 * Date: April 4, 2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

 #include "DFRobotDFPlayerMini.h"
 #include "Button.h"
 #include "Adafruit_BME280.h"
 #include <Wire.h>
 #include <Adafruit_MQTT.h>
 #include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
 #include "Adafruit_MQTT/Adafruit_MQTT.h"
 #include "credentials.h"
 #include "Seeed_HM330X.h"

// Declare particulate sensor object
HM330X PMsensor;

DFRobotDFPlayerMini myDFPlayerVoice;

//SYSTEM
SYSTEM_MODE(AUTOMATIC);

//ADAFRUIT.IO
TCPClient TheClient; 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
Adafruit_MQTT_Publish HUM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/auraHum");
Adafruit_MQTT_Publish TEMP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/auratemp");
Adafruit_MQTT_Publish UV = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/auraUV");
Adafruit_MQTT_Publish AIRQUALITY = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/auraairQ");
/************Declare Functions and Variables*************/
int buttonState;
unsigned long publishTime;
void MQTT_connect();
bool MQTT_ping();


 //UV SENSOR
 const int uvSensor = D14;
 float valueUv;
 float uvArray [10];
 int uvIndex;
 int totalUv;
 int k;
 float avgUv;

 //BME SENSOR
 Adafruit_BME280 bme;
 bool status;
 const int hexAddress = 0x76;
 unsigned int currentTime;
 unsigned int lastSecond;
 float tempArray [10];
 float humidRH;
 unsigned long lastTime, last;
 
 //THRESHOLDS
 const float OVERHEAT_THRESHOLD = 90; // 102.2 is considered Overheat in humans
 const float COLD_THRESHOLD = 60.0; // 50.0 is considered OverCold in humans
 const int UV_THRESHOLD = 150;
 const int AQ_THRESHOLD24 = 150;
 const int AQ_THRESHOLD1 = 300;


 float tempC;
 int i, p;
 int j;
 float totalTemp = 0;
 float avgTemp;
 float currentTemp;
 float totalAq;

// Declare variables and constants
uint8_t buf[30];
uint16_t pm10;
int pm25;
int m, sumPM10;
float avgPM10;

const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

 
 HM330XErrorCode print_result(const char* str, uint16_t value);
 HM330XErrorCode parse_result(uint8_t* data);
 HM330XErrorCode parse_result_value(uint8_t* data);

 void setup() {

  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);
  Serial1.begin(9600);
  Serial3.begin(9600);
  delay(1000);

  Serial.printf("DFRobot DFPlayer Mini Demo\n");
  Serial.printf("Initializing DFPlayer ... (May take 3~5 seconds)\n");

  if (!myDFPlayerVoice.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.printf("Unable to start Voice:\n");
    Serial.printf("1.Please recheck the connection!\n");
    Serial.printf("2.Please insert the SD card!\n");
  }
  else {
   Serial.printf("Voice Ready to Go\n");
  }
  myDFPlayerVoice.volume(20);  //Set volume value. From 0 to 30

  if (PMsensor.init()) {
    Serial.printf("HM330X init failed!!!\n");
    while (1);
}

//ADAFRUIT.IO
Serial.printf("Connecting to Internet \n");
WiFi.connect();
while(WiFi.connecting()) {
  Serial.printf(".");
}
Serial.printf("\n Connected!!!!!! \n");

 //BME 
 status = bme.begin (hexAddress); 
 if (status== false){
 Serial.printf("BME280 at address 0x%02x failed to start", hexAddress);
 }

//UV SENSOR
pinMode (uvSensor,INPUT);

}

 void loop() {
  if (mqtt.Update()) {
  if((millis()-lastTime) >1000){
    lastTime = millis ();

// ========= TEMPERATURE ===========
    currentTemp = ((bme.readTemperature())*(9.0/5.0)+32);
    if (currentTemp>190){
      currentTemp = 0;
      currentTemp = ((bme.readTemperature())*(9.0/5.0)+32);
    }

    tempArray [i] = currentTemp;
    // Serial.printf("arr TEMP: %i--- arr value: %.1f\n", i, tempArray[i]);
    i++;
    if (i == 10){
      i=0;
      totalTemp = 0;
      avgTemp =0 ;
      for (j=0; j<10; j++){
        totalTemp = totalTemp + tempArray [j];

      }     
      avgTemp = totalTemp/10.0;
      Serial.printf("Avg TEMP: %.1f\n ",avgTemp);
      TEMP.publish (avgTemp);

// OVERHEATING ALERT
      if(avgTemp > OVERHEAT_THRESHOLD){
        Serial.printf("Warning! High temperature detected");
        myDFPlayerVoice.play(1);
      }
// COLD ALERT
      if(avgTemp < COLD_THRESHOLD){

        Serial.printf("Warning! Cold temperature detected");
        myDFPlayerVoice.play(2);
      }  
     }
// HUMIDITY
      humidRH = bme.readHumidity();
      HUM.publish (humidRH);
// ========= UV ===========
    valueUv = analogRead (uvSensor);
    uvArray[uvIndex] = valueUv; 
    // Serial.printf("arr UV: %i--- arr value: %.1f\n", uvIndex, uvArray[uvIndex]);
    uvIndex++;
 
    if (uvIndex == 10) {
      uvIndex = 0;
      totalUv = 0;
      for (int k = 0; k < 10; k++) {
        totalUv = totalUv + uvArray [k];
      }
      avgUv = totalUv / 10.0;
      Serial.printf("Avg UV: %.2f\n",avgUv);
      UV.publish (avgUv);

      if (avgUv>UV_THRESHOLD) {
        Serial.println("Warning! UV exposure too high!");
        myDFPlayerVoice.play(3);
      }
    }
// ========= AIR QUALITY (PM10) ===========

    if (PMsensor.read_sensor_value(buf, 29)) {
      Serial.printf("HM330X read result failed!!!\n");
    }
    pm10 =buf [7*2] << 8 | buf [7*2+1];
    Serial.printf("PM10 CONC: %i\n" ,pm10);
    // AIRQUALITY.publish (pm10);
    sumPM10 = sumPM10 + pm10;
    if(m>=10){
      m++;
      avgPM10 = (float)sumPM10/m;
      m = 0;
      sumPM10=0;
      if (avgPM10>AQ_THRESHOLD1) {
      Serial.println("Warning! Pollution High!");
      myDFPlayerVoice.play(4);
    }
  }
}
  }
}
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>600000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}