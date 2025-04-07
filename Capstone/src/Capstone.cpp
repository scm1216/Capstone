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

 DFRobotDFPlayerMini myDFPlayerVoice;
 DFRobotDFPlayerMini myDFPlayerInstru;

 Button nextButton(D0);
 unsigned int lastSong;

 //BME SENSOR
 Adafruit_BME280 bme;
 bool status;
 const int hexAddress = 0x76;
 unsigned int currentTime;
 unsigned int lastSecond;
 float tempC;
 float tempF;
 float humidRH;
 const byte PERCENT = 37;
 const byte DEGREE  = 167;


 SYSTEM_MODE(SEMI_AUTOMATIC);

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
   if (!myDFPlayerInstru.begin(Serial3)) {  //Use softwareSerial to communicate with mp3.
    Serial.printf("Unable to start Instrumental:\n");
    Serial.printf("1.Please recheck the connection!\n");
    Serial.printf("2.Please insert the SD card!\n");
  }
  else {
    Serial.printf("Instrumental Ready to Go\n");
   }
   myDFPlayerVoice.volume(30);  //Set volume value. From 0 to 30
   myDFPlayerVoice.loop(1);  //Play the first mp3
   myDFPlayerVoice.enableLoopAll();
   myDFPlayerInstru.volume(30);  //Set volume value. From 0 to 30
   myDFPlayerInstru.loop(1);  //Play the first mp3
   myDFPlayerInstru.enableLoopAll();

  //BME 
  status = bme.begin (hexAddress); 
  if (status== false){
  Serial.printf("BME280 at address 0x%02x failed to start", hexAddress);
  }
 }

 void loop() {
   if(nextButton.isClicked()) {
     Serial.printf("Next Song\n");
     myDFPlayerVoice.next();
     myDFPlayerInstru.next();
   }
   tempC = bme.readTemperature();
   humidRH = bme.readHumidity();
   tempF = (tempC*9/5)+32;
     Serial.printf("Temp: %.2f%c\n ", tempF,DEGREE); 
     Serial.printf("Humi: %.2f%c\n",humidRH,PERCENT);
 }