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

 int var_LastParticulateRead = 0; // Number of milliseconds since last read
 float var_ParticulateReadValue = 0.0; //value read from sensor
 int var_ParticulateAccumulator = 0; // used to "remember" if any of the 3 particulate readings are in the warning zone
 
 int var_LastServerPing = 0;
 
//PARTICULATE SENSOR ITEMS
uint8_t buf[30];

unsigned int lastTime, last, lastSound; // used to determine the time elapsed between sensor reads

const char* arr_ParticulateString[] = 
  {
    "Sensor ID: ", 
    "PM 1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
    "PM 2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
    "PM 10.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
    // "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
    // "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
    // "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
  };
  uint16_t arr_WarningThreshhold[4] = {0, 7, 11, 13}; // used only for the particulate sensor
  const unsigned int var_SensorReadInterval = 30000; // in milliseconds
  float var_SoundDB;
  
 SYSTEM_THREAD(ENABLED);
 SYSTEM_MODE(SEMI_AUTOMATIC);

 void func_ReadParticulateSensorValue(uint8_t* data, int data_len);
 void func_ParseParticulateResultValue(uint8_t* data);
 void func_ParseResult(uint8_t* data);
 
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

 //SENSOR
  Wire.begin();
  Wire.beginTransmission(0x40);
  Wire.write(0x88);
  Wire.endTransmission(false);
}

 void loop() {
  //  if(nextButton.isClicked()) {
  //    Serial.printf("Next Song\n");
  //    myDFPlayerVoice.next();
  //    myDFPlayerInstru.next();
  //  }
   tempC = bme.readTemperature();
   humidRH = bme.readHumidity();
   tempF = (tempC*9/5)+32;
     Serial.printf("Temp: %.2f%c\n ", tempF,DEGREE); 
     Serial.printf("Humi: %.2f%c\n",humidRH,PERCENT);

     if((millis()-lastTime) > var_SensorReadInterval) {    
      func_ReadParticulateSensorValue(buf,29);
      func_ParseParticulateResultValue(buf);
      func_ParseResult(buf);
  
      // Print a couple of blank lines for readability
      Serial.printf("\n\n------------- \n\n");
  
      lastTime = millis();
    }
        
    delay(1); // allow some time to complete all system tasks
  }
  

 void func_ReadParticulateSensorValue(uint8_t* data, int data_len) {
  int i;
  
  Wire.requestFrom(0x40, 29);
  for (i = 0; i < data_len; i++) {
    data[i] = Wire.read();
  }
}
void func_ParseParticulateResultValue(uint8_t* data)
{
//  for (int i = 0; i < 28; i++)
//  {
//    Serial.print("From ParseResultValue (these are HEX values): ");
//    Serial.println(data[i], HEX);
//  }
  uint8_t sum = 0;
    for (int i = 0; i < 28; i++)
    {
        sum += data[i];
    }
    if (sum != data[28])
    {
        Serial.printf("wrong checkSum!! \n");
    }
}