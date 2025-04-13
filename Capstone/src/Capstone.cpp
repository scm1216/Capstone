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

 #define SENSOR_ADDRESS 0X40

 DFRobotDFPlayerMini myDFPlayerVoice;
 DFRobotDFPlayerMini myDFPlayerInstru;

 Button nextButton(D0);
 unsigned int lastSong;

 //UV SENSOR
 const int uvSensor = D14;
 int valueUv;

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
 unsigned long lastTime, last;

 uint8_t buf[30];

 const char* str[] = {"sensor num: ", 
  "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
  "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
  "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
  "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
  "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
  "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
  };
 
  void read_sensor_value(uint8_t* data, uint32_t data_len);
  void print_result(const char* str, uint16_t value);
  void atmosphericMatterRead(uint8_t* data);


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
   myDFPlayerVoice.volume(25);  //Set volume value. From 0 to 30
   myDFPlayerVoice.loop(1);  //Play the first mp3
   myDFPlayerVoice.enableLoopAll();
   myDFPlayerInstru.volume(25);  //Set volume value. From 0 to 30
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
 
 //UV SENSOR
 pinMode (uvSensor,INPUT);
}
 void loop() {

  if((millis()-lastTime) >1000){
   tempC = bme.readTemperature();
   humidRH = bme.readHumidity();
   tempF = (tempC*9/5)+32;

     Serial.printf("Temp: %.2f%\n ", tempF); 
     Serial.printf("Humi: %.2f%c\n",humidRH,PERCENT);

      read_sensor_value(buf,29); //Request 29 bytes of data
      atmosphericMatterRead(buf);
      lastTime = millis();
    
//UV SENSOR
valueUv = analogRead(uvSensor);
Serial.printf("UV %i \n",valueUv);
}
  }


  void read_sensor_value(uint8_t* data, uint32_t data_len){
    Wire.requestFrom(0x40, 29);
    for (int i = 0; i < data_len; i++){
      data[i] = Wire.read();
    }
  }
    void atmosphericMatterRead(uint8_t* data){
    uint16_t value2 = 0;
    int i = 7;
    value2 = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
    print_result(str[i - 1], value2);
    }
  
  void print_result(const char* str, uint16_t value){
      Serial.printf(str);
      Serial.println(value);
  }
  //150