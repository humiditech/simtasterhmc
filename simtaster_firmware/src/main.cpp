#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "OneWire.h"

#define DS18S20_PIN 23

OneWire ds(DS18S20_PIN);

#define TdsSensorPin  34
#define VREF          3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT        30       // sum of sample point

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

#define DO_PIN 32

#define VREF 3300    //VREF (mv)
#define ADC_RES 4096 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

const char* WIFI_SSID = "BISA";
const char* WIFI_PASS = "bayu1234";

String serverName = "http://192.168.0.122:5000/recv-sensor";

unsigned long prevMillis = 0;
#define SEND_INTERVAL 200

char postBodyBuf[100];

float readPH();
float readDO();
float readTemp();
void readTDS();
int getMedianNum(int bArray[], int iFilterLen);

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.println("Connecting");

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  readTDS();
  if ((millis() - prevMillis) > SEND_INTERVAL) {
    Serial.println(readTemp());
    // sprintf(postBodyBuf, "{\"ph\":\"%s\",\"do\":\"%s\",\"tds\":\"%s\",\"temp\":\"%s\"}",
    //                       String(readPH(), 2), String(readDO(), 2), String(readTDS(), 2), String(readTemp(), 2));

    // if(WiFi.status()== WL_CONNECTED){
    //   WiFiClient client;
    //   HTTPClient http;
    
    //   http.begin(client, serverName);

    //   http.addHeader("Content-Type", "application/json");
    //   int httpResponseCode = http.POST(postBodyBuf);

    //   Serial.print("HTTP Response code: ");
    //   Serial.println(httpResponseCode);
        
    //   http.end();
    // }
    // else {
    //   Serial.println("WiFi Disconnected");
    // }
    prevMillis = millis();
  }
}

float readPH(){
  // TODO : write pH sensor reading here

  return (float)random(0, 14);
}

float readDO(){
  Temperaturet = (uint8_t)readTemp();
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  
  #if TWO_POINT_CALIBRATION == 0
    uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * Temperaturet - (uint32_t)CAL1_T * 35;
    return (ADC_Voltage * DO_Table[Temperaturet] / V_saturation);
  #else
    uint16_t V_saturation = (int16_t)((int8_t)Temperaturet - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
    return (ADC_Voltage * DO_Table[Temperaturet] / V_saturation);
  #endif

  // return (float)random(0, 100);
}

float readTemp(){
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

  // return (float)random(0, 50);
}

void readTDS(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT) 
        analogBufferIndex = 0;
  }   
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
  }

  // return (float)random(0, 100);
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  
  return bTemp;
}