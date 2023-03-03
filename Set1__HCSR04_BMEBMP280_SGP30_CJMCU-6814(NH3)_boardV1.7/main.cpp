#include <Arduino.h>

#include "HardwareSerial_NB_BC95.h"

#include <TaskScheduler.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <NewPing.h>

#include <Update.h>
#include "cert.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_SGP30.h"

Adafruit_BMP280 bmp;
Adafruit_BME280 bme;
Adafruit_SGP30 sgp;

#define nh3_pin 35
#define EN_GND_I2C_pin 32
#define EN_GND_Io1213_pin 4
#define EN_GND_Io1617_pin 33
#define EN_GND_Io2526_pin 15

// #define Chip_ID_BME280 0x60
#define Chip_ID_BMP280 0x58
#define BME_BMP_ADDRESS 0x76

#define Microwave_pin 13

#define Sound_pin 34

#define TRIGGER_PIN 25   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 26      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

HardwareSerial_NB_BC95 AISnb;

String FirmwareVer = "0.1";

#define URL_fw_Version "https://raw.githubusercontent.com/greenioiot/PTT_Toliet_STATION_BoardV1.7/main/Set1__HCSR04_BMEBMP280_SGP30_CJMCU-6814(NH3)_boardV1.7/bin_version.txt"

#define URL_fw_Bin "https://raw.githubusercontent.com/greenioiot/PTT_Toliet_STATION_BoardV1.7/main/Set1__HCSR04_BMEBMP280_SGP30_CJMCU-6814(NH3)_boardV1.7/firmware.bin"

Scheduler runner;

// WiFi&OTA 参数
String HOSTNAME = "";
#define PASSWORD "green7650" // the password for OTA upgrade, can set it in any char you want
const char *ssid = "greenio";
const char *password = "green7650";

// char WIFINAME[50];

String deviceToken = "";

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);
String IP;
int status = WL_IDLE_STATUS;

String serverIP = "147.50.151.130"; // Your Server IP;
String serverPort = "19956";        // Your Server Port;
Signal meta;
String json = "";

int nh3;
int dis;
int mic;
int noise;
int TVOC;
int eCO2;
int Dis;
int distan[4], distan_avg;

float temp(NAN), pres(NAN), hum(NAN);

uint16_t modbusdata;

boolean Sta = 0, Sta2 = 0;
boolean Sensor_Type, BME = 1, BMP = 0;

const int Length = 40;
float SensorBuffer[8][Length] = {{},{},{},{},{},{},{},{}};
int Index[8] = {0, 0, 0, 0, 0,0,0,0};
int Ultrasonic = 0, Noise_Sensor = 1,nh3_Sensor = 2,TVOC_Sensor = 3,eCO2_Sensor = 4,temp_Sensor = 5,pres_Sensor = 6,hum_Sensor = 7;

// const int Length2 = 10;
// float SensorBuffer[1][Length2] = {{}};
// int Index2[1] = {0, 0, 0, 0, 0,0,0,0};


unsigned long Read_ultra_Cycle = 100;
unsigned long OTA_Check_Cycle = 60 * 1000;
unsigned long Sentdata_Cycle = 1000 * 60;

uint64_t espChipID = ESP.getEfuseMac();

uint16_t TVOC_base, eCO2_base;

String mac2String(byte ar[])
{
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]); // J-M-L: slight modification, added the 0 in the format for padding
    s += buf;
    if (i < 5)
      s += ':';
  }
  return s;
}

void firmwareUpdate();

int FirmwareVersionCheck(void)
{
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure *client = new WiFiClientSecure;

  if (client)
  {
    client->setCACert(rootCACertificate);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
    HTTPClient https;

    if (https.begin(*client, fwurl))
    { // HTTPS
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      }
      else
      {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();
    }
    delete client;
  }

  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer))
    {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    }
    else
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}

void firmwareUpdate(void)
{
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void OTA_git_CALL()
{
  if (FirmwareVersionCheck())
  {
    firmwareUpdate();
  }
}

void setupOTA()
{

  ArduinoOTA.setHostname(HOSTNAME.c_str());
  ArduinoOTA.setPassword(PASSWORD);
  ArduinoOTA.onStart([]()
                     {    
                      //  lcd.begin(COLUMS, ROWS);
                       Serial.println("Start Updating....");
                       Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem"); });

  ArduinoOTA.onEnd([]()
                   {
                     Serial.println("Update Complete!");
                     // lcd.clear();
                     // lcd.setCursor(0, 0);
                     // lcd.print("Update Complete!");
                   });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
                          String pro = String(progress / (total / 100)) + "%";
                          int progressbar = (progress / (total / 100));
                          Serial.print("Progress : ");
                          Serial.println((progress / (total / 100)));

                          // lcd.clear();
                          // lcd.setCursor(0, 0);
                          // lcd.print("Update :");
                          // lcd.setCursor(9, 0);
                          // lcd.print(progress / (total / 100));
                        });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart(); });
  ArduinoOTA.begin();
}

void appendValueToBuffer(int Sensor, float value)
{
  SensorBuffer[Sensor][Index[Sensor]] = value;
  // Serial.printf("index: %d  \n", Index[Sensor]);
  //  Serial.printf("add: %f  \n", SensorBuffer[Sensor][Index[Sensor]]);
  Index[Sensor]++;
  if (Index[Sensor] >= Length)
  {
    Index[Sensor] = 0;
  }
}

int sum(float array[]) // assuming array is int.
{
  long sum = 0L; // sum will be larger than an item, long for safety.
  for (int i = 0; i < Length; i++){
    sum += array[i];
  }
  return (sum); // average will be fractional, so float may be appropriate.
}

float avg(float array[]) // assuming array is int.
{
  float sum = 0L; // sum will be larger than an item, long for safety.
  for (int i = 0; i < Length; i++){
    sum += array[i];
  }
  Serial.println (sum);
  return (sum/Length); // average will be fractional, so float may be appropriate.
}

void t1HumanDetections();
void t2OTA_Check();
void t3GASDetections();
void t4sendViaNBIOT();
Task t1(Read_ultra_Cycle, TASK_FOREVER, &t1HumanDetections);
Task t2(OTA_Check_Cycle, TASK_FOREVER, &t2OTA_Check);
Task t3(1000, TASK_FOREVER, &t3GASDetections);
Task t4(Sentdata_Cycle, TASK_FOREVER, &t4sendViaNBIOT);

void connect_wifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {

    WiFi.reconnect();
  }
  else
  {
    OTA_git_CALL();
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(EN_GND_I2C_pin, OUTPUT);
  digitalWrite(EN_GND_I2C_pin, HIGH); // on I2C
  pinMode(EN_GND_Io1213_pin, OUTPUT);
  digitalWrite(EN_GND_Io1213_pin, HIGH);
  pinMode(EN_GND_Io2526_pin, OUTPUT);
  digitalWrite(EN_GND_Io2526_pin, HIGH);
  pinMode(EN_GND_Io1617_pin, OUTPUT);
  digitalWrite(EN_GND_Io1617_pin, HIGH);

  pinMode(Microwave_pin, INPUT_PULLUP);
  pinMode(nh3_pin, INPUT);

  // deviceToken = mac2String((byte *)&espChipID);

  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  deviceToken = AISnb.getNCCID();

  HOSTNAME.concat(deviceToken);
  // HOSTNAME.toCharArray(WIFINAME, 50);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
  connect_wifi();

  if (!sgp.begin())
  {
    Serial.println("Sensor not found :(");
    delay(10);
    ;
  }
  else
  {
    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
  }

  if (!bmp.begin(BME_BMP_ADDRESS, Chip_ID_BMP280))
  {
    Sensor_Type = BME;
    Serial.println(F("Could not find a valid BMP280 sensor"));
    if (!bme.begin(BME_BMP_ADDRESS))
    {
      delay(10);
    }
  }
  else
  {
    Serial.println("Found BMP280 sensor");
    Sensor_Type = BMP;
  }

  IP = WiFi.localIP().toString();

  Serial.println(IP);
  setupOTA();

  runner.init();

  runner.addTask(t1);
  runner.addTask(t2);
  runner.addTask(t3);
  runner.addTask(t4);

  delay(5000);
  t1.enable(); // ultra Detections
  t2.enable(); // OTA_Check && reconnect wifi
  t3.enable(); // GASDetections
  t4.enable(); // sendViaNBIOT
}

void read_distance()
{
  distan_avg = 0;
  for (int i = 0; i < 3; i++)
  {
    distan[i] = sonar.ping_cm();
    delay(5);
  }
  if ((abs(distan[0] - distan[1]) < 10) || (abs(distan[0] - distan[2]) < 10) || (abs(distan[1] - distan[2]) < 10))
  {
    distan_avg = (distan[0] + distan[1] + distan[2]) / 3;
    if (distan_avg < 100)
    {
      appendValueToBuffer(Ultrasonic, 1);
      Serial.printf("Add: %d cm to Buffer\n", distan_avg);
    }
  }
}

void t1HumanDetections()
{
  Dis = sonar.ping_cm();

  if (Sta2 == 1)
  {
    t1.setInterval(100);
    Sta2 = 0;
  }

  // Serial.printf("Sound_val: %d \n", Sound_val);
  // Serial.printf("Distance: %d cm \n", Dis);
  // Serial.printf("Microwave_STA: %d  \n", Microwave_STA);

  if (Dis < 100 && Dis != 0)
  {
    read_distance();
    t1.setInterval(1000);
    Sta2 = 1;
  }
}

void t2OTA_Check()
{
  connect_wifi();
}

void t3GASDetections()
{
  // nh3 = analogRead(nh3_pin);
    appendValueToBuffer(nh3_Sensor, analogRead(nh3_pin));
  if (!sgp.IAQmeasure())
  {
    Serial.println("Measurement failed");
    return;
  }
      appendValueToBuffer(TVOC_Sensor,sgp.TVOC);
      appendValueToBuffer(eCO2_Sensor,sgp.eCO2);
  // TVOC = sgp.TVOC;
  // eCO2 = sgp.eCO2;

  if (!sgp.IAQmeasureRaw())
  {
    Serial.println("Raw Measurement failed");
    return;
  }

  if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
  {
    Serial.println("Failed to get baseline readings");
    return;
  }
  if (Sensor_Type == BME)
  {
    temp = bme.readTemperature();
    pres = bme.readPressure() / 100.0F;
    hum = bme.readHumidity();
    appendValueToBuffer(temp_Sensor, bme.readTemperature());
    appendValueToBuffer(pres_Sensor, bme.readPressure() / 100.0F);
    appendValueToBuffer(hum_Sensor, bme.readHumidity());
    // Serial.printf("Sensor_Type == BME \n ");
  }
  if (Sensor_Type == BMP)
  {
    // temp = bmp.readTemperature();
    // pres = bmp.readPressure() / 100.0F;
    appendValueToBuffer(temp_Sensor, bmp.readTemperature());
    appendValueToBuffer(pres_Sensor, bmp.readPressure() / 100.0F);
    appendValueToBuffer(hum_Sensor, 0);
    // Serial.printf("Sensor_Type == BMP \n ");
  }

  // // Serial.printf(" ****Baseline values: eCO2: 0x%d \t & TVOC: 0x%d  \n ", (eCO2_base, HEX), (TVOC_base, HEX));
  // // Serial.printf("Raw H2 %d \t Raw Ethanol %d \n ", sgp.rawH2, sgp.rawEthanol);
  // Serial.printf(" TVOC : %d  ppb\t eCO2 : %d ppm \n ", TVOC, eCO2);
  // Serial.printf(" Pressure: %.2f \t Temperature: %f  \t Humidity: %.2f \t NH3:  %d \t noise: %d \n ", pres, temp,hum ,nh3,modbusdata);

}

void t4sendViaNBIOT()
{

  dis = sum(SensorBuffer[Ultrasonic]);
  nh3 = avg(SensorBuffer[nh3_Sensor]);
  temp = avg(SensorBuffer[temp_Sensor]);
  pres = avg(SensorBuffer[pres_Sensor]);
  hum  = avg(SensorBuffer[hum_Sensor]);
  TVOC = avg(SensorBuffer[TVOC_Sensor]);
  eCO2 = avg(SensorBuffer[eCO2_Sensor]);

  meta = AISnb.getSignal();
  Serial.print("RSSI:");
  Serial.println(meta.rssi);
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"nh3\":");
  json.concat(nh3);
  json.concat(",\"TVOC\":");
  json.concat(TVOC);
  json.concat(",\"eCO2\":");
  json.concat(eCO2);
  json.concat(",\"hum\":");
  json.concat(String(hum, 1));
  json.concat(",\"pres\":");
  json.concat(String(pres, 1));
  json.concat(",\"temp\":");
  json.concat(String(temp, 1));
  json.concat(",\"dis\":");
  json.concat(dis);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat(",\"ver\":");
  json.concat(FirmwareVer);
  json.concat("}");
  Serial.println(json);

  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  memset(SensorBuffer[Ultrasonic], 0, sizeof(SensorBuffer[Ultrasonic]));
  memset(SensorBuffer[nh3_Sensor], 0,sizeof(SensorBuffer[nh3_Sensor]));
  memset(SensorBuffer[temp_Sensor], 0, sizeof(SensorBuffer[temp_Sensor]));
  memset(SensorBuffer[pres_Sensor], 0,sizeof(SensorBuffer[pres_Sensor]));
  memset(SensorBuffer[hum_Sensor], 0,sizeof(SensorBuffer[hum_Sensor]));
  memset(SensorBuffer[TVOC_Sensor], 0,sizeof(SensorBuffer[TVOC_Sensor]));
  memset(SensorBuffer[eCO2_Sensor], 0,sizeof(SensorBuffer[eCO2_Sensor]));
}

void loop()
{
  ArduinoOTA.handle();
  runner.execute();
}