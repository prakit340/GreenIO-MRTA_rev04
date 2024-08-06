// Board MRTA Rev 0.4 addon watchdog switch

#include <Arduino.h>
#include <Update.h>
#include <RS485.h>
#include "Wire.h"
#include <SIM76xx.h>
#include <Storage.h>
#include <GSMNetwok.h>
#include <GSMClientSecure.h>
#include <GSMUdp.h>
#include "SPIFFS.h"
// #include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <MD5Builder.h>
#include "time.h"
#include "EEPROM.h"
// #include <TaskScheduler.h>
#include <ArduinoHttpClient.h>
#include <TimeLib.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
// #include "BluetoothSerial.h"

String version = "2.3";
String get_version2;

#define t1 5    // Time for t1Accu Cam
#define t2 60   // Time for t2sendtelemetry
#define t3 60   // Time for t3onlinePing
#define t4 300  // Time for t4getOTA
#define t5 300  // Time for t5reConnWiFi
#define t6 2    // Time for t6timeBlink
#define t7 300  // Time for t7timeGetSettingCamera
#define t8 1800 // Time for t8timeSetting default X

// 300 seconds WDT
#define WDT_TIMEOUT 300

unsigned long current = 0;
unsigned long previous_t1 = 0;
unsigned long previous_t2 = 0;
unsigned long previous_t3 = 0;
unsigned long previous_t4 = 0;
unsigned long previous_t5 = 0;
unsigned long previous_t6 = 0;
unsigned long previous_t7 = 0;
unsigned long previous_t8 = 0;

#define sensMB_ID 1      // Modbus env.Gas ID
#define sensitivity 8.91 // Mic sensitivity
#define ref_SPL 94       // Refference cal sound pressure level @ 1kH 94dB  //https://embedded-lab.com/blog/making-a-spl-db-meter/
#define micPIN 32        // Mic connect to Analog pin
#define ledPIN 15        // LED onboard pin
#define avgSens 12       // 12 Time @ 5 Sec/Time
#define countPing 5      // Count for offline ping to restart
#define countSend 5      // Count for fail send to restart
#define trigWDTPin 25    // Watchdog Heartbeat Pin trig every <10 min

/* Setup VS121
1. SSID -> Sensor_VS121
2. PASS -> mrta7650
3. Secu -> WPA-PSK/WPA2-PSK
4. IP -> 192.168.1.100-192.168.1.102
5. USER -> admin
6. PASS -> mrta7650
*/
// Sensor_VS121 WiFi
const char *ssid = "Sensor_VS121";
const char *ssidpass = "mrta7650";

// Sensor_VS121 Username
const char *username = "admin";
const char *password = "mrta7650";

// URL of Sensor_VS121
const char *server = "http://192.168.1.1";
const char *uri = "/vb.htm?language=ie&getppccurnumber";

String deviceToken = "";
// char thingsboardServer[] = "mqtt.thingcontrol.io";
// int PORT = 8883;

const char *HTTPserverAddress = "thingcontrol.io"; // server address
int HTTPport = 443;

// URL of FOTA
String OTAhost = "iotserv.io";
int OTAport = 443;
String OTAbin = "/thingcontrol/mrta/ota/firmware_update_rev04.bin";
long contentLength = 0;
bool isValidContentType = false;

// URL of get version for FOTA
const char *VersionServerAddress = "iotserv.io"; // server address
int VersionPort = 443;

// URL of get Type Camera
const char *CameraServerAddress = "thingcontrol.io"; // server address
int CameraPort = 443;

// EEPROM Section
unsigned int addVersion = 10;

const int timeZone = 7;
unsigned int localPort = 2390;        // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48;       // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];   // buffer to hold incoming and outgoing packets

#define READ_OTA_BUFFER (4 * 1024)

GSMUdp Udp;
// BluetoothSerial SerialBT;

// Scheduler runner;
// GSMClientSecure gsm_client;
// PubSubClient client(gsm_client);

// void t1AccuAVG();
// void t2sendtelemetry();
// void t3onlinePing();
// void t4getOTA();
// void t5reConnWiFi();

// Task t1(5000, TASK_FOREVER, &t1AccuAVG);
// Task t2(60000, TASK_FOREVER, &t2sendtelemetry);
// Task t3(60000, TASK_FOREVER, &t3onlinePing);
// Task t4(300000, TASK_FOREVER, &t4getOTA);
// Task t5(300000, TASK_FOREVER, &t5reConnWiFi);

IPAddress ip;
String ccid = "";
String imsi = "";
String imei = "";
char filename[] = "000000000.CSV";

uint16_t mb_buffer[7];

int count_ping = 0;
int count_send = 0;
int count = 0;
int count_acc = 0;

// AVGsensorAll
long avg_sens_CO2;
int avg_sens_CH2O;
int avg_sens_TVOC;
int avg_sens_PM25;
int avg_sens_PM10;
float avg_sens_Temp;
float avg_sens_Humi;
float avg_sens_dB = 33.3;
unsigned long avg_sens_People;
unsigned long avg_sens_People_In;
unsigned long avg_sens_People_Out;

// ACCsensorAll
unsigned long acc_sens_CO2;
unsigned long acc_sens_CH2O;
unsigned long acc_sens_TVOC;
unsigned long acc_sens_PM25;
unsigned long acc_sens_PM10;
int acc_sens_Temp;
int acc_sens_Humi;
double acc_sens_dB;
unsigned long acc_sens_People;
unsigned long acc_sens_People_In;
unsigned long acc_sens_People_Out;

// readSensor
int sens_CO2;
int sens_CH2O;
int sens_TVOC;
int sens_PM25;
int sens_PM10;
float sens_Temp;
float sens_Humi;
float sens_dB;
int sens_People;
int sens_People_In;
int sens_People_Out;

bool sensErr = 0;
bool camType = 0;
int defaultX = 0;

void printDEBUG(void);
void setCounterCount(float startX, float startY, float endX, float endY, float interval);
void getCounterCount(void);

//****************************************************************//
//*************** Trig watchdog protection reset *****************//
//****************************************************************//
void heartbeat()
{
  // Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  // Led monitor foe Heartbeat
  // digitalWrite(ledHeartPIN, LOW);
  delay(300);
  // digitalWrite(ledHeartPIN, HIGH);

  // Return to high-Z
  pinMode(trigWDTPin, INPUT);

  Serial.println("Heartbeat sent");
}

//****************************************************//
//************* MQTT Callback Function ***************//
//****************************************************//
// void callback(char *topic, byte *payload, unsigned int length)
// {
//   Serial.print("Message arrived [");
//   Serial.print(topic);
//   Serial.print("] ");
//   for (int i = 0; i < length; i++)
//   {
//     Serial.print((char)payload[i]);
//   }
//   Serial.println();
// }

//**************************************************************************//
//**** Func Utility to extract header value from headers for OTA ***********//
//**************************************************************************//
String getHeaderValue(String header, String headerName)
{
  return header.substring(strlen(headerName.c_str()));
}

//****************************************************//
//***************** Excecute FOTA ********************//
//****************************************************//
void execOTA()
{
  GSMClientSecure clientExeOta;
  clientExeOta.setInsecure();

  Serial.println("Connecting to: " + String(OTAhost));
  // SerialBT.println("Connecting to: " + String(OTAhost));

  // Connect to S3
  if (clientExeOta.connect(OTAhost.c_str(), OTAport))
  {
    // Connection Succeed.
    // Fecthing the bin
    Serial.println("Fetching Bin: " + String(OTAbin));
    // SerialBT.println("Fetching Bin: " + String(OTAbin));

    // Get the contents of the bin file
    clientExeOta.print(String("GET ") + OTAbin + " HTTP/1.1\r\n" +
                       "Host: " + OTAhost + "\r\n" +
                       "Cache-Control: no-cache\r\n" +
                       "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (clientExeOta.available() == 0)
    {
      if (millis() - timeout > 5000)
      {
        Serial.println(F("Client OTA Timeout !"));
        // SerialBT.println(F("Client OTA Timeout !"));
        clientExeOta.stop();
        return;
      }
    }
    // Once the response is available,
    // check stuff

    /*
       Response Structure
        HTTP/1.1 200 OK
        x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
        x-amz-request-id: 2D56B47560B764EC
        Date: Wed, 14 Jun 2017 03:33:59 GMT
        Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
        ETag: "d2afebbaaebc38cd669ce36727152af9"
        Accept-Ranges: bytes
        Content-Type: application/octet-stream
        Content-Length: 357280
        Server: AmazonS3

        {{BIN FILE CONTENTS}}
    */
    while (clientExeOta.available())
    {
      // read line till /n
      String line = clientExeOta.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      // if the the line is empty,
      // this is end of headers
      // break the while and feed the
      // remaining `client` to the
      // Update.writeStream();
      if (!line.length())
      {
        // headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1"))
      {
        if (line.indexOf("200") < 0)
        {
          Serial.println(F("Got a non 200 status code from server. Exiting OTA Update."));
          // SerialBT.println(F("Got a non 200 status code from server. Exiting OTA Update."));
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: "))
      {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
        // SerialBT.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: "))
      {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        // SerialBT.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream")
        {
          isValidContentType = true;
        }
      }
    }
  }
  else
  {
    // Connect to S3 failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(OTAhost) + " failed. Please check your setup");
    // SerialBT.println("Connection to " + String(OTAhost) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));
  // SerialBT.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType)
  {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin)
    {
      Serial.println(F("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!"));
      // SerialBT.println(F("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!"));
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      uint32_t written = 0;
      uint8_t buffer[READ_OTA_BUFFER];
      while (1)
      {
        int read_bytes = clientExeOta.readBytes(buffer, min((uint32_t)READ_OTA_BUFFER, (uint32_t)(contentLength - written)));
        if (read_bytes == 0)
        {
          break;
        }
        Update.write(buffer, read_bytes);
        written += read_bytes;
        Serial.printf("Write %.02f%% (%ld/%ld)\n", (float)written / (float)contentLength * 100.0, written, contentLength);
        // SerialBT.printf("Write %.02f%% (%ld/%ld)\n", (float)written / (float)contentLength * 100.0, written, contentLength);
        if (written == contentLength)
        {
          break;
        }
      }

      if (written == contentLength)
      {
        Serial.println("Written : " + String(written) + " successfully");
        // SerialBT.println("Written : " + String(written) + " successfully");
      }
      else
      {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
        // SerialBT.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
        // retry??
        // execOTA();
      }

      if (Update.end())
      {
        Serial.println(F("OTA done!"));
        if (Update.isFinished())
        {
          EEPROM.writeString(addVersion, get_version2);
          EEPROM.commit();

          Serial.println(F("Update successfully completed. Rebooting."));
          Serial.println(F("Shutting Down GSM."));
          // SerialBT.println(F("Update successfully completed. Rebooting."));
          // SerialBT.println(F("Shutting Down GSM."));
          GSM.shutdown();
          for (int a = 0; a < 10; a++)
          {
            GSM.shutdown();
            delay(1000);
          }
          GSM.shutdown();
          ESP.restart();
        }
        else
        {
          Serial.println(F("Update not finished? Something went wrong!"));
          // SerialBT.println(F("Update not finished? Something went wrong!"));
        }
      }
      else
      {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        // SerialBT.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println(F("Not enough space to begin OTA"));
      // SerialBT.println(F("Not enough space to begin OTA"));
      clientExeOta.flush();
    }
  }
  else
  {
    Serial.println(F("There was no content in the response"));
    // SerialBT.println(F("There was no content in the response"));
    clientExeOta.flush();
  }
}

//****************************************************//
//******** Get People count type Cross ***************//
//****************************************************//
void getPeopleCross()
{
  Serial.println();
  Serial.println(F("making GET request People count region"));
  // SerialBT.println();
  // SerialBT.println(F("making GET request People count region"));

  WiFiClient ClientPeopleCountCross;
  HTTPClient clientPeopleCross;

  clientPeopleCross.begin(ClientPeopleCountCross, "http://admin:mrta7650@192.168.1.1/vb.htm?getcountercount");

  // read the status code and body of the response
  int statusCode = clientPeopleCross.GET();

  if (statusCode > 0)
  {
    String payload = clientPeopleCross.getString();
    Serial.println(payload);

    int In = 0;
    int Out = 0;

    // OK incount=5,outcount=3
    int charOne = payload.indexOf("incount");
    int charTwo = payload.indexOf(",");
    int charThree = payload.indexOf("outcount");

    In = payload.substring(charOne + 8, charTwo).toInt();
    Out = payload.substring(charThree + 9).toInt();

    Serial.print(F("In : "));
    Serial.println(In);
    Serial.print(F("Out : "));
    Serial.println(Out);
    // SerialBT.print(F("In : "));
    // SerialBT.println(In);
    // SerialBT.print(F("Out : "));
    // SerialBT.println(Out);

    sens_People_In = In;
    sens_People_Out = Out;
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", clientPeopleCross.errorToString(statusCode).c_str());
    // SerialBT.printf("[HTTP] GET... failed, error: %s\n", clientPeopleCross.errorToString(statusCode).c_str());
  }

  clientPeopleCross.end();
}

//****************************************************//
//******** Get People count type Region ***************//
//****************************************************//
void getPeopleRegion()
{
  Serial.println();
  Serial.println(F("making GET request People count region"));
  // SerialBT.println();
  // SerialBT.println(F("making GET request People count region"));

  WiFiClient ClientPeopleCount;
  HTTPClient clientPeople;

  clientPeople.begin(ClientPeopleCount, "http://admin:mrta7650@192.168.1.1/vb.htm?getppccurnumber");

  // read the status code and body of the response
  int statusCode = clientPeople.GET();

  if (statusCode > 0)
  {
    String payload = clientPeople.getString();
    Serial.println(payload);
    // SerialBT.println(payload);

    // OK getppccurnumber=3
    int charOne = payload.indexOf("getppccurnumber");
    payload = payload.substring(charOne + 16);
    Serial.print(F("Count : "));
    Serial.println(payload.toInt());
    // SerialBT.print(F("Count : "));
    // SerialBT.println(payload.toInt());
    count = payload.toInt();
    sens_People = count;
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", clientPeople.errorToString(statusCode).c_str());
    // SerialBT.printf("[HTTP] GET... failed, error: %s\n", clientPeople.errorToString(statusCode).c_str());
  }

  clientPeople.end();
}

//****************************************************//
//******************* Get Camera type ****************//
//****************************************************//
void t7getCameraType()
{
  StaticJsonDocument<200> doc;

  Serial.println();
  Serial.println(F("making GET request Camera type and setting"));
  // SerialBT.println();
  // SerialBT.println(F("making GET request Camera type and setting"));

  GSMClientSecure gsmClientCameraType;
  gsmClientCameraType.setInsecure(); // ignore CA check

  String pathCCID = "";
  pathCCID += "/api/v1/";
  pathCCID += ccid;
  pathCCID += "/attributes";
  Serial.println(pathCCID);
  // SerialBT.println(pathCCID);

  HttpClient clientCameraType = HttpClient(gsmClientCameraType, CameraServerAddress, CameraPort);
  clientCameraType.get(pathCCID); // endpoint

  // read the status code and body of the response
  int statusCode = clientCameraType.responseStatusCode();
  String response = clientCameraType.responseBody();

  clientCameraType.stop();

  Serial.print(F("Status code get camera type: "));
  Serial.println(statusCode);
  Serial.print(F("Response get camera type: "));
  Serial.println(response);
  // SerialBT.print(F("Status code get camera type: "));
  // SerialBT.println(statusCode);
  // SerialBT.print(F("Response get camera type: "));
  // SerialBT.println(response);

  if (statusCode != 200)
  {
    Serial.println(F("Not recieve Camera Type and setting"));
    // SerialBT.println(F("Not recieve Camera Type and setting"));
    return;
  }

  DeserializationError error = deserializeJson(doc, response);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    // SerialBT.print(F("deserializeJson() failed: "));
    // SerialBT.println(error.f_str());
    return;
  }

  camType = doc["counting"];
  defaultX = doc["defaultX"];

  // Print values.
  Serial.println(F("===================================="));
  Serial.print(F("Get API for CAM type : "));
  Serial.println(camType);
  Serial.print(F("Get API for Setting cross report interval : "));
  Serial.println(defaultX);
  Serial.println(F("===================================="));
  // SerialBT.println(F("===================================="));
  // SerialBT.print(F("Get API for CAM type : "));
  // SerialBT.println(camType);
  // SerialBT.print(F("Get API for Setting cross report interval : "));
  // SerialBT.println(defaultX);
  // SerialBT.println(F("===================================="));
}

//****************************************************//
//**************** Get Version to OTA ****************//
//****************************************************//
void t4getOTA()
{
  StaticJsonDocument<200> doc;

  Serial.println();
  Serial.println(F("making GET request version"));
  // SerialBT.println();
  // SerialBT.println(F("making GET request version"));

  GSMClientSecure gsmClientOta;
  gsmClientOta.setInsecure(); // ignore CA check

  HttpClient clientOTA = HttpClient(gsmClientOta, VersionServerAddress, VersionPort);
  clientOTA.get("/thingcontrol/mrta/get-version.php?idsite=2"); // endpoint

  // read the status code and body of the response
  int statusCode = clientOTA.responseStatusCode();
  String response = clientOTA.responseBody();

  clientOTA.stop();

  Serial.print(F("Status code get version: "));
  Serial.println(statusCode);
  Serial.print(F("Response get version: "));
  Serial.println(response);
  // SerialBT.print(F("Status code get version: "));
  // SerialBT.println(statusCode);
  // SerialBT.print(F("Response get version: "));
  // SerialBT.println(response);

  if (statusCode != 200)
  {
    Serial.println(F("Not recieve Version for OTA."));
    // SerialBT.println(F("Not recieve Version for OTA."));
    return;
  }

  DeserializationError error = deserializeJson(doc, response);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    // SerialBT.print(F("deserializeJson() failed: "));
    // SerialBT.println(error.f_str());
    return;
  }

  String site = doc["site_id"];
  String get_filename = doc["filename"];
  String get_version = doc["version"];
  String get_time = doc["time"];
  get_version2 = get_version;

  // Print values.
  Serial.println(F("===================================="));
  Serial.print(F("Get API for FOTA Site: "));
  Serial.println(site);
  Serial.print(F("Filename: "));
  Serial.println(get_filename);
  Serial.print(F("Version: "));
  Serial.println(get_version);

  Serial.print(F("Previous version: "));
  Serial.println(version);

  Serial.print(F("Time: "));
  Serial.println(get_time);

  // SerialBT.println(F("===================================="));
  // SerialBT.print(F("Get API for FOTA Site: "));
  // SerialBT.println(site);
  // SerialBT.print(F("Filename: "));
  // SerialBT.println(get_filename);
  // SerialBT.print(F("Version: "));
  // SerialBT.println(get_version);

  // SerialBT.print(F("Previous version: "));
  // SerialBT.println(version);

  // SerialBT.print(F("Time: "));
  // SerialBT.println(get_time);

  if (version != get_version)
  {
    Serial.println(F("!! Update Version Start FOTA !!"));
    // SerialBT.println(F("!! Update Version Start FOTA !!"));
    execOTA();
  }
  Serial.println(F("===================================="));
  // SerialBT.println(F("===================================="));
}

//****************************************************//
//*********** Get File name with NTP time ************//
//****************************************************//
void getFilename(char *filename)
{
  int SDYear = year();
  int SDMonth = month();
  int SDDay = day();
  filename[0] = '/';
  filename[1] = SDDay / 10 + '0';
  filename[2] = SDDay % 10 + '0';
  filename[3] = '-';
  filename[4] = SDMonth / 10 + '0';
  filename[5] = SDMonth % 10 + '0';
  filename[6] = '-';
  filename[7] = (SDYear - 2000) / 10 + '0';
  filename[8] = SDYear % 10 + '0';
  filename[9] = '.';
  filename[10] = 'C';
  filename[11] = 'S';
  filename[12] = 'V';
  return;
}

//****************************************************//
//************** Save data to SD Card ****************//
//****************************************************//
void saveSD()
{
  int sHour = 0;
  int sMinute = 0;
  int sSecond = 0;
  String message = "";
  String filenameStr = "";

  sHour = hour();
  sMinute = minute();
  sSecond = second();

  getFilename(filename);

  message = "\n";
  if (sHour < 10)
  {
    message += "0";
  }
  message += sHour;
  message += ":";
  if (sMinute < 10)
  {
    message += "0";
  }
  message += sMinute;
  message += ":";
  if (sSecond < 10)
  {
    message += "0";
  }
  message += sSecond;
  message += ",";
  message += deviceToken.c_str();
  message += ",";
  message += avg_sens_CO2;
  message += ",";
  message += avg_sens_CH2O;
  message += ",";
  message += avg_sens_TVOC;
  message += ",";
  message += avg_sens_PM25;
  message += ",";
  message += avg_sens_PM10;
  message += ",";
  message += avg_sens_Temp;
  message += ",";
  message += avg_sens_Humi;
  message += ",";
  message += avg_sens_dB;
  message += ",";
  message += avg_sens_People;
  message += ",";
  message += version;
  message += ",";
  message += ip;
  message += ",";
  message += Network.getSignalStrength();

  // appendFile(SD, filename, message.c_str());

  filenameStr = "D:/"; // D: is MicroSD Card on sim7600
  filenameStr += filename;

  bool write_ok = Storage.fileWrite(filenameStr, message);
  if (!write_ok)
  {
    Serial.println(F("Write file error !"));
    // SerialBT.println(F("Write file error !"));
  }
}

//****************************************************************//
//************ HTTP Auth Extract para VS121 Function *************//
//****************************************************************//
String exractParam(String &authReq, const String &param, const char delimit)
{
  int _begin = authReq.indexOf(param);
  if (_begin == -1)
  {
    return "";
  }
  return authReq.substring(_begin + param.length(), authReq.indexOf(delimit, _begin + param.length()));
}

//****************************************************************//
//************* HTTP Auth Get C Nonce VS121 Function *************//
//****************************************************************//
String getCNonce(const int len)
{
  static const char alphanum[] = "0123456789"
                                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                 "abcdefghijklmnopqrstuvwxyz";
  String s = "";

  for (int i = 0; i < len; ++i)
  {
    s += alphanum[rand() % (sizeof(alphanum) - 1)];
  }

  return s;
}

//****************************************************************//
//************* HTTP type Digest Auth VS121 Function *************//
//****************************************************************//
String getDigestAuth(String &authReq, const String &username, const String &password, const String &method, const String &uri, unsigned int counter)
{
  // extracting required parameters for RFC 2069 simpler Digest
  String realm = exractParam(authReq, "realm=\"", '"');
  String nonce = exractParam(authReq, "nonce=\"", '"');
  String cNonce = getCNonce(8);

  char nc[9];
  snprintf(nc, sizeof(nc), "%08x", counter);

  // parameters for the RFC 2617 newer Digest
  MD5Builder md5;
  md5.begin();
  md5.add(username + ":" + realm + ":" + password); // md5 of the user:realm:user
  md5.calculate();
  String h1 = md5.toString();

  md5.begin();
  md5.add(method + ":" + uri);
  md5.calculate();
  String h2 = md5.toString();

  md5.begin();
  md5.add(h1 + ":" + nonce + ":" + String(nc) + ":" + cNonce + ":" + "auth" + ":" + h2);
  md5.calculate();
  String response = md5.toString();

  String authorization = "Digest username=\"" + username + "\", realm=\"" + realm + "\", nonce=\"" + nonce + "\", uri=\"" + uri + "\", algorithm=\"MD5\", qop=auth, nc=" + String(nc) + ", cnonce=\"" + cNonce + "\", response=\"" + response + "\"";
  // Serial.println(authorization);

  return authorization;
}

//************************************************************************//
//************** Count person with get http VS121 Function ***************//
//************************************************************************//
void getPeopleCount()
{
  WiFiClient client;
  HTTPClient http; // must be declared after WiFiClient for correct destruction order, because used by http.begin(client,...)

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.print(F("[HTTP] begin...\n"));
  // SerialBT.println(F(""));
  // SerialBT.println(F(""));
  // SerialBT.print(F("[HTTP] begin...\n"));

  // configure traged server and url
  http.begin(client, String(server) + String(uri));

  const char *keys[] = {"WWW-Authenticate"};
  http.collectHeaders(keys, 1);

  Serial.print(F("[HTTP] GET...\n"));
  // SerialBT.print(F("[HTTP] GET...\n"));
  // start connection and send HTTP header
  int httpCode = http.GET();

  if (httpCode > 0)
  {
    String authReq = http.header("WWW-Authenticate");

    String authorization = getDigestAuth(authReq, String(username), String(password), "GET", String(uri), 1);

    http.end();
    http.begin(client, String(server) + String(uri));

    http.addHeader("Authorization", authorization);

    int httpCode = http.GET();
    if (httpCode > 0)
    {
      String payload = http.getString();
      payload = payload.substring(30);
      Serial.print(F("Count : "));
      Serial.println(payload.toInt());
      // SerialBT.print(F("Count : "));
      // SerialBT.println(payload.toInt());
      count = payload.toInt();
      sens_People = count;
    }
    else
    {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      // SerialBT.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    // SerialBT.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

//****************************************************//
//************** Connect MQTT Function ***************//
//****************************************************//
// void reconnectMqtt()
// {
//   if (client.connect(imei.c_str(), deviceToken.c_str(), NULL))
//   {
//     Serial.println(F("Connect MQTT Success."));
//     client.subscribe("v1/devices/me/rpc/request/+");
//     digitalWrite(ledPIN, LOW);
//   }
//   else
//   {
//     digitalWrite(ledPIN, HIGH);
//     Serial.print("failed, rc=");
//     Serial.print(client.state());
//     Serial.println(" try again in 5 seconds");
//     // Wait 5 seconds before retrying
//     delay(5000);
//   }
// }

//****************************************************//
//*************** HTTPS Post Function ****************//
//****************************************************//
void sendHTTPSpost(char jsonTele[])
{
  Serial.println(F("Making POST request"));
  // SerialBT.println(F("Making POST request"));
  String contentType = "application/json";

  String api = "/api/v1/";
  api += deviceToken.c_str();
  api += "/telemetry";

  char *aString = jsonTele;

  GSMClientSecure gsmClientPost;
  // gsmClientPost.setCACert(root_digi_cert); // Set CA Certificate
  gsmClientPost.setInsecure(); // ignore CA check

  HttpClient clientPost = HttpClient(gsmClientPost, HTTPserverAddress, HTTPport);
  clientPost.post(api, contentType, aString);

  // read the status code and body of the response
  int statusCode = clientPost.responseStatusCode();
  String response = clientPost.responseBody();

  clientPost.stop();
  // delay(1000);

  Serial.print(F("Status code: "));
  Serial.println(statusCode);
  // SerialBT.print(F("Status code: "));
  // SerialBT.println(statusCode);
  if (statusCode != 200)
  {
    count_send++;
    Serial.println(F("===================================="));
    Serial.print(F("Send with HTTPS not success count to RST: "));
    Serial.print(count_send);
    Serial.print(F("/"));
    Serial.println(countSend);
    Serial.println(F("===================================="));
    // SerialBT.println(F("===================================="));
    // SerialBT.print(F("Send with HTTPS not success count to RST: "));
    // SerialBT.print(count_send);
    // SerialBT.print(F("/"));
    // SerialBT.println(countSend);
    // SerialBT.println(F("===================================="));
    if (count_send >= countSend)
    {
      count_send = 0;
      Serial.println(F("Shutting Down GSM."));
      // SerialBT.println(F("Shutting Down GSM."));
      GSM.shutdown();
      for (int a = 0; a < 10; a++)
      {
        GSM.shutdown();
        delay(1000);
      }
      ESP.restart();
    }
    digitalWrite(ledPIN, HIGH);
  }
  else
  {
    count_send = 0;
    Serial.println(F("Send with HTTPS successed"));
    // SerialBT.println(F("Send with HTTPS successed"));
    digitalWrite(ledPIN, LOW);
  }
  Serial.print(F("Response: "));
  Serial.println(response);
  Serial.println(F("===================================="));
  // SerialBT.print(F("Response: "));
  // SerialBT.println(response);
  // SerialBT.println(F("===================================="));
}

//****************************************************//
//****** Send with MQTT or HTTPS Post Function *******//
//****************************************************//
void processTele(char jsonTele[])
{
  Serial.println(F("============== Send ================"));
  // SerialBT.println(F("============== Send ================"));
  char *aString = jsonTele;
  Serial.println(F("OK"));
  Serial.print(F("+:topic v1/devices/me/ , "));
  Serial.println(aString);
  // SerialBT.println(F("OK"));
  // SerialBT.print(F("+:topic v1/devices/me/ , "));
  // SerialBT.println(aString);
  digitalWrite(ledPIN, HIGH);
  delay(80);
  digitalWrite(ledPIN, LOW);
  // client.publish("v1/devices/me/telemetry", aString);
  sendHTTPSpost(aString);
}

//****************************************************//
//*********** Read Modbus Sensor Function ************//
//****************************************************//
void readModbus()
{
  // Read Modbus ENV _ M701 RS485
  delay(200);
  for (int a = 1; a <= 7; a++)
  {
    // delay(200);
    mb_buffer[a - 1] = RS485.holdingRegisterRead(sensMB_ID, a * 2);
    delay(1000);
  }
}

//****************************************************//
//************* readSensor Function ******************//
//****************************************************//
void readSensor()
{
  readModbus();

  if ((mb_buffer[0] > 30000) || (mb_buffer[1] > 30000) || (mb_buffer[2] > 30000) || (mb_buffer[3] > 30000) || (mb_buffer[4] > 30000) || (mb_buffer[5] > 30000) || (mb_buffer[6] > 30000))
  {
    sensErr = 1;
    Serial.println(F(""));
    Serial.println(F("===================================="));
    Serial.println(F("Modbus not stable"));
    // SerialBT.println(F(""));
    // SerialBT.println(F("===================================="));
    // SerialBT.println(F("Modbus not stable"));
    readModbus();
  }
  else
  {
    sensErr = 0;
    Serial.println(F(""));
    Serial.println(F("===================================="));
    Serial.println(F("Modbus OK"));
    // SerialBT.println(F(""));
    // SerialBT.println(F("===================================="));
    // SerialBT.println(F("Modbus OK"));
    sens_CO2 = mb_buffer[0];
    sens_CH2O = mb_buffer[1];
    sens_TVOC = mb_buffer[2];
    sens_PM25 = mb_buffer[3];
    sens_PM10 = mb_buffer[4];
    sens_Temp = mb_buffer[5] * 0.1;
    sens_Humi = mb_buffer[6] * 0.1;
  }
}

//****************************************************//
//***** Sort json parameter for send Function ********//
//****************************************************//
void t2sendtelemetry()
{
  readSensor();
  printDEBUG();

  Serial.println(F(""));
  // SerialBT.println(F(""));
  if (sens_Temp != 0)
  {
    String json = "";
    json.concat("{\"tn\":\"");
    json.concat(deviceToken.c_str());
    json.concat("\",\"co2\":");
    json.concat(sens_CO2);
    json.concat(",\"ch2o\":");
    json.concat(sens_CH2O);
    json.concat(",\"tvoc\":");
    json.concat(sens_TVOC);
    json.concat(",\"pm25\":");
    json.concat(sens_PM25);
    json.concat(",\"pm10\":");
    json.concat(sens_PM10);
    json.concat(",\"temp\":");
    json.concat(String(sens_Temp, 2));
    json.concat(",\"humi\":");
    json.concat(String(sens_Humi, 2));
    json.concat(",\"noise\":");
    json.concat(String(avg_sens_dB, 2));
    json.concat(",\"person_acc_1min\":"); // person_acc_1min
    json.concat(avg_sens_People);
    json.concat(",\"person_cur_1min\":"); // person_cur_1min
    json.concat(acc_sens_People);
    json.concat(",\"version\":"); // Version
    json.concat(version);
    json.concat(",\"ip\":");
    json.concat(String(ip));
    json.concat(",\"rssi\":");
    json.concat(Network.getSignalStrength());
    json.concat(",\"FreeHeap\":"); // FreeHeap
    json.concat(ESP.getFreeHeap());
    json.concat(",\"SensError\":"); // SensError
    json.concat(sensErr);
    json.concat(",\"counting\":"); // counting
    json.concat(camType);
    json.concat(",\"defaultX\":"); // defaultX
    json.concat(defaultX);
    json.concat(",\"in_person_cross_cur_1min\":"); // in_person_cross_cur_1min
    json.concat(sens_People_In);
    json.concat(",\"out_person_cross_cur_1min\":"); // out_person_cross_cur_1min
    json.concat(sens_People_Out);
    json.concat("}");
    Serial.println(json);
    // SerialBT.println(json);

    int str_len = json.length() + 1;
    char char_array[str_len];

    json.toCharArray(char_array, str_len);
    processTele(char_array);
  }
  // saveSD();
}

//****************************************************//
//************* Read MIC Analog Function *************//
//****************************************************//
float readMic_ANA() // https://embedded-lab.com/blog/making-a-spl-db-meter/
{
  unsigned long long tmp = 0;
  float rms = 0.0;

  for (int a = 0; a < 20; a++)
  {
    tmp += analogRead(micPIN);
    delay(1);
  }
  rms = tmp / 20;
  tmp = 0;
  rms = rms * (3.3 / 4095.0);

  if (rms <= 0)
  {
    rms = 3.3 / 4095.0;
  }
  return rms;
}

//****************************************************//
//*************** Cal MIC dB Function ****************//
//****************************************************//
void read_SPL()
{
  float dB_current = 0.0;

  dB_current = readMic_ANA();
  sens_dB = (ref_SPL + 20 * log10(dB_current / sensitivity));
}

//****************************************************//
//*************** Debug Function *********************//
//****************************************************//
void printDEBUG()
{
  Serial.println(F(""));
  Serial.println(F("===================================="));
  Serial.print(F("CO2: "));
  Serial.print(sens_CO2);
  Serial.println(F(" ppm"));
  Serial.print(F("CH2O: "));
  Serial.print(sens_CH2O);
  Serial.println(F(" ug/m3"));
  Serial.print(F("TVOC: "));
  Serial.print(sens_TVOC);
  Serial.println(F(" ug/m3"));
  Serial.print(F("PM2.5: "));
  Serial.print(sens_PM25);
  Serial.println(F(" ug/m3"));
  Serial.print(F("PM10: "));
  Serial.print(sens_PM10);
  Serial.println(F(" ug/m3"));
  Serial.print(F("TEMP: "));
  Serial.print(sens_Temp);
  Serial.println(F(" ํC"));
  Serial.print(F("HUMI: "));
  Serial.print(sens_Humi);
  Serial.println(F(" %"));
  Serial.print(F("Sound: "));
  Serial.print(sens_dB);
  Serial.println(F(" dB"));
  Serial.print(F("1-Min People count: "));
  Serial.print(acc_sens_People);
  Serial.println(F(" Person"));
  Serial.println(F("===================================="));

  Serial.println(F("=============== AVG ================"));
  // Serial.print(F("AVG CO2: "));
  // Serial.print(avg_sens_CO2);
  // Serial.println(F(" ppm"));
  // Serial.print(F("AVG CH2O: "));
  // Serial.print(avg_sens_CH2O);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("AVG TVOC: "));
  // Serial.print(avg_sens_TVOC);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("AVG PM2.5: "));
  // Serial.print(avg_sens_PM25);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("AVG PM10: "));
  // Serial.print(avg_sens_PM10);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("AVG TEMP: "));
  // Serial.print(avg_sens_Temp);
  // Serial.println(F(" ํC"));
  // Serial.print(F("AVG HUMI: "));
  // Serial.print(avg_sens_Humi);
  // Serial.println(F(" %"));
  Serial.print(F("AVG Sound: "));
  Serial.print(avg_sens_dB);
  Serial.println(F(" dB"));
  Serial.print(F("1-Min ACC People count: "));
  Serial.print(avg_sens_People);
  Serial.println(F(" Person"));
  Serial.println(F("===================================="));

  Serial.println(F("=============== TIME ==============="));
  Serial.print(hour());
  Serial.print(F(":"));
  Serial.print(minute());
  Serial.print(F(":"));
  Serial.print(second());
  Serial.print(F(" "));
  Serial.print(day());
  Serial.print(F("-"));
  Serial.print(month());
  Serial.print(F("-"));
  Serial.println(year());
  Serial.println(F("===================================="));

  //////////////// Bluetooth
  // SerialBT.println(F(""));
  // SerialBT.println(F("===================================="));
  // SerialBT.print(F("CO2: "));
  // SerialBT.print(sens_CO2);
  // SerialBT.println(F(" ppm"));
  // SerialBT.print(F("CH2O: "));
  // SerialBT.print(sens_CH2O);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("TVOC: "));
  // SerialBT.print(sens_TVOC);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("PM2.5: "));
  // SerialBT.print(sens_PM25);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("PM10: "));
  // SerialBT.print(sens_PM10);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("TEMP: "));
  // SerialBT.print(sens_Temp);
  // SerialBT.println(F(" ํC"));
  // SerialBT.print(F("HUMI: "));
  // SerialBT.print(sens_Humi);
  // SerialBT.println(F(" %"));
  // SerialBT.print(F("Sound: "));
  // SerialBT.print(sens_dB);
  // SerialBT.println(F(" dB"));
  // SerialBT.print(F("1-Min People count: "));
  // SerialBT.print(acc_sens_People);
  // SerialBT.println(F(" Person"));
  // SerialBT.println(F("===================================="));

  // SerialBT.println(F("=============== AVG ================"));
  // SerialBT.print(F("AVG CO2: "));
  // SerialBT.print(avg_sens_CO2);
  // SerialBT.println(F(" ppm"));
  // SerialBT.print(F("AVG CH2O: "));
  // SerialBT.print(avg_sens_CH2O);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("AVG TVOC: "));
  // SerialBT.print(avg_sens_TVOC);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("AVG PM2.5: "));
  // SerialBT.print(avg_sens_PM25);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("AVG PM10: "));
  // SerialBT.print(avg_sens_PM10);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("AVG TEMP: "));
  // SerialBT.print(avg_sens_Temp);
  // SerialBT.println(F(" ํC"));
  // SerialBT.print(F("AVG HUMI: "));
  // SerialBT.print(avg_sens_Humi);
  // SerialBT.println(F(" %"));
  // SerialBT.print(F("AVG Sound: "));
  // SerialBT.print(avg_sens_dB);
  // SerialBT.println(F(" dB"));
  // SerialBT.print(F("1-Min ACC People count: "));
  // SerialBT.print(avg_sens_People);
  // SerialBT.println(F(" Person"));
  // SerialBT.println(F("===================================="));

  // SerialBT.println(F("=============== TIME ==============="));
  // SerialBT.print(hour());
  // SerialBT.print(F(":"));
  // SerialBT.print(minute());
  // SerialBT.print(F(":"));
  // SerialBT.print(second());
  // SerialBT.print(F(" "));
  // SerialBT.print(day());
  // SerialBT.print(F("-"));
  // SerialBT.print(month());
  // SerialBT.print(F("-"));
  // SerialBT.println(year());
  // SerialBT.println(F("===================================="));
}

//****************************************************//
//**************** Connect WiFi VS121 ****************//
//****************************************************//
void t5reConnWiFi()
{
  int countWiFiconn = 0;

  Serial.print(F("Connect WiFi VS121 .."));
  // SerialBT.print(F("Connect WiFi VS121 .."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, ssidpass);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    // SerialBT.print(".");
    digitalWrite(ledPIN, HIGH);
    delay(500);
    digitalWrite(ledPIN, LOW);
    countWiFiconn++;
    if (countWiFiconn > 12)
    {
      Serial.println(F("Connect WiFi VS121.. Not connect"));
      Serial.println(F(""));
      // SerialBT.println(F("Connect WiFi VS121.. Not connect"));
      // SerialBT.println(F(""));
      break;
    }
  }
  delay(1000);
  Serial.println(F("Connect WiFi VS121.. Success"));
  Serial.println(F(""));
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  // SerialBT.println(F("Connect WiFi VS121.. Success"));
  // SerialBT.println(F(""));
  // SerialBT.println(F("WiFi connected"));
  // SerialBT.println(F("IP address: "));
  // SerialBT.println(WiFi.localIP());
}

//***********************************************************//
//*********** Read Senson Accumulate for Average ************//
//***********************************************************//
void t1AccuAVG()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    // getPeopleCount();

    if (camType == 1)
    {
      // Serial.println(F(""));
      // Serial.println(F("!! Set Reporting Interval to 300 S !!"));
      //  setCounterCount(161.4035087719,0,161.4035087719,243.75,120);
      //  getCounterCount();

      Serial.println(F(""));
      Serial.println(F("!! Camera Count type of Cross In-Out !!"));
      // SerialBT.println(F(""));
      // SerialBT.println(F("!! Camera Count type of Cross In-Out !!"));
      getPeopleCross();
    }
    else
    {
      Serial.println(F(""));
      Serial.println(F("!! Camera Count type of Region !!"));
      // SerialBT.println(F(""));
      // SerialBT.println(F("!! Camera Count type of Region !!"));
      getPeopleRegion();
    }
  }

  // readSensor();
  read_SPL();

  count_acc++;

  // acc_sens_CO2 += sens_CO2;
  // acc_sens_CH2O += sens_CH2O;
  // acc_sens_TVOC += sens_TVOC;
  // acc_sens_PM25 += sens_PM25;
  // acc_sens_PM10 += sens_PM10;
  // acc_sens_Temp += sens_Temp;
  // acc_sens_Humi += sens_Humi;
  acc_sens_dB += sens_dB;
  acc_sens_People += sens_People; // Accumulate every 5 Sec.  acc_sens.People + Count

  Serial.println(F(""));
  Serial.println(F("===================================="));
  Serial.print(F("Sensor accumurate for AVG Count = "));
  Serial.print(count_acc);
  Serial.print(F("/"));
  Serial.println(avgSens);
  Serial.println(F("===================================="));

  Serial.println(F("===================================="));
  // Serial.print(F("CO2: "));
  // Serial.print(sens_CO2);
  // Serial.println(F(" ppm"));
  // Serial.print(F("CH2O: "));
  // Serial.print(sens_CH2O);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("TVOC: "));
  // Serial.print(sens_TVOC);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("PM2.5: "));
  // Serial.print(sens_PM25);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("PM10: "));
  // Serial.print(sens_PM10);
  // Serial.println(F(" ug/m3"));
  // Serial.print(F("TEMP: "));
  // Serial.print(sens_Temp);
  // Serial.println(F(" ํC"));
  // Serial.print(F("HUMI: "));
  // Serial.print(sens_Humi);
  // Serial.println(F(" %"));
  Serial.print(F("Sound: "));
  Serial.print(sens_dB);
  Serial.println(F(" dB"));
  Serial.print(F("1-Min People count: "));
  Serial.print(acc_sens_People);
  Serial.println(F(" Person"));
  Serial.println(F("===================================="));

  // SerialBT.println(F(""));
  // SerialBT.println(F("===================================="));
  // SerialBT.print(F("Sensor accumurate for AVG Count = "));
  // SerialBT.print(count_acc);
  // SerialBT.print(F("/"));
  // SerialBT.println(avgSens);
  // SerialBT.println(F("===================================="));

  // SerialBT.println(F("===================================="));
  // SerialBT.print(F("CO2: "));
  // SerialBT.print(sens_CO2);
  // SerialBT.println(F(" ppm"));
  // SerialBT.print(F("CH2O: "));
  // SerialBT.print(sens_CH2O);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("TVOC: "));
  // SerialBT.print(sens_TVOC);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("PM2.5: "));
  // SerialBT.print(sens_PM25);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("PM10: "));
  // SerialBT.print(sens_PM10);
  // SerialBT.println(F(" ug/m3"));
  // SerialBT.print(F("TEMP: "));
  // SerialBT.print(sens_Temp);
  // SerialBT.println(F(" ํC"));
  // SerialBT.print(F("HUMI: "));
  // SerialBT.print(sens_Humi);
  // SerialBT.println(F(" %"));
  // SerialBT.print(F("Sound: "));
  // SerialBT.print(sens_dB);
  // SerialBT.println(F(" dB"));
  // SerialBT.print(F("1-Min People count: "));
  // SerialBT.print(acc_sens_People);
  // SerialBT.println(F(" Person"));
  // SerialBT.println(F("===================================="));

  if (count_acc == avgSens)
  {
    // avg_sens_CO2 = acc_sens_CO2 / avgSens;
    // avg_sens_CH2O = acc_sens_CH2O / avgSens;
    // avg_sens_TVOC = acc_sens_TVOC / avgSens;
    // avg_sens_PM25 = acc_sens_PM25 / avgSens;
    // avg_sens_PM10 = acc_sens_PM10 / avgSens;
    // avg_sens_Temp = (acc_sens_Temp / avgSens)*0.1;
    // avg_sens_Humi = (acc_sens_Humi / avgSens)*0.1;
    avg_sens_dB = acc_sens_dB / avgSens;
    avg_sens_People += acc_sens_People; // All 12 time save to avg_sens.People is Accumulate

    // acc_sens_CO2 = 0;
    // acc_sens_CH2O = 0;
    // acc_sens_TVOC = 0;
    // acc_sens_PM25 = 0;
    // acc_sens_PM10 = 0;
    // acc_sens_Temp = 0;
    // acc_sens_Humi = 0;
    acc_sens_dB = 0;
    acc_sens_People = 0;

    count_acc = 0;
    // printDEBUG();
  }
  count = 0;
}

//****************************************************//
//******************* Ping online ********************//
//****************************************************//
void t3onlinePing()
{
  Serial.println();
  Serial.print(F("Ping www.google.com ... "));
  // SerialBT.println();
  // SerialBT.print(F("Ping www.google.com ... "));
  if (Network.pingIP("www.google.com"))
  {
    Serial.print(F("OK"));
    // SerialBT.print(F("OK"));
    count_ping = 0;
  }
  else
  {
    count_ping++;
    Serial.println(F("===================================="));
    Serial.print(F("Ping google.com FAIL count to RST: "));
    Serial.print(count_ping);
    Serial.print(F("/"));
    Serial.println(countPing);
    Serial.println(F("===================================="));
    // SerialBT.println(F("===================================="));
    // SerialBT.print(F("Ping google.com FAIL count to RST: "));
    // SerialBT.print(count_ping);
    // SerialBT.print(F("/"));
    // SerialBT.println(countPing);
    // SerialBT.println(F("===================================="));
    if (count_ping >= countPing)
    {
      count_ping = 0;
      Serial.println(F("Shutting Down GSM."));
      // SerialBT.println(F("Shutting Down GSM."));
      GSM.shutdown();
      for (int a = 0; a < 10; a++)
      {
        GSM.shutdown();
        delay(1000);
      }
      ESP.restart();
    }
  }
  Serial.println();
}

//****************************************************//
//*************** Print GSM Parameter ****************//
//****************************************************//
void print_GSMpara()
{
  Serial.println(F("//******************************//"));
  Serial.print(F("CCID: "));
  Serial.println(ccid);
  Serial.print(F("IMSI: "));
  Serial.println(imsi);
  Serial.print(F("IMEI: "));
  Serial.println(imei);
  Serial.print(F("IP: "));
  Serial.println(ip);
  Serial.print(F("Ping www.google.com ... "));

  Serial.println();
  Serial.println(F("//******************************//"));

  // SerialBT.println(F("//******************************//"));
  // SerialBT.print(F("CCID: "));
  // SerialBT.println(ccid);
  // SerialBT.print(F("IMSI: "));
  // SerialBT.println(imsi);
  // SerialBT.print(F("IMEI: "));
  // SerialBT.println(imei);
  // SerialBT.print(F("IP: "));
  // SerialBT.println(ip);
  // SerialBT.print(F("Ping www.google.com ... "));

  // SerialBT.println();
  // SerialBT.println(F("//******************************//"));
}

//****************************************************//
//*************** Get GSM Parameter ******************//
//****************************************************//
void get_GSMpara()
{
  ccid = GSM.getICCID();
  deviceToken = ccid;
  imsi = GSM.getIMSI();
  imei = GSM.getIMEI();
  ip = Network.getDeviceIP();
  print_GSMpara();
}

//*******************************************************************************************//
//************** Send an NTP request to the time server at the given address ****************//
//*******************************************************************************************//
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//****************************************************//
//***************** Get NTP Time *********************//
//****************************************************//
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0)
    ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  // SerialBT.println(F("Transmit NTP Request"));
  // get a random server from the pool
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500)
  {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE)
    {
      Serial.println(F("Receive NTP Response"));
      // SerialBT.println(F("Receive NTP Response"));
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  // SerialBT.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

//****************************************************//
//**************** Blink Heartbeat *******************//
//****************************************************//
void t6timeBlink()
{
  digitalWrite(ledPIN, HIGH);
  delay(150);
  digitalWrite(ledPIN, LOW);

  heartbeat(); // Trig Heartbeat to watchdog
}

//****************************************************//
//************* Midnight Auto reboot *****************//
//****************************************************//
void midNightReboot()
{
  if ((hour() == 23) && (minute() == 59))
  {
    Serial.println(F("Midnight Shutting Down GSM."));
    // SerialBT.println(F("Midnight Shutting Down GSM."));
    GSM.shutdown();
    for (int a = 0; a < 10; a++)
    {
      GSM.shutdown();
      delay(1000);
    }
    ESP.restart();
  }
}

//*****************************************************//
String HTTPReq(String url)
{
  String payload = "";
  WiFiClient client;
  HTTPClient http; // must be declared after WiFiClient for correct destruction order, because used by http.begin(client,...)

  Serial.print("[HTTP] begin...\n");

  // configure traged server and url
  http.begin(client, String(url));

  const char *keys[] = {"WWW-Authenticate"};
  http.collectHeaders(keys, 1);

  Serial.print("[HTTP] GET...\n");
  // start connection and send HTTP header
  int httpCode = http.GET();

  if (httpCode > 0)
  {
    String authReq = http.header("WWW-Authenticate");

    String authorization = getDigestAuth(authReq, String(username), String(password), "GET", url, 1);

    http.end();
    http.begin(client, String(url));

    http.addHeader("Authorization", authorization);

    int httpCode = http.GET();
    if (httpCode > 0)
    {
      payload = http.getString();
    }
    else
    {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
  return payload;
}

void setPeopleCount(float interval)
{
  // Max X : 322.8070175438596 , Max Y : 243.75
  // Center X : 161.4035087719 , Center Y : 121.875
  String url = "http://192.168.1.1/vb.htm?page=peoplecounting&ppcenable=1&ppcregularenable=1&ppcreportinterval=";
  url.concat(interval);
  url.concat("&ppcchangereportenable=1&ppcchangereporttype=0&ppcdetectdebouncetime=2&ppcsetenable=0&ppcdetectiontype=0&ppcplgx=-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;-1:-1:-1:-1:-1:-1:-1:-1:-1:-1:;");
  HTTPReq(url);
}

void setCounterCount(float startX, float startY, float endX, float endY, float interval)
{
  // Max X : 322.8070175438596 , Max Y : 243.75
  // Center X : 161.4035087719 , Center Y : 121.875
  String url = "http://192.168.1.1/vb.htm?page=msvca&counterenable=1&countreportinterval=";
  url.concat(interval);
  // url.concat("&displaystatus=1&osdtype=3&osdfontsize=1&osdfontcolor=0:255:0&osdtextpos=0&autoreset=0&resetday=7&resettime=00:00:00&countline=");
  // url.concat(startX);
  // url.concat(";");
  // url.concat(startY);
  // url.concat(";");
  // url.concat(endX);
  // url.concat(";");
  // url.concat(endY);
  // url.concat(";&countline_p1=1;2;3;4&countline_p2=1;2;3;4&countline_p3=1;2;3;4&countline_p4=1;2;3;4&countlinescene=0");
  HTTPReq(url);
}

void getCounterCount()
{
  String responseText = HTTPReq("http://192.168.1.1/vb.htm?language=ie&getcountercount");
  int InCountIndex = responseText.indexOf("=");
  String InCountStr = responseText.substring(InCountIndex);
  int EndInCountIndex = InCountStr.indexOf(",");
  InCountStr = InCountStr.substring(0, EndInCountIndex);
  int OutCountIndex = responseText.indexOf("=", InCountIndex + 1);
  String OutCountStr = responseText.substring(OutCountIndex);
  int EndOutCountIndex = OutCountStr.indexOf(",");
  OutCountStr = OutCountStr.substring(0, EndOutCountIndex);
  int InCount = InCountStr.toInt();
  int OutCount = OutCountStr.toInt();
}

//*****************************************************//

//****************************************************//
//*************** Setup Function *********************//
//****************************************************//
void setup()
{
  pinMode(ledPIN, OUTPUT);
  digitalWrite(ledPIN, LOW);

  Serial.begin(115200);
  RS485.begin(9600, SERIAL_8N1);
  EEPROM.begin(512);
  delay(1000);
  version = EEPROM.readString(addVersion);

  heartbeat();
  Serial.println();
  Serial.print(F("Boot mrta ... "));
  delay(1000);
  Serial.print(F("version: "));
  Serial.println(version);

  Serial.println();
  delay(1000);
  Serial.println(F("Setup GSM.."));
  while (!GSM.begin())
  {
    Serial.println(F("GSM setup fail"));
    delay(2000);
  }
  delay(1000);
  Serial.println(F("Setup GSM.. Success"));
  Serial.println();

  heartbeat();

  get_GSMpara();

  // gsm_client.setInsecure();

  Serial.println();
  delay(1000);

  Serial.println(F("Starting connection to UDP server..."));
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);

  // runner.addTask(t1);
  // runner.addTask(t2);
  // runner.addTask(t3);
  // runner.addTask(t4);
  // runner.addTask(t5);
  // t1.enable();
  // t2.enable();
  // t3.enable();
  // t4.enable();
  // t5.enable();
  // Serial.println(F("Enabled t1"));
  // Serial.println(F("Enabled t2"));
  // Serial.println(F("Enabled t3"));
  // Serial.println(F("Enabled t4"));
  // Serial.println(F("Enabled t5"));

  // client.setServer(thingsboardServer, PORT);
  // client.setCallback(callback);

  // reconnectMqtt();

  heartbeat();

  Serial.println();
  t5reConnWiFi();
  Serial.println();
  t4getOTA();
  Serial.println();
  t7getCameraType();
  Serial.println();
  if (camType == 1)
  {
    Serial.println(F(""));
    Serial.println(F("!! Set Reporting Interval to 300 S !!"));
    setCounterCount(161.4035087719, 0, 161.4035087719, 243.75, defaultX);
  }

  heartbeat();

  String btName = "mrta-";
  btName += ccid;
  // SerialBT.begin(btName); //Bluetooth device name
  // SerialBT.println(btName);

  esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               // add current thread to WDT watch

  digitalWrite(ledPIN, LOW);
  esp_task_wdt_reset();

  // Note - the default maximum packet size is 128 bytes. If the
  // combined length of clientId, username and password exceed this use the
  // following to increase the buffer size:
  // client.setBufferSize(255);

  previous_t1 = millis() / 1000;
  previous_t2 = millis() / 1000;
  previous_t3 = millis() / 1000;
  previous_t4 = millis() / 1000;
  previous_t5 = millis() / 1000;
  previous_t6 = millis() / 1000;
  previous_t7 = millis() / 1000;
  previous_t8 = millis() / 1000;

  heartbeat();
}

//****************************************************//
//*************** Loop Function **********************//
//****************************************************//
void loop()
{
  // runner.execute();

  // if (!client.connected())
  // {
  //   reconnectMqtt();
  // }
  // else
  // {
  //   client.loop();
  // }

  current = millis() / 1000;

  // Task t1(5000, TASK_FOREVER, &t1AccuAVG);
  if ((current - previous_t1) >= t1)
  {
    previous_t1 = millis() / 1000;
    t1AccuAVG();
  }

  // Task t2(60000, TASK_FOREVER, &t2sendtelemetry);
  if ((current - previous_t2) >= t2)
  {
    previous_t2 = millis() / 1000;
    t2sendtelemetry();
  }

  // Task t3(60000, TASK_FOREVER, &t3onlinePing);
  if ((current - previous_t3) >= t3)
  {
    previous_t3 = millis() / 1000;
    t3onlinePing();
  }

  // Task t4(300000, TASK_FOREVER, &t4getOTA);
  if ((current - previous_t4) >= t4)
  {
    previous_t4 = millis() / 1000;
    t4getOTA();
  }

  // Task t5(300000, TASK_FOREVER, &t5reConnWiFi);
  if ((current - previous_t5) >= t5)
  {
    previous_t5 = millis() / 1000;
    t5reConnWiFi();
  }

  // Task t6(3000, TASK_FOREVER, &t6timeBlink);
  if ((current - previous_t6) >= t6)
  {
    previous_t6 = millis() / 1000;
    t6timeBlink();
  }

  // Task t7(300000, TASK_FOREVER, &t7getCameraType);
  if ((current - previous_t7) >= t7)
  {
    previous_t7 = millis() / 1000;
    t7getCameraType();
  }

  // Task t8(18000000, TASK_FOREVER, &t8getCameraType);
  if ((current - previous_t8) >= t8)
  {
    previous_t8 = millis() / 1000;
    if (camType == 1)
    {
      Serial.println(F(""));
      Serial.println(F("!! Set Reporting Interval to 300 S !!"));
      // SerialBT.println(F(""));
      // SerialBT.println(F("!! Set Reporting Interval to 300 S !!"));
      setCounterCount(161.4035087719, 0, 161.4035087719, 243.75, defaultX);
    }
  }

  // Midnight Auto reboot
  midNightReboot();

  esp_task_wdt_reset();
}