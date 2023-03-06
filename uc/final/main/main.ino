#include <TinyGPSPlus.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "ThingSpeak.h"

#define DELAY_IN_MINUTES 1
#define TICK_RATE_MS 1000 // assuming tick rate is 1ms
#define SerialGPS Serial2


unsigned long myChannelNumber = 2053888;
const String myWriteAPIKey = ""; //chave api para escrita
const String server = "https://api.thingspeak.com/update";

const char *ssid = "Freire 2G";
const char *password = "262806va";

float lat, lng = 0.0;

const String USER = "oi";
const String PASS = "oi";


const int RXPin = 4; // SIM800L
const int TXPin = 5;

SoftwareSerial SerialGSM;
// HardwareSerial SerialGPS(2);

SemaphoreHandle_t meuMutex;
TaskHandle_t taskGPSHandle, taskWifiHandle, taskGSMHandle;

StaticJsonDocument<200> gpsData;

String requestBody = "";

void setup()
{
  Serial.begin(115200);                                     // monitor
  while (!Serial);
  SerialGSM.begin(9600, SWSERIAL_8N1, RXPin, TXPin, false); // GSM module, PINS RX: 4 e TX: 5, software serial
  if (!SerialGSM)                                           // If the object did not initialize, then its configuration is invalid
  {
    Serial.println("Invalid SoftwareSerial pin configuration, check config");
    while (1)
    {
    }
  }
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // GPS modute, PINS RX2 e TX2
  while (!Serial2);


  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println(".");
  }

  Serial.println("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  meuMutex = xSemaphoreCreateMutex(); //inicializo meu semáforo

  xTaskCreate(taskGPS, "task_gps", 10000, NULL, 1, &taskGPSHandle);
  xTaskCreate(taskWifi, "task_wifi", 10000, NULL, 1, &taskWifiHandle);
  xTaskCreate(taskGSM, "task_gsm", 10000, NULL, 1, &taskGSMHandle);
}

void loop()
{
}

void taskGPS(void *pvParameters) // coleta dados do gps
{
  TinyGPSPlus gps;
  /*velocidade, altitude, data, hora, minutos, segundos,*/

  while (true)
  {
    while (Serial2.available() > 0)
    {
      if (gps.encode(Serial2.read()))
      {
        if (gps.location.isValid())
        {
          if (xSemaphoreTake(meuMutex, portMAX_DELAY) == pdTRUE)
          {
            lat = gps.location.lat();
            lng = gps.location.lng();

            gpsData.clear();
            gpsData["lat"] = lat;
            gpsData["lng"] = lng;

            xSemaphoreGive(meuMutex);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // delay de 0,5s
    //vTaskSuspend(taskGPSHandle);
  }
}

void taskWifi(void *pvParameters) // envia por wifi
{
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi connection lost, reconnecting...");
      WiFi.begin(ssid, password);

      while (WiFi.status() != WL_CONNECTED)
      {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.println("Connecting to WiFi...");
      }

      Serial.println("Connected to WiFi");
    } // end if

    WiFiClient client;
    ThingSpeak.begin(client);  // Initialize ThingSpeak

    if (xSemaphoreTake(meuMutex, pdMS_TO_TICKS(58000)) == pdTRUE)
    {
      ThingSpeak.setField(1, lat);
      ThingSpeak.setField(2, lng);
      int x = ThingSpeak.writeFields(myChannelNumber , (myWriteAPIKey).c_str());
      if (x == 200) {
        Serial.println("Channel update successful.");
      }
      else {
        Serial.println("Problem updating channel. HTTP error code " + String(x));
      }
      xSemaphoreGive(meuMutex);
    }

    client.stop();
    //vTaskResume(taskGPSHandle);
    // delay de 1 minutos entre os envios
    vTaskDelay(DELAY_IN_MINUTES * 60 * TICK_RATE_MS / portTICK_PERIOD_MS);
  }
}


void taskGSM(void *pvParameters) // envia por gprs
{
  while (true)
  {
    // GSM Communication Starts

    if (SerialGSM.available())
      Serial.write(SerialGSM.read());

    requestBody = server;
    requestBody += "?api_key=" + myWriteAPIKey;
    requestBody += "&field1=";
    requestBody += String(lat);
    requestBody += "&field2=";
    requestBody += String(lng);

    SerialGSM.println("AT");
    vTaskDelay(pdMS_TO_TICKS(3000));

    SerialGSM.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+SAPBR=3,1,\"APN\",\"gprs.oi.com.br\""); // APN
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+SAPBR=3,1,\"USER\"," + USER);
    SerialGSM.println("AT+SAPBR=3,1,\"PASS\"," + PASS);

    // Enable bearer 1
    SerialGSM.println("AT+SAPBR=1,1");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    // Check whether bearer 1 is open.
    SerialGSM.println("AT+SAPBR=2,1");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPINIT");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPPARA=\"CID\",1");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());
    vTaskDelay(pdMS_TO_TICKS(4000));

    SerialGSM.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    vTaskDelay(pdMS_TO_TICKS(4000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPPARA=\"URL\"," + requestBody); // Server address
    vTaskDelay(pdMS_TO_TICKS(4000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPDATA=" + String(requestBody.length()) + ",100000");
    Serial.println(requestBody);
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println(requestBody);
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPACTION=1");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPREAD");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPTERM");
    vTaskDelay(pdMS_TO_TICKS(10000));
    Serial.write(SerialGSM.read());

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
