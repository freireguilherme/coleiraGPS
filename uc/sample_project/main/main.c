#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define DELAY_IN_MINUTES 1
#define TICK_RATE_MS 1000 // assuming tick rate is 1ms
#define SerialGPS Serial2

const char *ssid = "Freire 2G";
const char *password = "262806va";
const char *serverName = "https://postman-echo.com/post"; //"https://daea-170-78-23-134.sa.ngrok.io";

const int RXPin = 4; // SIM800L
const int TXPin = 5;

SoftwareSerial SerialGSM;
// HardwareSerial SerialGPS(2);

SemaphoreHandle_t meuMutex;
TaskHandle_t taskGPSHandle, taskWifiHandle, taskGSMHandle;

StaticJsonDocument<200> gpsData;

String requestBody;

void setup()
{
  Serial.begin(115200);                                     // monitor
  SerialGSM.begin(9600, SWSERIAL_8N1, RXPin, TXPin, false); // GSM module, PINS RX: 4 e TX: 5, software serial
  if (!SerialGSM)                                           // If the object did not initialize, then its configuration is invalid
  {
    Serial.println("Invalid SoftwareSerial pin configuration, check config");
    while (1)
    {
    }
  }
  SerialGPS.begin(115200, SERIAL_8N1, 16, 17); // GPS modute, PINS RX2 e TX2

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  Serial.println("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  meuMutex = xSemaphoreCreateMutex();

  xTaskCreate(taskGPS, "task_gps", 10000, NULL, 1, &taskGPSHandle);
  xTaskCreate(taskWifi, "task_wifi", 10000, NULL, 1, &taskWifiHandle);
  // xTaskCreate(taskGSM, "task_gsm", 10000, NULL, 1, &taskGSMHandle);
}

void loop()
{
}

void taskGPS(void *pvParameters) // coleta dados do gps
{
  TinyGPSPlus gps;
  double lat, lng, /*velocidade, altitude, data, hora, minutos, segundos,*/ dados;

  while (true)
  {
    while (SerialGPS.available() > 0)
    {
      if (gps.encode(SerialGPS.read()))
      {
        if (gps.location.isValid())
        {
          lat = gps.location.lat();
          lng = gps.location.lng();
          /*velocidade = gps.speed.kmph();
                      altitude = gps.altitude.meters();
                      data = gps.date.value();
                      hora = gps.time.hour();
                      minutos = gps.time.minute();
                      segundos = gps.time.second();
                      */
          if (xSemaphoreTake(meuMutex, portMAX_DELAY) == pdTRUE)
          {
            gpsData.clear();
            gpsData["lat"] = lat;
            gpsData["lng"] = lng;
            /*gpsData["velocidade"] = velocidade;
                          gpsData["altitude"] = altitude;
                          gpsData["data"] = data;
                          gpsData["hora"] = hora;
                          gpsData["minutos"] = minutos;
                          gpsData["segundos"] = segundos;
                          */
            // requestBody = "\0";
            // serializeJson(gpsData, requestBody);

            // Serial.println(requestBody);
            xSemaphoreGive(meuMutex);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000)); // delay de 0,5s
    // vTaskSuspend(taskGPSHandle);
  }
}

void taskWifi(void *pvParameters) // envia por wifi
{
  while (true)
  {
    // garantir que está conectado no wifi
    WiFiClient client;
    HTTPClient http;
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

    // começa uma conexao entre o localhost e o server
    http.begin(client, serverName); // char *serverName = "https://bdfa-170-78-23-134.sa.ngrok.io/";
    // informa o cabeçalho
    http.addHeader("Content-Type", "application/json");

    // transforma para uma versão string minimizada do Json. pode usar serializeJsonPretty()
    // entra no semáfaro por ate 58s
    if (xSemaphoreTake(meuMutex, pdMS_TO_TICKS(58000)) == pdTRUE)
    {
      requestBody = "";
      serializeJson(gpsData, requestBody);
      Serial.println(requestBody);
      // envia a requisição post
      int httpResponseCode = http.POST(requestBody);
      vTaskDelay(pdMS_TO_TICKS(500));

      // imprime a resposta
      // Serial.println(httpResponseCode);
      if (httpResponseCode > 0)
      {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
      }
      else
      {
        Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
      }
      // Serial.println(requestBody);
      // sai do semáforo
      xSemaphoreGive(meuMutex);
    }

    http.end();
    // vTaskResume(taskGPSHandle);
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

    SerialGSM.println("AT");
    vTaskDelay(pdMS_TO_TICKS(3000));

    SerialGSM.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+SAPBR=3,1,\"APN\",\"gprs.oi.com.br\""); // APN
    vTaskDelay(pdMS_TO_TICKS(6000));
    Serial.write(SerialGSM.read());

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

    SerialGSM.println("AT+HTTPPARA=\"URL\",\"https://bdfa-170-78-23-134.sa.ngrok.io\""); // Server address
    vTaskDelay(pdMS_TO_TICKS(4000));
    Serial.write(SerialGSM.read());

    SerialGSM.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
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
