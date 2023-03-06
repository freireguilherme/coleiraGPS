#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "ThingSpeak.h"

#define DELAY_IN_MINUTES 1
#define TICK_RATE_MS 1000 // assuming tick rate is 1ms
#define SerialGPS Serial2



unsigned long myChannelNumber = 2053888;
const char * myWriteAPIKey = "GH6G0BUMNLU1LC22";
const char *ssid = "Jr Telecom - MA";
const char *password = "simpatica";
const char *serverName = "https://f12f-2804-1690-82a-f2a2-592b-fa4-9bc2-3295.sa.ngrok.io"; //"https://daea-170-78-23-134.sa.ngrok.io";
const char serverName2[] = "http://515c-2804-1690-82a-f2a2-6146-250a-5019-963a.sa.ngrok.io";
const char *server = "api.thingspeak.com";
String endpoint = "/api/data";
const int port = 3000;
float lat, lng = 0.0;

const char *test_root_ca = \
                           "-----BEGIN CERTIFICATE-----\n" \
                           "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
                           "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
                           "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
                           "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
                           "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
                           "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
                           "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
                           "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
                           "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
                           "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
                           "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
                           "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
                           "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
                           "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
                           "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
                           "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
                           "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
                           "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
                           "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
                           "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
                           "-----END CERTIFICATE-----\n";



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
  /*velocidade, altitude, data, hora, minutos, segundos,*/

  while (true)
  {
    while (Serial2.available() > 0)
    {
      if (gps.encode(Serial2.read()))
      {
        //if (gps.location.isValid())
        //{
        if (xSemaphoreTake(meuMutex, portMAX_DELAY) == pdTRUE)
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
          /*requestBody = "\0";
            serializeJson(gpsData, requestBody);

            Serial.println(requestBody + "task gps");
          */
          xSemaphoreGive(meuMutex);
        }
        //}
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

    HTTPClient http;
    WiFiClient client;
    ThingSpeak.begin(client);  // Initialize ThingSpeak

    

    /*client.setTimeout(5000);
      if (!client.connect(serverName2, port)) {
      Serial.println("Connection failed");
      return;
      }

      // Cria a requisição GET
      // send HTTP header
      requestBody = "";
      serializeJson(gpsData, requestBody);
      Serial.println(serverName2 + endpoint);
      String request = "POST " + String(endpoint) + " HTTP/1.1\r\n" +
                     "Host: " + String(serverName2) + "\r\n" +
                     "Content-Type: application/json\r\n" +
                     "Content-Length: " + String(requestBody.length()) + "\r\n" +
                     "Connection: close\r\n\r\n" +
                     requestBody;



      // send HTTP body
      client.println(request);


      // Espera pela resposta
      while (client.connected()) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.println(line);
      }
      }

      // Encerra a conexão
      client.stop();
    */

    if (xSemaphoreTake(meuMutex, pdMS_TO_TICKS(58000)) == pdTRUE)
    {
      /*  int length = sizeof requestBody;
        // começa uma conexao entre o localhost e o server
        http.begin(client, serverName + endpoint); // char *serverName = "https://bdfa-170-78-23-134.sa.ngrok.io/";
        // informa o cabeçalho
        //http.addHeader("ngrok-skip-browser-warning", "true");
        http.addHeader("Content-Length", String(length));
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Accept", "text/html,application/xhtml+xml,application/xml,application/json");
        http.addHeader("Accept-Charset", "ISO-8859-1,utf-8" );
        //http.addHeader("User-Agent", "DigiCert Global Root CA" );
        //http.addHeader("Keep-Alive", "300" );
        //http.addHeader("Connection", "keep-alive" );
        http.addHeader("Connection", "close" );

        // transforma para uma versão string minimizada do Json. pode usar serializeJsonPretty()
        requestBody = "";
        serializeJson(gpsData, requestBody);
        // entra no semáfaro por ate 58s
        Serial.println(requestBody + " task wifi");
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
      */
      int x = thingSpeakWrite(myChannelNumber,[lat,lng],'Fields',[lat,lng],'Writekey',myWriteAPIKey);
      if(x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
      xSemaphoreGive(meuMutex);
    }

    client.stop();
    // vTaskResume(taskGPSHandle);
    // delay de 1 minutos entre os envios
    //vTaskDelay(DELAY_IN_MINUTES * 60 * TICK_RATE_MS / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(5000));
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
