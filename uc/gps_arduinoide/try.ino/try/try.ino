#include <TinyGPS.h>
#include <TinyGPSPlus.h>

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  delay(3000);
}

void loop() {
  //updateSerial();
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  //if (gps.location.isValid()){
    Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("Lat: ");
    Serial.println(gps.location.rawLat().deg);
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("Lng: ");
    Serial.println(gps.location.rawLng().deg);
    Serial.print(F(","));
    Serial.print("\n");
    Serial.print(gps.date.day());//LEITURA DO DIA
    Serial.print("/");
    Serial.print(gps.date.month());//LEITURA DO MêS
    Serial.print("/");
    Serial.println(gps.date.year());//LEITURA DO ANO
    
    if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour() - 3); //AJUSTA O FUSO HORARIO PARA NOSSA REGIAO (FUSO DE SP 03:00, POR ISSO O -3 NO CÓDIGO) E IMPRIME NA SERIAL
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());//IMPRIME A INFORMAÇÃO DOS MINUTOS NA SERIAL
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(gps.time.second());//IMPRIME A INFORMAÇÃO DOS SEGUNDOS NA SERIAL
 
//}
    Serial.println();
  //}  
  /*else
  {
    Serial.print(F("INVALID\n"));
  }*/
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}