#include <TinyGPS++.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// The TinyGPSPlus object
TinyGPSPlus gps;
SoftwareSerial gpsSerial(D4, D3); // RX, TX - Configuração do objeto para comunicação serial com o GPS

float Latitude , Longitude; // Variáveis para armazenar latitude e longitude
String LatitudeString , LongitudeString; // Variáveis de string para armazenar latitude e longitude como texto

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600); // Inicia a comunicação serial com o GPS
  delay(1000);
}


void loop() {
 while (gpsSerial.available() > 0) { // Verifica se há dados disponíveis para leitura do GPS
    gps.encode(gpsSerial.read()); // Lê e processa os dados recebidos do GPS
    if (gps.location.isUpdated()) { // Verifica se a localização do GPS foi atualizada
      Latitude = gps.location.lat(); // Obtém a latitude
      Longitude = gps.location.lng(); // Obtém a longitude
      LatitudeString = String(Latitude, 6); // Converte a latitude em string
      LongitudeString = String(Longitude, 6); // Converte a longitude em string
    }
  }

  Serial.print("Latitude: ");
  Serial.print(LatitudeString); // Imprime a latitude
  Serial.print(" | Longitude: ");
  Serial.print(LongitudeString); // Imprime a longitude
}
