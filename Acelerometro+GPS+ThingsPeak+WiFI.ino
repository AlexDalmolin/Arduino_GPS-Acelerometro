#include <ESP8266WiFi.h> //essa biblioteca já vem com a IDE. Portanto, não é preciso baixar nenhuma biblioteca adicional
#include <Wire.h>         // biblioteca de comunicação I2C
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

/* defines - wi-fi */
#define SSID_REDE "robson's Galaxy A21s" /* coloque aqui o nome da rede que se deseja conectar */
#define SENHA_REDE "aroy3647" /* coloque aqui a senha da rede que se deseja conectar */
#define INTERVALO_ENVIO_THINGSPEAK 30000 /* intervalo entre envios de dados ao ThingSpeak (em ms) */

//GPS
TinyGPSPlus gps;
SoftwareSerial gpsSerial(D4, D3); // RX, TX

const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio
const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro
 
const int sda_pin = D5; // definição do pino I2C SDA
const int scl_pin = D6; // definição do pino I2C SCL
 
bool led_state = false;
 
// variáveis para armazenar os dados "crus" do acelerômetro
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
/* constantes e variáveis globais */
char endereco_api_thingspeak[] = "api.thingspeak.com";
String chave_escrita_thingspeak = "6KHRB2TN9AGRHPNP";  /* Coloque aqui sua chave de escrita do seu canal */
unsigned long last_connection_time;
WiFiClient client;
 
/* prototypes */
void envia_informacoes_thingspeak(String string_dados);
void init_wifi(void);
void conecta_wifi(void);
void verifica_conexao_wifi(void);
 
/*
* Implementações
*/
 
/* Função: envia informações ao ThingSpeak
* Parâmetros: String com a informação a ser enviada
* Retorno: nenhum
*/
void envia_informacoes_thingspeak(String string_dados)
{
    if (client.connect(endereco_api_thingspeak, 80))
    {
        /* faz a requisição HTTP ao ThingSpeak */
        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        client.print("X-THINGSPEAKAPIKEY: "+chave_escrita_thingspeak+"\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(string_dados.length());
        client.print("\n\n");
        client.print(string_dados);
         
        last_connection_time = millis();
        Serial.println("- Informações enviadas ao ThingSpeak!");
    }
}
 
/* Função: inicializa wi-fi
* Parametros: nenhum
* Retorno: nenhum
*/
void init_wifi(void)
{
    Serial.println("------WI-FI -----");
    Serial.println("Conectando-se a rede: ");
    Serial.println(SSID_REDE);
    Serial.println("\nAguarde...");
 
    conecta_wifi();
}
 
/* Função: conecta-se a rede wi-fi
* Parametros: nenhum
* Retorno: nenhum
*/
void conecta_wifi(void)
{
    /* Se ja estiver conectado, nada é feito. */
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }
     
    /* refaz a conexão */
    WiFi.begin(SSID_REDE, SENHA_REDE);
     
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
    }
 
    Serial.println("Conectado com sucesso a rede wi-fi \n");
    Serial.println(SSID_REDE);
}
 
/* Função: verifica se a conexao wi-fi está ativa
* (e, em caso negativo, refaz a conexao)
* Parametros: nenhum
* Retorno: nenhum
*/
void verifica_conexao_wifi(void)
{
    conecta_wifi();
}

void initI2C() 
{
  //Serial.println("---inside initI2C");
  Wire.begin(sda_pin, scl_pin);
}
 
/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int reg, int val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão
}

/*
 * função que lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}
 
/*
 * função de inicialização do sensor
 */
void initMPU()
{
  setSleepOff();
  setGyroScale();
  setAccelScale();
}
 
/* 
 *  função para configurar o sleep bit  
 */
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}
 
/* função para configurar as escalas do giroscópio
   registro da escala do giroscópio: 0x1B[4:3]
   0 é 250°/s
 */
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}
 
/* função para configurar as escalas do acelerômetro
   registro da escala do acelerômetro: 0x1C[4:3]
   0 é 250°/s
 
*/
void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}
 
/* função que lê os dados 'crus'(raw data) do sensor
   são 14 bytes no total sendo eles 2 bytes para cada eixo e 2 bytes para temperatura:
*/

int readRawMPU(int a)
{ 
  if (a==1){
  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
  
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 
  
  Serial.print("AcX = "); Serial.print(AcX);
  return AcX;
  }

  if (a==2){
  Serial.print(" | AcY = "); Serial.print(AcY);
    return AcY;
  }

  if (a==3){
  Serial.print(" | AcZ = "); Serial.print(AcZ);
    return AcZ;
  }

  if (a==4){
  Serial.print(" | GyX = "); Serial.print(GyX);
    return GyX;
  }

  if (a==5){
  Serial.print(" | GyY = "); Serial.print(GyY);
    return GyY;
  }

  if (a==6){
  Serial.print(" | GyZ = "); Serial.println(GyZ);
    return GyZ;
  }

  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor
  delay(50);                                        
}

//Variaveis do GPS
float Latitude , Longitude;
String LatitudeString , LongitudeString;
 
void setup()
{
    Serial.begin(115200);
    last_connection_time = 0;
    
    // GPS 
     gpsSerial.begin(9600);
    
 pinMode(LED_BUILTIN, OUTPUT);
 
  Serial.println("nIniciando configuração do MPU6050n");
  initI2C();
  initMPU();
  Serial.println("nConfiguração finalizada, iniciando loopn");
  
    /* Inicializa e conecta-se ao wi-fi */
    init_wifi();
}
 
//loop principal
void loop()
{
    char fields_a_serem_enviados[100] = {0};
    float AcX, AcY, AcZ, GyX, GyY, GyZ;
 
    /* Força desconexão ao ThingSpeak (se ainda estiver conectado) */
    if (client.connected())
    {
        client.stop();
        Serial.println(" - Desconectado do ThingSpeak");
        Serial.println();
    }
   
    /* Garante que a conexão wi-fi esteja ativa */
    verifica_conexao_wifi();
     
    /* Verifica se é o momento de enviar dados para o ThingSpeak */
   if( millis() - last_connection_time > INTERVALO_ENVIO_THINGSPEAK ){

       //Dados do Acelerometro
       AcX = readRawMPU(1);
       AcY = readRawMPU(2);
       AcZ = readRawMPU(3);
       GyX = readRawMPU(4);
       GyY = readRawMPU(5);
       GyZ = readRawMPU(6);
        
   //Pegar dados do GPS
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {
        Latitude = gps.location.lat();
        Serial.println("Latitude: ");
        LatitudeString = String(Latitude , 6);
        Serial.println(LatitudeString);
        Longitude = gps.location.lng();
        Serial.println("Longitude: ");
        LongitudeString = String(Longitude , 6);
        Serial.println(LongitudeString);
      }
     else{
      Serial.print("INVALID");
       }
     }
     if (millis() > 5000 && gps.charsProcessed() < 10){
     Serial.println("No GPS detected: check wiring.");
   }

  sprintf(fields_a_serem_enviados,"field1=%.2f&field2=%.2f&field3=%.2f&field4=%.2f&field5=%.2f&field6=%f&field7=%f&field8=%f", AcX, AcY, AcZ, GyX, GyY, GyZ, LatitudeString, LongitudeString);
  envia_informacoes_thingspeak(fields_a_serem_enviados);
    }
 
    delay(1000);
}
