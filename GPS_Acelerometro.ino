#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <Wire.h>         // biblioteca de comunicação I2C

TinyGPSPlus gps; // Objeto para manipulação dos dados do GPS
SoftwareSerial gpsSerial(D4, D3); // RX, TX - Configuração do objeto para comunicação serial com o GPS

float Latitude , Longitude; // Variáveis para armazenar latitude e longitude
String LatitudeString , LongitudeString; // Variáveis de string para armazenar latitude e longitude como texto

const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio
const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro

const int sda_pin = D5; // definição do pino I2C SDA
const int scl_pin = D6; // definição do pino I2C SCL

bool led_state = false; // Estado atual do LED

// Variáveis para armazenar os dados "crus" do acelerômetro e giroscópio
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void initI2C() 
{
  Wire.begin(sda_pin, scl_pin); // Inicia a comunicação I2C
}

/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int reg, int val)
{
  Wire.beginTransmission(MPU_ADDR); // Inicia a comunicação com o endereço do MPU6050
  Wire.write(reg); // Envia o registro com o qual se deseja trabalhar
  Wire.write(val); // Escreve o valor no registro
  Wire.endTransmission(true); // Termina a transmissão
}

/*
 * função que lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg)
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR); // Inicia a comunicação com o endereço do MPU6050
  Wire.write(reg); // Envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false); // Termina transmissão, mas mantém I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1); // Configura para receber 1 byte do registro escolhido acima
  data = Wire.read(); // Lê o byte e guarda em 'data'
  return data; // Retorna 'data'
}

/*
 * função que procura pelo sensor no endereço 0x68
 */
void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
 
  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
  }
}

void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR); // Procura pelo MPU6050 no endereço 0x68
     
  int data = readRegMPU(WHO_AM_I); // Registro 117 – Who Am I - 0x75
   
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    data = readRegMPU(PWR_MGMT_1); // Registro 107 – Power Management 1 - 0x6B
 
    if(data == 64) 
      Serial.println("MPU6050 em modo SLEEP! (64)");
    else 
      Serial.println("MPU6050 em modo ACTIVE!"); 
  }
  else 
  {
    Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
  }
}

void initMPU()
{
  setSleepOff(); // Configura o MPU6050 para o modo ACTIVE
  setGyroScale(); // Configura a escala do giroscópio
  setAccelScale(); // Configura a escala do acelerômetro
}

void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // Escreve 0 no registro de gerenciamento de energia (0x68), colocando o sensor no modo ACTIVE
}

void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0); // Escreve 0 no registro de configuração do giroscópio
}

void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0); // Escreve 0 no registro de configuração do acelerômetro
}

int readRawMPU(int a)
{ 
  // Lê os dados brutos do MPU6050 e os armazena nas variáveis correspondentes
  if (a == 1){
    Wire.beginTransmission(MPU_ADDR); // Inicia a comunicação com o endereço do MPU6050
    Wire.write(ACCEL_XOUT); // Envia o registro do eixo X do acelerômetro (0x3B)
    Wire.endTransmission(false); // Termina transmissão, mas mantém I2C aberto (envia STOP e START)
    Wire.requestFrom(MPU_ADDR, 14); // Configura para receber 14 bytes a partir do registro escolhido acima (0x3B)
    
    AcX = Wire.read() << 8; // Lê primeiro o byte mais significativo
    AcX |= Wire.read(); // Depois lê o bit menos significativo
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
    return AcX; // Retorna o valor lido do eixo X do acelerômetro
  }

  if (a == 2){
    Serial.print(" | AcY = "); Serial.print(AcY);
    return AcY; // Retorna o valor lido do eixo Y do acelerômetro
  }

  if (a == 3){
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    return AcZ; // Retorna o valor lido do eixo Z do acelerômetro
  }
  if (a == 4){
    Serial.print(" | GyX = "); Serial.print(GyX);
    return GyX; // Retorna o valor lido do eixo X do giroscópio
  }
  if (a == 5){
    Serial.print(" | GyY = "); Serial.print(GyY);
    return GyY; // Retorna o valor lido do eixo Y do giroscópio
  }
  if (a == 6){
    Serial.print(" | GyZ = "); Serial.println(GyZ);
    return GyZ; // Retorna o valor lido do eixo Z do giroscópio
  }
}

void setup() {
  Serial.begin(9600); // Inicia a comunicação serial
  gpsSerial.begin(9600); // Inicia a comunicação serial com o GPS
  
  initI2C(); // Inicializa a comunicação I2C
  checkMPU(MPU_ADDR); // Verifica se o MPU6050 está disponível
  
  initMPU(); // Inicializa o MPU6050

  delay(1000); // Aguarda 1 segundo
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

  int acX = readRawMPU(1); // Lê o valor do eixo X do acelerômetro
  int acY = readRawMPU(2); // Lê o valor do eixo Y do acelerômetro
  int acZ = readRawMPU(3); // Lê o valor do eixo Z do acelerômetro
  int gyX = readRawMPU(4); // Lê o valor do eixo X do giroscópio
  int gyY = readRawMPU(5); // Lê o valor do eixo Y do giroscópio
  int gyZ = readRawMPU(6); // Lê o valor do eixo Z do giroscópio

  // Imprime os valores lidos do acelerômetro e giroscópio
  Serial.print(" | Acelerômetro - X: ");
  Serial.print(acX);
  Serial.print(" Y: ");
  Serial.print(acY);
  Serial.print(" Z: ");
  Serial.print(acZ);
  Serial.print(" | Giroscópio - X: ");
  Serial.print(gyX);
  Serial.print(" Y: ");
  Serial.print(gyY);
  Serial.print(" Z: ");
  Serial.println(gyZ);

  delay(1000); // Aguarda 1 segundo antes de realizar a próxima leitura
