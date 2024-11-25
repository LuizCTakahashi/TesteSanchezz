#include <WiFi.h>              // Biblioteca para conectar o Wi-Fi no ESP32-S3
#include <PubSubClient.h>      // Biblioteca para o protocolo MQTT
#include <Wire.h>
#include <Adafruit_MPU6050.h>  // Biblioteca para MPU6050
#include <Adafruit_Sensor.h>   // Biblioteca para MPU6050
#include <Adafruit_INA219.h>   // Biblioteca para o Sensor de corrente
#include <FastLED.h>           // Biblioteca para o LED


// Variaveis do LED
#define LED_PIN     48
#define NUM_LEDS    1
#define BRIGHTNESS  153
#define PULSE_SPEED 50
#define DELAY_TIME  6000
CRGB leds[NUM_LEDS];

// Variaveis do sensor de corrente
Adafruit_INA219 ina219;
float tensao = 0;
float corrente = 0;
float potencia = 0;

// Variaveis do MPU6050
Adafruit_MPU6050 mpu;
#define MPU6050_ADDRESS 0x68

int16_t ax, ay, az;  // Acelerômetro
int16_t gx, gy, gz;  // Giroscópio
unsigned long previousTime = 0;
float elapsedTime;
unsigned long lastTime; // Tempo da última leitura

int numleituras = 100;
float AErroX, AErroY, GErroZ;
float offsetaX, offsetaY, offsetgZ;
int angm1, angm2, angm3, angm4, angm5;

float gyroZ = 0;           // Taxa de rotação no eixo Z
float angleX = 0, angleY = 0, angleZ = 0;      // Ângulo acumulado em graus
float gyroXAngle, gyroYAngle, gyroZAngle;
int matriz[3][5];
int offset[3][4];
// Pinos de selecao AD0
const int pinAcel1 = 20;
const int pinAcel2 = 21;
const int pinAcel3 = 47;
const int pinAcel4 = 48;
const int pinAcel5 = 45;
// Defina os pinos SDA e SCL para ESP32-S3
const int I2C_SDA = 8;
const int I2C_SCL = 9;


// Variaveis do WIFI
const char* ssid = "Galleria 151";
const char* password = "diegobruna151";

// Variaveis do MQTT
const char* mqtt_broker = "broker.hivemq.com";
const char* topic1 = "Eixo1";
const char* topic2 = "Eixo2";
const char* topic3 = "Eixo3";
const char* topic4 = "Eixo4";
const char* topic5 = "Eixo5";
const char* topic_voltage = "Tensão";
const char* topic_current = "Corrente";
const char* topic_power = "Potencia";

// Definindo o cliente Wi-Fi e MQTT
WiFiClient espClient1;
PubSubClient client(espClient1);

void pulseColor(CRGB color) {
    // Faz o brilho subir e descer para criar o efeito de "pulso"
    for (int brightness = 0; brightness <= BRIGHTNESS; brightness++) {
        FastLED.setBrightness(brightness);
        leds[0] = color;
        FastLED.show();
    }
}

void connect_wifi() {
  Serial.println();
  Serial.print("Conectando ao Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    pulseColor(CRGB::Red);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Conectado ao Wi-Fi!");
  pulseColor(CRGB::Blue);
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

void connect_mqtt() {
  client.setServer(mqtt_broker, 1883);

  while (!client.connected()) {
    Serial.print("Conectando ao MQTT Broker...");
    if (client.connect("esp32s3_client1")) {
      Serial.println("Conectado ao MQTT Broker!");
      pulseColor(CRGB::Green);
    } else {
      pulseColor(CRGB::Magenta);
      Serial.print("Falha na conexão. Estado MQTT: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// Controla o NL do AD0 de cada MPU
void selecionaacel(int acel){
  if(acel == 1){
    digitalWrite(pinAcel1, LOW);
    digitalWrite(pinAcel2, HIGH);
    digitalWrite(pinAcel3, HIGH);
    digitalWrite(pinAcel4, HIGH);
    digitalWrite(pinAcel5, HIGH);
//    Serial.println("ACELEROMETRO 1 SELECIONADO");
  }
  else if(acel == 2){
    digitalWrite(pinAcel1, HIGH);
    digitalWrite(pinAcel2, LOW);
    digitalWrite(pinAcel3, HIGH);
    digitalWrite(pinAcel4, HIGH);
    digitalWrite(pinAcel5, HIGH);
//    Serial.println("ACELEROMETRO 2 SELECIONADO");
  }
  else if(acel == 3){
    digitalWrite(pinAcel1, HIGH);
    digitalWrite(pinAcel2, HIGH);
    digitalWrite(pinAcel3, LOW);
    digitalWrite(pinAcel4, HIGH);
    digitalWrite(pinAcel5, HIGH);
//    Serial.println("ACELEROMETRO 3 SELECIONADO");
  }
  else if(acel == 4){
    digitalWrite(pinAcel1, HIGH);
    digitalWrite(pinAcel2, HIGH);
    digitalWrite(pinAcel3, HIGH);
    digitalWrite(pinAcel4, LOW);
    digitalWrite(pinAcel5, HIGH);
//    Serial.println("ACELEROMETRO 4 SELECIONADO");
  }
  else if(acel == 5){
    digitalWrite(pinAcel1, HIGH);
    digitalWrite(pinAcel2, HIGH);
    digitalWrite(pinAcel3, HIGH);
    digitalWrite(pinAcel4, HIGH);
    digitalWrite(pinAcel5, LOW);
//    Serial.println("ACELEROMETRO 5 SELECIONADO");
  }
}
void calculaoffset(int leituras, int acel) {
  for (int nR = 0; nR < leituras; nR++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Converte para segundos
    lastTime = currentTime;

    float accelX = a.acceleration.x / 16384.0;
    float accelY = a.acceleration.y / 16384.0;
    float accelZ = a.acceleration.z / 16384.0;
    
    AErroX += (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI);
    AErroY += (atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI);
    GErroZ += g.gyro.z;
  }
  
  offsetaX = AErroX / (float)leituras;
  offsetaY = AErroY / (float)leituras;
  offsetgZ = GErroZ / (float)leituras;

  offset[0][acel-1] = offsetaX;
  offset[1][acel-1] = offsetaY;
  offset[2][acel-1] = offsetgZ;
}
void calibra(){
  selecionaacel(1);
  calculaoffset(numleituras, 1);
  selecionaacel(2);
  calculaoffset(numleituras, 2);
  selecionaacel(3);
  calculaoffset(numleituras, 3);
  selecionaacel(4);
  calculaoffset(numleituras, 4);
}
// Realiza a leitura dos dados do MPU
void leitura(int acel){
  // Inicializa o MPU6050
  if (!mpu.begin(0x68)) { // Endereço I2C padrão do MPU6050
    Serial.println("Falha ao inicializar o MPU6050!");
  }else{
    //Serial.println("MPU6050 inicializado com sucesso!");
    // Atualiza os dados do sensor
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Converte para segundos
    lastTime = currentTime;

    // Lê a taxa de rotação no eixo Z (em graus/s)
    gyroZ = (g.gyro.z - offset[2][acel-1]) * 57.2958; // Converte de rad/s para graus/s
    
    angleZ += gyroZ * deltaTime;
    
    float accelX = a.acceleration.x / 16384.0;
    float accelY = a.acceleration.y / 16384.0;
    float accelZ = a.acceleration.z / 16384.0;
    
    angleX = (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI)- offset[0][acel-1];
    angleY = (atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI)- offset[1][acel-1];
    matriz[0][acel-1] = angleX;
    matriz[1][acel-1] = angleY;
    matriz[2][acel-1] = angleZ;
    
    delay(100);
  } 
}

void publicamatriz(){
  Serial.println("==================");
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 5; j++){
      if(matriz[i][j]<100){
        Serial.print(" ");
      }
      if(matriz[i][j]<10){
        Serial.print(" ");
      }
      if(matriz[i][j]>0){
        Serial.print(" ");
      }
      Serial.print(matriz[i][j]);
      Serial.print(" ");  
    }
    Serial.println(" ");
  }
  char mensagem1[40];
  char mensagem2[40];
  char mensagem3[40];
  char mensagem4[40];
  char mensagem5[40];
  angm1 = 0;
  angm2 = 180-(matriz[0][1]);
  angm3 = 180-(matriz[0][2]);
  angm4 = matriz[1][3];
  angm5 = matriz[1][3];
  dtostrf(angm1,2,2,mensagem1);
  dtostrf(angm2,2,2,mensagem2);
  dtostrf(angm3,2,2,mensagem3);
  dtostrf(angm4,2,2,mensagem4);
  dtostrf(angm5,2,2,mensagem5);
  client.publish(topic1, mensagem1);
  client.publish(topic2, mensagem2);
  client.publish(topic3, mensagem3);
  client.publish(topic4, mensagem4);
  client.publish(topic5, mensagem5);
  client.publish(topic_voltage, String(tensao).c_str());
  client.publish(topic_current, String(corrente).c_str());
  client.publish(topic_power, String(potencia).c_str());
  Serial.println("Valores de cada motor:");
  Serial.print(angm1);
  Serial.print(" ");
  Serial.print(angm2);
  Serial.print(" ");
  Serial.print(angm3);
  Serial.print(" ");
  Serial.print(angm4);
  Serial.print(" ");
  Serial.println(angm5);
}

void leituracorrente()
{
  tensao = ina219.getBusVoltage_V();
  corrente = ina219.getCurrent_mA();
  potencia = ina219.getPower_mW();
  
  matriz[0][4] = tensao;
  matriz[1][4] = corrente;
  matriz[2][4] = potencia;
}

// Função de configuração inicial
void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  Serial.begin(9600);
  delay(1000);
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(pinAcel1, OUTPUT);
  pinMode(pinAcel2, OUTPUT);
  pinMode(pinAcel3, OUTPUT);
  pinMode(pinAcel4, OUTPUT);
  pinMode(pinAcel5, OUTPUT);
  
  connect_wifi();

  connect_mqtt();

  if (!ina219.begin())
  {
    Serial.println("Falha no sensor de corrente");
  }else{
    Serial.println("Sensor de corrente inicializado");
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastTime = millis();
  calibra();
}

// Função principal que roda em loop
void loop() {
  if (!client.connected()) {
    connect_mqtt();
  }
  leituracorrente();
  selecionaacel(1);
  leitura(1);
  selecionaacel(2);
  leitura(2);
  selecionaacel(3);
  leitura(3);
  selecionaacel(4);
  leitura(4);
  publicamatriz();
  delay(5000); // Atraso entre leituras
}
