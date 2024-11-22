#include <WiFi.h>              // Biblioteca para conectar o Wi-Fi no ESP32-S3
#include <PubSubClient.h>      // Biblioteca para o protocolo MQTT
#include <Wire.h>
#include <MPU6050.h>

#define MPU6050_ADDRESS 0x68

MPU6050 mpu(MPU6050_ADDRESS);
// Variáveis MPU
int16_t ax, ay, az;  // Acelerômetro
int16_t gx, gy, gz;  // Giroscópio
float elapsedTime, currentTime, previousTime;
float pitch, roll, yaw;
int angm1, angm2, angm3, angm4, angm5;
float gyroAngleX, gyroAngleY, gyroAngleZ;
int matriz[3][4];

// Defina os pinos de saída (ajuste de acordo com seu layout de pinos no ESP32-S3)
const int pinAcel1 = 20;
const int pinAcel2 = 21;
const int pinAcel3 = 47;
const int pinAcel4 = 48;
const int pinAcel5 = 45;

// Defina os pinos SDA e SCL para o ESP32-S3 (ajuste conforme seu setup)
const int I2C_SDA = 8;
const int I2C_SCL = 9;


// Configurações do Wi-Fi
const char* ssid = "Luiz";
const char* password = "luiz1234";

// Configurações do MQTT
const char* mqtt_broker = "test.mosquitto.org";
const char* topic1 = "Eixo1";
const char* topic2 = "Eixo2";
const char* topic3 = "Eixo3";
const char* topic4 = "Eixo4";
const char* topic5 = "Eixo5";

// Definindo o cliente Wi-Fi e MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Função para conectar ao Wi-Fi
void connect_wifi() {
  Serial.println();
  Serial.print("Conectando ao Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Conectado ao Wi-Fi!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// Função para conectar ao broker MQTT
void connect_mqtt() {
  client.setServer(mqtt_broker, 1883);

  while (!client.connected()) {
    Serial.print("Conectando ao MQTT Broker...");
    if (client.connect("esp32s3_client")) {
      Serial.println("Conectado ao MQTT Broker!");
    } else {
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

// Realiza a leitura dos dados do MPU
void leitura(int acel){
  mpu.initialize();

  // Verifica a conexão do MPU6050
  if (!mpu.testConnection()) {
    Serial.println("Falha na conexão com o MPU6050");
  }else{
//    Serial.println("MPU6050 Conectado");  
    // Leitura do acelerômetro e giroscópio
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int accAngleX = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI);
    int accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI);

    previousTime = currentTime;
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000.0f;
    
    gyroAngleX += gx * elapsedTime;
    gyroAngleY += gy * elapsedTime;
    gyroAngleZ += gz * elapsedTime;
    
    // Calculando os ângulos em relação aos eixos X, Y e Z
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    yaw = gyroAngleZ;
    // Imprimindo os ângulos X (Roll), Y (Pitch) e Z (Yaw)
//    Serial.print("Angulo X y z ");
//    Serial.print(roll);
//    Serial.print(" ");
//    Serial.print(pitch);
//    Serial.print(" ");
//    Serial.println(yaw);
    matriz[0][acel-1] = roll;
    matriz[1][acel-1] = pitch;
    matriz[2][acel-1] = yaw;
    // Publica uma mensagem no tópico1
  
    // Atraso para leitura contínua
    delay(100);
  } 
}

void publicamatriz(){
  Serial.println("==================");
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 4; j++){
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
// Função de configuração inicial
void setup() {
  
  Serial.begin(9600);
  
  // Inicializa a comunicação I2C nos pinos definidos para ESP32-S3
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configura os pinos como saídas
  pinMode(pinAcel1, OUTPUT);
  pinMode(pinAcel2, OUTPUT);
  pinMode(pinAcel3, OUTPUT);
  pinMode(pinAcel4, OUTPUT);
  pinMode(pinAcel5, OUTPUT);
  
  // Conecta ao Wi-Fi
  connect_wifi();

  // Conecta ao MQTT
  connect_mqtt();
}

// Função principal que roda em loop
void loop() {
  if (!client.connected()) {
    connect_mqtt();
  }

  selecionaacel(1);
  leitura(1);
  selecionaacel(2);
  leitura(2);
  selecionaacel(3);
  leitura(3);
  selecionaacel(4);
  leitura(4);
//  selecionaacel(5);
//  leitura(5);
  publicamatriz();
  delay(5000); // Atraso de 5 segundos
}
