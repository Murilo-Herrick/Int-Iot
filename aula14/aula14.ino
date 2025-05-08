#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ThingSpeak.h>


const char* ssid = "MFIoT";
const char* password = "12345678";

const char* mqtt_broker = "192.168.137.1";
//const char* mqttClientName = "ESP01S_client";

const char* topicPubTemp = "GRP5/TEMPERATURA";
const char* topicPubNivel = "GRP5/NIVEL";

const char* topicSubGrp1Temp = "GRP1/TEMP";
const char* topicSubGrp1Nivel = "GRP1/NIVEL";
const char* topicSubGrp2Temp = "GRP2/TEMP";
const char* topicSubGrp2Nivel = "GRP2/NIVEL";

const char* mqtt_username = "";
const char* mqtt_password = "";

const int mqtt_port = 1883;

bool mqttStatus = 0;
float Temp = 0.0;
float Nivel = 0.0;
String MsgTemp;
String MsgNivel;

float TempPlac1 = 0.0;
float NivelPlac1 = 0.0;
float TempPlac2 = 0.0;
float NivelPlac2 = 0.0;

float TempMedia = 0.0;
float NivelMedio = 0.0;

WiFiClient espClient;
PubSubClient client(espClient);

void setupWiFi();
bool connectMQTT();
void bombaFluxo(bool estado);
float obterTemperaturaCelsius();
float medirDistanciaCM();
void callback(char *topic, byte * payload, unsigned int length);

// const int pinoTemperatura = 21;
// OneWire sensorWireBus (pinoTemperatura);
// DallasTemperature sensors(&sensorOneWireBus);


const int trigPin = 5
const int echoPin = 18;

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;


const bool ON = true;
const bool OFF = false;

const int releBomba = 5;


void setup() {
  //pinMode(releBomba, OUTPUT);

  Serial.begin(9600);
  //Serial.println("Inicializando sensor DS18B20...");
  //sensors.begin();
  //delay(500);
  //Serial.print("Número de dispositivos encontrados: ");
  //Serial.println(sensors.getDeviceCount());
  //Serial.println("Sensor DS18B20 inicializado.");
  //Serial.println("Inicializando sensor HC-SR04...");
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
  //digitalWrite(trigPin, LOW);
  //Serial.println("Sensor HC-SR04 inicializado.");

  setupWiFi();
  mqttStatus = connectMQTT();
}

void loop() {
  //bombaFluxo(ON);
  //delay(500);

  //bombaFluxo(OFF);
  //delay(500);
  //float temperaturaCelsius = obterTemperaturaCelsius();
  //Serial.print("Temperatura: ");
  //Serial.print(temperaturaCelsius);
  //Serial.println(" °C");

  //delay(1000);
  //float distanciaCM = medirDistanciaCM();
  //Serial.print("Distância: ");
  //Serial.print(distanciaCM);
  //Serial.println(" cm");

  static long long pooling = 0;
  if ( mqttStatus){
    client.loop();
    if(millis() > pooling +5000){
      pooling = millis();
      Temp = random(0,1000)/10;
      Nivel = random(0,3000)/10;
      MsgTemp = String(Temp);
      MsgNivel = String(Nivel);
      client.publish(topicPubTemp, MsgTemp.c_str());
      client.publish(topicPubNivel, MsgNivel.c_str());
      Serial.println("Mensagem publicadas: " );
    }
  }
}

void bombaFluxo(bool estado){
  
  if(estado)
  {
    digitalWrite(releBomba,HIGH);
  }
  else
  {
    digitalWrite(releBomba,LOW);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Função para obter a temperatura em graus Celsius do sensor DS18B20.
// float obterTemperaturaCelsius() {
//   sensors.requestTemperatures(); // Envia o comando para iniciar a conversão de temperatura em todos os sensores DS18B20 no barramento.
//   return sensors.getTempCByIndex(0); // Retorna a temperatura em graus Celsius do primeiro sensor encontrado (índice 0).
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Função para medir a distância em centímetros usando o sensor HC-SR04.
float medirDistanciaCM() {
  // Limpa o pino Trig, garantindo um pulso LOW inicial.
  // digitalWrite(trigPin, LOW);
  // delayMicroseconds(2);
  // Define o pino Trig em estado HIGH por 10 microssegundos para gerar o pulso ultrassônico.
  // digitalWrite(trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);

  // Lê o pino Echo, retornando a duração do pulso HIGH (tempo de viagem da onda sonora) em microssegundos.
  // duration = pulseIn(echoPin, HIGH);

  // Calcula a distância em centímetros usando a fórmula: distância = (tempo * velocidade do som) / 2.
  // distanceCm = duration * SOUND_SPEED/2;

  // return distanceCm; // Retorna o valor da distância calculada em centímetros.
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Conectar ao Wi-Fi
void setupWiFi() {
  delay(10);
  Serial.print("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts > 20) {
      Serial.println("\nFalha ao conectar no Wi-Fi. Reiniciando...");
      ESP.restart();
    }
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Conectar ao broker MQTT
// Implementacao do metodo de conexao com o broker
bool conectaMQTT() {
  byte tentativa = 0; // variavel byte que contará o número de tentativas de conexao
  client.setServer(mqtt_broker, mqtt_port); // chama metodo setServer passando url e port do broker
  client.setCallback(callback); // Informa o objeto client qual metodo deve ser chamado quando houver
 // alguma mensagem no topico subscrito.
  do {
    // Define o ID do cliente (a própria placa ESP)
    String client_id = "ESP-"; // Que usa o prefixo ESP-
    client_id += String(WiFi.macAddress()); // Concatenando com seu respectivo MAC address

    // O if tenta estabelecer a conexao com o broker
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      // Com sucesso da conexao, informamos os dados do cliente (a placa)
      Serial.println("Êxito na conexão:");
      Serial.printf("Cliente %s conectado ao broker\n", client_id.c_str());
    } else {
      // Se falha na conexão e aguarda 2 segundos para nova tentativa
      Serial.print("Falha ao conectar: ");
      Serial.print(client.state());
      Serial.println();
      Serial.print("Tentativa:");
      Serial.println(tentativa);
      delay(2000);
    }
    tentativa++; // Incrementa numero de tentativas
  } while (!client.connected() && tentativa < 5); // Limita número de tentativas

  if (tentativa < 5) {
    // Conexão realizada com sucesso
    // se subscreve no broker para receber mensagens
    client.subscribe(topicPubTemp);
    client.subscribe(topicPubNivel);
    client.subscribe(topicSubGrp2Temp);
    client.subscribe(topicSubGrp2Nivel);
    return 1; // retorna 1 confirmando sucesso na conexao
  } else {
    // caso contrário avisa falha e retorna 0
    Serial.println("MQTT Não conectado");
    return 0; // informa falha na conexao
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Este metodo eh chamado quando o client identifica nova mensagem no broker
void callback(char *topic, byte *payload, unsigned int length) {
  // char *topic identifica o tópico registrado
  // byte *payload conjunto de bytes que foram publicados
  // int length é o tamanho do vetor de bytes do payload
  Serial.print("Mensagem chegou no tópico: ");
  Serial.println(topic);

  // Cria uma String a partir do payload
  String mensagemString = "";
  for (int i = 0; i < length; i++) {
    mensagemString += (char)payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Converte a String para um float
  float valorFloat = mensagemString.toFloat();

  // PLACA 01 EXTERNA
  if (strcmp(topic, topicPubTemp) == 0) {
    // Se os tópicos forem iguais, converte o payload para float e atualiza TempPlaca1
    TempPlaca1 = mensagemString.toFloat();
    Serial.print("Temperatura da Placa 1 atualizada para: ");
    Serial.println(TempPlaca1);
  }

  if (strcmp(topic, topicPubNivel) == 0) {
    // Se os tópicos forem iguais, converte o payload para float e atualiza TempPlaca1
    NivelPlaca1 = mensagemString.toFloat();
    Serial.print("Nível da Placa 1 atualizada para: ");
    Serial.println(NivelPlaca1);
  }
}
  // PLACA 02 EXTERNA
  if (strcmp(topic, topicSubGrp2Temp) == 0) {
    // Se os tópicos forem iguais, converte o payload para float e atualiza TempPlaca1
    TempPlaca2 = mensagemString.toFloat();
    Serial.print("Temperatura da Placa 2 atualizada para: ");
    Serial.println(TempPlaca1);
  }

  if (strcmp(topic, topicSubGrp2Nivel) == 0) {
    // Se os tópicos forem iguais, converte o payload para float e atualiza TempPlaca1
    NivelPlaca2 = mensagemString.toFloat();
    Serial.print("Nível da Placa 2 atualizada para: ");
    Serial.println(TempPlaca1);
  }

  Serial.print("Mensagem (Float): ");
  Serial.println(valorFloat);
  Serial.println("------------------------");
}

