#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ThingSpeak.h>

const char* ssid = "MFIoT";
const char* password = "12345678";

const char* mqtt_broker = "192.168.137.1";

const char* topicPubTemp = "GRP5/TEMPERATURA";
const char* topicPubNivel = "GRP5/NIVEL";

const char* topicSubGrp1Temp = "GRP1/TEMP";
const char* topicSubGrp1Nivel = "GRP1/NIVEL";
const char* topicSubGrp2Temp = "GRP2/TEMP";
const char* topicSubGrp2Nivel = "GRP2/NIVEL";

const char* mqtt_username = "";
const char* mqtt_password = "";
const int mqtt_port = 1883;

bool mqttStatus = false;
float Temp = 0.0;
float Nivel = 0.0;
String MsgTemp;
String MsgNivel;

float TempPlaca1 = 0.0;
float NivelPlaca1 = 0.0;
float TempPlaca2 = 0.0;
float NivelPlaca2 = 0.0;

float TempMedia = 0.0;
float NivelMedio = 0.0;

WiFiClient espClient;
PubSubClient client(espClient);

const int trigPin = 4;
const int echoPin = 18;
const int releBomba = 5;

#define SOUND_SPEED 0.034

void setupWiFi();
bool conectaMQTT();
void bombaFluxo(bool estado);
float medirDistanciaCM();
void callback(char *topic, byte * payload, unsigned int length);

void setup() {
  pinMode(releBomba, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  Serial.begin(9600);

  setupWiFi();
  mqttStatus = conectaMQTT();
}

void loop() {
  static long long pooling = 0;

  if (mqttStatus) {
    client.loop();

    if (millis() > pooling + 5000) {
      pooling = millis();

      Temp = random(0, 1000) / 10.0;
      Nivel = random(0, 3000) / 10.0;
      MsgTemp = String(Temp);
      MsgNivel = String(Nivel);

      client.publish(topicPubTemp, MsgTemp.c_str());
      client.publish(topicPubNivel, MsgNivel.c_str());

      Serial.println("Mensagens publicadas:");
      Serial.println("Temperatura: " + MsgTemp + " °C");
      Serial.println("Nível: " + MsgNivel + " cm");
    }
  }
}

void bombaFluxo(bool estado) {
  digitalWrite(releBomba, estado ? HIGH : LOW);
}

float medirDistanciaCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return duration * SOUND_SPEED / 2;
}

void setupWiFi() {
  Serial.print("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempts > 20) {
      Serial.println("\nFalha ao conectar no Wi-Fi. Reiniciando...");
      ESP.restart();
    }
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

bool conectaMQTT() {
  byte tentativa = 0;
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  do {
    String client_id = "ESP-";
    client_id += String(WiFi.macAddress());

    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.printf("Cliente %s conectado ao broker\n", client_id.c_str());
    } else {
      Serial.print("Falha ao conectar: ");
      Serial.println(client.state());
      Serial.print("Tentativa: ");
      Serial.println(tentativa);
      delay(2000);
    }
  } while (!client.connected() && ++tentativa < 5);

  if (client.connected()) {
    client.subscribe(topicPubTemp);
    client.subscribe(topicPubNivel);
    client.subscribe(topicSubGrp2Temp);
    client.subscribe(topicSubGrp2Nivel);
    return true;
  } else {
    Serial.println("MQTT Não conectado");
    return false;
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Mensagem chegou no tópico: ");
  Serial.println(topic);

  String mensagemString = "";
  for (unsigned int i = 0; i < length; i++) {
    mensagemString += (char)payload[i];
  }

  float valorFloat = mensagemString.toFloat();

  if (strcmp(topic, topicPubTemp) == 0) {
    TempPlaca1 = valorFloat;
    Serial.print("Temperatura da Placa 1 atualizada para: ");
    Serial.println(TempPlaca1);
  } else if (strcmp(topic, topicPubNivel) == 0) {
    NivelPlaca1 = valorFloat;
    Serial.print("Nível da Placa 1 atualizado para: ");
    Serial.println(NivelPlaca1);
  } else if (strcmp(topic, topicSubGrp2Temp) == 0) {
    TempPlaca2 = valorFloat;
    Serial.print("Temperatura da Placa 2 atualizada para: ");
    Serial.println(TempPlaca2);
  } else if (strcmp(topic, topicSubGrp2Nivel) == 0) {
    NivelPlaca2 = valorFloat;
    Serial.print("Nível da Placa 2 atualizado para: ");
    Serial.println(NivelPlaca2);
  }

  Serial.print("Mensagem (Float): ");
  Serial.println(valorFloat);
  Serial.println("------------------------");
}
