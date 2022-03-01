#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_now.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "Phi Hung";
const char* password = "26022000";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.1.21";

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
  uint8_t code;
} espnow_msg;

espnow_msg send_msg;
espnow_msg recv_msg;
esp_now_peer_info_t peerInfo;

WiFiClient espClient;
PubSubClient client(espClient);
void callback(char* topic, byte* message, unsigned int length);
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len);
void setup_wifi();
void setup_espnow();
void reconnect();

void setup() {
  Serial.begin(115200);
  setup_wifi();
  setup_espnow();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.channel());
  Serial.println(WiFi.macAddress());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    if(messageTemp == "toggle"){
      digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN)?0:1);
      if(digitalRead(LED_BUILTIN)) {
        client.publish("esp32/outputstatus","ON");
      } else {
        client.publish("esp32/outputstatus","OFF");
      }
    }
  }
}

void setup_espnow() {
  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
      client.subscribe("esp32/airoutput");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&recv_msg,incomingData,sizeof(recv_msg));
  if(recv_msg.code)
    client.publish("esp32/airoutputstatus","ON");
    else client.publish("esp32/airoutputstatus","OFF");
}