#include <WiFi.h>
#include <PubSubClient.h>

// WiFi Credentials
const char* ssid = "Realme";
const char* password = "45677654";
unsigned long lastPing = 0;  // Store last ping time
const int pingInterval = 5000;  // Send a ping every 5 seconds

const char* mqtt_server = "broker.hivemq.com";  // Free MQTT broker

WiFiClient espClient;
PubSubClient client(espClient);

#define LED_BUILTIN 2  // ESP32 onboard LED

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED ON (WiFi not connected)

  // Connect to WiFi
  connectWifi();

  // Setup MQTT
  client.setServer(mqtt_server, 1883);  // MQTT uses port 1883
  client.setCallback(callback);  // Function to handle incoming messages
}

void loop() {
  if (!client.connected()) {
    Serial.println("Connection to MQTT server is lost");
    delay(5000);
    reconnectMQTT();
  }
  client.loop();  // Keep MQTT connection alive 

  if (millis() - lastPing > pingInterval) {
    lastPing = millis();
    Serial.println("Publishing test message...");
    client.publish("esp32/test", "ON");
  }

}

// Function to Connect to WiFi
void connectWifi() {
  Serial.println("\nConnecting to WiFi...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 30) {
    delay(500);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println("\nWiFi connection failed! Retrying in 5 seconds...");
    delay(5000);
    connectWifi();
  }
}

// Function to Connect to MQTT Broker
void reconnectMQTT() {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(String("ESP32Client-" + String(random(1000))).c_str())) {
    Serial.println("MQTT connected!");
    client.subscribe("esp32/test");
    Serial.println("Subscribed to esp32/test");  // ✅ Debugging Log
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
}

// Function to Handle Incoming MQTT Messages
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Convert message to String
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }

  Serial.println(msg);

  if (msg.equals("ON")) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  } 
}

