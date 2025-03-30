#include <WiFi.h>
#include <WiFiClientSecure.h>  
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>

// WiFi Credentials
const char* ssid = "Nayatel";
const char* password = "Numan2442";

// Define serial ports
HardwareSerial gpsSerial(1);  // UART1 for GPS
TinyGPSPlus gps;  // GPS object

unsigned long lastPing = 0;  // Store last ping time
const int pingInterval = 10000;  // Send a ping every 5 seconds

const char* mqtt_server = "48683b21a1994206bc2ea5c7b8e02f1b.s1.eu.hivemq.cloud";  // MQTT broker
const char* mqtt_username = "esp32";  
const char* mqtt_password = "Esp3212345";  

WiFiClientSecure espClient;
PubSubClient client(espClient);

#define LED_BUILTIN 2  // ESP32 onboard LED

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // GPS (RX=16, TX=17)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED ON (WiFi not connected)

  // Connect to WiFi
  connectWifi();  

  // Setup MQTT
  espClient.setInsecure();  // For accepting unsecure connections => only set for testing
  client.setServer(mqtt_server, 8883);  // MQTT uses port 1883
  client.setCallback(callback);  // Function to handle incoming messages
}

void loop() {
  client.loop();  // Keep MQTT connection alive 

  if (millis() - lastPing > pingInterval) {
    lastPing = millis();
    publishGpsData();  
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
void connectMQTT() {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(1000));  // Create a unique client ID
    const char* clientIdChar = clientId.c_str();
if (client.connect(clientIdChar, mqtt_username, mqtt_password)) {
    Serial.println("MQTT connected!");
    client.subscribe("esp32/gps");
    Serial.println("Subscribed to esp32/gps");  // ✅ Debugging Log
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

  if (msg) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  } 
}

void publishGpsData() {
  unsigned long startTime = millis(); // Record start time
  bool gpsDataReceived = false;

  if (!client.connected()) {
    connectMQTT();
  }

  // Read GPS data for max 1 second
  while (millis() - startTime < 1000) {  // Keep reading for 1 sec
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());  // Process GPS data

      if (gps.location.isValid()) {  // If GPS data is valid
        gpsDataReceived = true;
        break;  // Exit inner loop
      }
    }
    if (gpsDataReceived) break;  // Exit outer loop if valid data is received
  }

  if (gpsDataReceived) {
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["timestamp"] = millis();
    jsonDoc["latitude"] = gps.location.lat();
    jsonDoc["longitude"] = gps.location.lng();

    char buffer[256];
    serializeJson(jsonDoc, buffer);

    client.publish("esp32/gps", buffer);
  } else {
    Serial.println("No valid GPS data received.");
  }
}



// void getGpsData() {
//   while (gpsSerial.available() > 0) {
//         gps.encode(gpsSerial.read());  // Parse GPS data
//         if (gps.location.isValid()) {
//           if (gps.location.isUpdated()) {  // New GPS data available
//             Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
//             Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
//             Serial.print("Speed (km/h): "); Serial.println(gps.speed.kmph());
//             Serial.print("Altitude (m): "); Serial.println(gps.altitude.meters());
//             Serial.print("Satellites: "); Serial.println(gps.satellites.value());
//             Serial.println("----------------------");
//           }
//         }
//     }
// }
