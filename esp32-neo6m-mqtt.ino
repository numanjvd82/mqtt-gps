#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <NTPClient.h>
#include <time.h>


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

// NTP Setup (for accurate timestamps)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // UTC time, updates every 60s

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
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);

  timeClient.begin();  
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
    String clientId = "ESP32Client-" + String(random(1000)); 
    const char* clientIdChar = clientId.c_str();
if (client.connect(clientIdChar, mqtt_username, mqtt_password)) {
    Serial.println("MQTT connected!");
    client.subscribe("esp32/gps");
    Serial.println("Subscribed to esp32/gps"); 
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
  timeClient.update();  // Update time from NTP server
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
    jsonDoc["timestamp"] = getFormattedTime();
    jsonDoc["latitude"] = gps.location.lat();
    jsonDoc["longitude"] = gps.location.lng();

    char buffer[256];
    serializeJson(jsonDoc, buffer);

    client.publish("esp32/gps", buffer);
  } else {
    Serial.println("No valid GPS data received.");
  }
}

// Function to format time as "YYYY-MM-DD HH:MM:SS"
String getFormattedTime() {
    time_t rawTime = timeClient.getEpochTime();  // Get UNIX timestamp
    struct tm *timeInfo = localtime(&rawTime);  // Convert to readable format

    char formattedTime[25];
    sprintf(formattedTime, "%04d-%02d-%02dT%02d:%02d:%02dZ", 
            timeInfo->tm_year + 1900, 
            timeInfo->tm_mon + 1, 
            timeInfo->tm_mday, 
            timeInfo->tm_hour, 
            timeInfo->tm_min, 
            timeInfo->tm_sec);

    return String(formattedTime);
}
