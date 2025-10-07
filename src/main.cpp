#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Update.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

#define LED_BUILTIN 2

// --- WiFi & OTA Configuration ---
const char* ssid = "Xiaomi_6569";
const char* password = "8076382852";
const char* firmwareUrl = "https://raw.githubusercontent.com/arpit2512/railway_iot/refs/heads/main/.pio/build/esp32doit-devkit-v1/firmware.bin";

// --- MQTT Configuration ---
const char* mqtt_server = "127.0.0.1"; // Replace with your MQTT broker's IP
const int mqtt_port = 1883;
const char* mqtt_user = "";          // <-- ADDED: Your MQTT username
const char* mqtt_token = "aTcqY3tKCrelGjFULGRI";   // <-- ADDED: Your token acts as the password

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

// --- Bluetooth Configuration ---
BluetoothSerial SerialBT;
String bt_command = "";

// --- Pin Configuration ---
const int LED_PIN = 2;

// --- Function Declarations ---
void setup_wifi();
void performOTA();
void handleBluetoothCommands();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();

//================================================================================
// SETUP FUNCTION
//================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (!SerialBT.begin("ESP32_OTA_Trigger")) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth active. Send 'START_OTA' to begin update.");
  }
}

//================================================================================
// MAIN LOOP
//================================================================================
void loop() {
  handleBluetoothCommands();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // --- Multi-Sensor Publishing Logic ---
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    JsonDocument doc;
    doc["temperature"] = random(2000, 3001) / 100.0;
    doc["pressure"] = random(98000, 102001) / 100.0;
    doc["humidity"] = random(400, 601) / 10.0;
    doc["gas"] = random(50, 501) / 10.0;

    char jsonBuffer[128];
    serializeJson(doc, jsonBuffer);

    Serial.print("Publishing sensor data: ");
    Serial.println(jsonBuffer);
    client.publish("esp32/sensors", jsonBuffer);
  }

  // --- LED Blinking Logic (Status Indicator) ---
  static unsigned long previousMillis = 0;
  if (millis() - previousMillis >= 500) {
    previousMillis = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

//================================================================================
// HELPER FUNCTIONS
//================================================================================

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == '\n') {
      bt_command.trim();
      Serial.printf("Bluetooth command received: '%s'\n", bt_command.c_str());

      if (bt_command.equalsIgnoreCase("START_OTA")) {
        Serial.println("OTA command recognized. Disabling Bluetooth...");
        SerialBT.println("OK. Starting update...");
        delay(1000);
        SerialBT.end();
        performOTA();
      } else {
        String msg = "Unknown command: '" + bt_command + "'";
        Serial.println(msg);
        SerialBT.println(msg);
      }
      bt_command = "";
    } else {
      bt_command += incomingChar;
    }
  }
}

void performOTA() {
  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Re-establishing WiFi for OTA...");
      setup_wifi();
  }
  
  Serial.println("Checking for firmware updates...");
  HTTPClient http;
  http.begin(firmwareUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int len = http.getSize();
    if (Update.begin(len)) {
      Serial.println("Starting OTA...");
      WiFiClient* stream = http.getStreamPtr();
      size_t written = Update.writeStream(*stream);
      
      if (written == len) {
        Serial.println("Firmware written successfully.");
        if (Update.end(true)) { // true to set the boot partition
          Serial.println("OTA finished! Rebooting...");
          delay(2000);
          ESP.restart();
        }
      }
    }
  } else {
    Serial.printf("HTTP request failed, code: %d\n", httpCode);
  }
  http.end();
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
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

  if (String(topic) == "esp32/output") {
    if(messageTemp == "on"){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if(messageTemp == "off"){
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection with token...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    // --- MODIFIED: Attempt to connect using the username and token ---
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_token)) {
      Serial.println("connected");
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // MQTT Return Codes:
      // 4: Bad user name or password (token) <-- Check your credentials!
      // 5: Not authorized
      delay(5000);
    }
  }
}