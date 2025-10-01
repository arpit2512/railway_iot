#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>

// --- WiFi & OTA Configuration ---
const char* ssid = "hotspot";
const char* password = "12345678";
const char* firmwareUrl = "https://raw.githubusercontent.com/arpit2512/railway_iot/78076d1e9419c5e8c318ea1b49c06355cc39dd59/.pio/build/esp32doit-devkit-v1/firmware.bin";

// --- MQTT Configuration ---
const char* mqtt_server = "192.168.1.100"; // IMPORTANT: Change to your local MQTT broker's IP
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/analog/readings";
const char* mqtt_client_id = "esp32-analog-sender";

// --- Pin Configuration ---
const int analogPins[] = {34, 35, 32, 33, 39};
const int NUM_PINS = sizeof(analogPins) / sizeof(analogPins[0]);
const int GREEN_LED_PIN = 2; // Built-in LED on many ESP32 boards

// --- Global Variables ---
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsgTime = 0;
const long msgInterval = 5000; // Publish data every 5 seconds

// --- BT --- Global objects for Bluetooth Serial
BluetoothSerial SerialBT;
String bt_command = ""; // A string to store the incoming command

// --- Function Declarations ---
void setup_wifi();
void performOTA();
void reconnect_mqtt();
void publishAnalogReadings();
void handleBluetoothCommands();

//================================================================================
// SETUP FUNCTION
//================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);

  delay(1000);

  setup_wifi(); // Connect to WiFi

  client.setServer(mqtt_server, mqtt_port); // Configure MQTT client

  // --- BT --- Start the Bluetooth Serial service with a specific name
  if (!SerialBT.begin("ESP32_OTA_Updater")) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth device active, ready to receive commands.");
  }
}

//================================================================================
// MAIN LOOP
//================================================================================
void loop() {
  handleBluetoothCommands();

  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  unsigned long currentTime = millis();
  if (currentTime - lastMsgTime >= msgInterval) {
    lastMsgTime = currentTime;
    publishAnalogReadings();
  }
}

//================================================================================
// HELPER FUNCTIONS
//================================================================================

/**
 * @brief --- BT --- Checks for incoming data from Bluetooth Serial and processes it.
 */
void handleBluetoothCommands() {
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == '\n') {
      bt_command.trim();
      Serial.printf("Bluetooth command received: '%s'\n", bt_command.c_str());

      if (bt_command.equalsIgnoreCase("START_OTA")) {
        Serial.println("OTA command recognized. Starting update process...");
        SerialBT.println("OK. OTA starting. Device will disconnect and reboot upon success.");
        delay(1000); // Give a moment for the BT message to be sent
        performOTA();
      } else {
        Serial.printf("Unknown command: '%s'\n", bt_command.c_str());
        SerialBT.printf("Unknown command: '%s'\n", bt_command.c_str());
      }
      bt_command = "";
    } else {
      bt_command += incomingChar;
    }
  }
}

/**
 * @brief Connects to the WiFi network.
 */
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 20;
  while (WiFi.status() != WL_CONNECTED && retries > 0) {
    delay(500);
    Serial.print(".");
    retries--;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection FAILED! Rebooting...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief Reconnects to the MQTT broker with a back-off delay.
 */
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/**
 * @brief Reads analog pins, formats data as JSON, and publishes to MQTT.
 */
void publishAnalogReadings() {
  JsonDocument doc;
  for (int i = 0; i < NUM_PINS; i++) {
    int pin = analogPins[i];
    doc["pin" + String(pin)] = analogRead(pin);
  }

  String jsonString;
  serializeJson(doc, jsonString);

  if (client.publish(mqtt_topic, jsonString.c_str())) {
    Serial.print("Published to topic ");
    Serial.print(mqtt_topic);
    Serial.print(": ");
    Serial.println(jsonString);
  } else {
    Serial.println("MQTT publish failed!");
  }
}


/**
 * @brief Performs an Over-The-Air (OTA) firmware update with detailed progress reporting over Bluetooth.
 */
void performOTA() {
  if (WiFi.status() != WL_CONNECTED) {
    String msg = "Error: WiFi not connected. Cannot start OTA.";
    Serial.println(msg);
    SerialBT.println(msg);
    return;
  }

  String msg = "Checking for firmware updates...";
  Serial.println(msg);
  SerialBT.println(msg);

  HTTPClient http;
  http.begin(firmwareUrl);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    String err_msg = "Error: HTTP request failed with code " + String(httpCode);
    Serial.println(err_msg);
    SerialBT.println(err_msg);
    http.end();
    return;
  }
  
  int len = http.getSize();
  if (len <= 0) {
    String err_msg = "Error: Invalid firmware size on server.";
    Serial.println(err_msg);
    SerialBT.println(err_msg);
    http.end();
    return;
  }

  msg = "Connected to server. Firmware size: " + String(len) + " bytes.";
  Serial.println(msg);
  SerialBT.println(msg);

  if (!Update.begin(len)) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    String err_msg = "Error: Not enough space for OTA (" + String(Update.getError()) + ")";
    Serial.println(err_msg);
    SerialBT.println(err_msg);
    http.end();
    return;
  }

  msg = "Starting download and flash...";
  Serial.println(msg);
  SerialBT.println(msg);

  WiFiClient* stream = http.getStreamPtr();
  
  // --- Manual download loop to report progress ---
  size_t written = 0;
  uint8_t buff[1024] = { 0 };
  int last_progress = -1;

  while (written < len) {
    // Read a chunk of data from the stream
    size_t size = stream->readBytes(buff, sizeof(buff));
    if (size == 0) {
      // Stream timeout
      break; 
    }
    
    // Write the chunk to the flash
    if (Update.write(buff, size) != size) {
        String err_msg = "Error: Flash write failed during update.";
        Serial.println(err_msg);
        SerialBT.println(err_msg);
        Update.abort(); // Abort the update
        http.end();
        return;
    }
    written += size;

    // Calculate and report progress
    int progress = (written * 100) / len;
    if (progress > last_progress) { // Only report on change
        Serial.printf("Progress: %d%%\n", progress);
        SerialBT.printf("Progress: %d%%\n", progress);
        last_progress = progress;
    }
  }

  if (written != len) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    String err_msg = "Error: Firmware download incomplete. Wrote " + String(written) + "/" + String(len) + " bytes.";
    Serial.println(err_msg);
    SerialBT.println(err_msg);
    Update.abort();
    http.end();
    return;
  }

  if (!Update.end(true)) { // true to set the sketch to bootable
    digitalWrite(GREEN_LED_PIN, HIGH);
    String err_msg = "Error: Failed to finalize update (" + String(Update.getError()) + ")";
    Serial.println(err_msg);
    SerialBT.println(err_msg);
    return;
  }

  msg = "Update successful! Rebooting now...";
  Serial.println(msg);
  SerialBT.println(msg);
  delay(1000);
  ESP.restart();
}