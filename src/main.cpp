#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Update.h>
#include <BluetoothSerial.h>
#define  LED_BUILTIN 2
// --- WiFi & OTA Configuration ---
const char* ssid = "Xiaomi_6569";
const char* password = "8076382852";
const char* firmwareUrl = "https://raw.githubusercontent.com/arpit2512/railway_iot/refs/heads/main/.pio/build/esp32doit-devkit-v1/firmware.bin";

const char* mqtt_server = "127.0.0.1";
const int mqtt_port = 1883;

float temperature = 0;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

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
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  if (!SerialBT.begin("RAVI_OTA_Trigger")) {
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

  long now = millis();
  if (now - lastMsg > 5000) {
    // Temperature in Celsius
    char tempString[8];
    temperature = random(20, 30); // Simulated temperature value
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);
  }







  static unsigned long previousMillis = 0;
  const long interval = 500; // Blink interval in ms

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    static int ledState = LOW;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
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
        Serial.println("OTA command recognized. Disabling Bluetooth to free memory...");
        SerialBT.println("OK. Disabling Bluetooth and starting update...");
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
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");

  
  while (WiFi.status() != WL_CONNECTED ) {

    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection timed out after 10 seconds!");
  }
  Serial.println("\nWiFi Connected!");
  


  HTTPClient http;
  
  // Begin the request
  http.begin(firmwareUrl);
  
  // Set to follow redirects
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  // Send the request
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    // Content-Length for the firmware
    int len = http.getSize();

    // Pointer to the stream
    WiFiClient *stream = http.getStreamPtr();

    // Start OTA update
    if (Update.begin(len)) {
      Serial.println("Starting OTA...");
      
      // Write firmware to flash
      size_t written = Update.writeStream(*stream);

      if (written == len) {
        Serial.println("Firmware written successfully.");
      } else {
        Serial.printf("Only wrote %d/%d bytes.\n", written, len);
      }

      // Finish update
      if (Update.end()) {
        Serial.println("OTA completed!");
        if (Update.isFinished()) {
          Serial.println("Rebooting...");
         
          delay(3000);
          ESP.restart();
        } else {
         
          delay(10000);
          

 // Switch back to main page
          Serial.println("OTA not fully completed.");
        }
      } else {
        
        delay(10000);
        
 // Switch back to main page 
        Serial.printf("OTA Error: %d\n", Update.getError());
      }
    } else {
      
      delay(10000);
     
    
 // Switch back to main page 
      Serial.println("Not enough space for OTA.");
    }
  } else {
   
    delay(10000);
   
 // Switch back to main page 
    Serial.printf("HTTP request failed with code: %d\n", httpCode);
  }

  // End HTTP connection
  http.end();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
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

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
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
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}