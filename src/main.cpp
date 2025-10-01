#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <BluetoothSerial.h>

// --- WiFi & OTA Configuration ---
const char* ssid = "hotspot";
const char* password = "12345678";
const char* firmwareUrl = "https://raw.githubusercontent.com/arpit2512/railway_iot/78076d1e9419c5e8c318ea1b49c06355cc39dd59/.pio/build/esp32doit-devkit-v1/firmware.bin";

// --- Bluetooth Configuration ---
BluetoothSerial SerialBT;
String bt_command = "";

// --- Pin Configuration ---
const int LED_PIN = 2;

// --- Function Declarations ---
void setup_wifi();
void performOTA();
void handleBluetoothCommands();

//================================================================================
// SETUP FUNCTION
//================================================================================
void setup() {
  Serial.begin(115200);



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
      Serial.println("sStarting OTA...");
      
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