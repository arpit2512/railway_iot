#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
const char* ssid = "hotspot";
const char* password = "12345678";
const char* firmwareUrl = "https://raw.githubusercontent.com/arpit2512/esp32dev/refs/heads/main/firmware.bin";


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
        
// ADD GREEN LED ON
 // Switch back to main page
          Serial.println("OTA not fully completed.");
        }
      } else {
        
        delay(1000);
        
// ADD GREEN LED ON
 // Switch back to main page
        Serial.printf("OTA Error: %d\n", Update.getError());
      }
    } else {
    
      delay(1000);
      
// ADD GREEN LED ON
 // Switch back to main page 
      Serial.println("Not enough space for OTA.");
    }
  } else {
   
    delay(10000);
   
// ADD GREEN LED ON
 // Switch back to main page 
    Serial.printf("HTTP request failed with code: %d\n", httpCode);
  }

  // End HTTP connection
  http.end();
}

