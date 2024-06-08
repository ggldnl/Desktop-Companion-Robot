#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <map>
#include "message.h"


// Wifi settings
const char *ssid = "ERR_EMPTY_RESPONSE";
const char *password = "[Pr0tocol_3rror]={}";
const unsigned int localPort = 5005;

// Data transfer protocol
WiFiUDP udp;
const int packetSize = 512;
std::map<uint8_t, Message> message_map;

// Create an instance of the display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {

  Serial.begin(115200);
  delay(1000);

  // Connect to WiFi
  Serial.print("Connecting to: ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.print("Connected to: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP server
  udp.begin(localPort);
  Serial.printf("UDP server started at port %d\n", localPort);

  // Initialize the display
  u8g2.begin();
}


void loop() {

  byte packetBuffer[packetSize]; // Buffer to hold incoming packet
  int packetSize = udp.parsePacket();

  if (packetSize) {

    // Serial.printf("Received packet of size %d\n", packetSize);

    // Read packet into buffer
    udp.read(packetBuffer, sizeof(packetBuffer));

    // Extract header from packet
    uint8_t messageId = packetBuffer[0];
    uint8_t packetNumber = packetBuffer[1];
    uint8_t totalPacketsNumber = packetBuffer[2];
    uint8_t width = packetBuffer[3];
    uint8_t height = packetBuffer[4];
    uint8_t* payload = packetBuffer + 5;

    // Calculate payload size
    int payloadSize = packetSize - 5;

    // Display packet information
    Serial.printf("Array ID: %d, Sequence Number: %d, Num Packets: %d, Width: %d, Height: %d, Size: %d\n", messageId, packetNumber, totalPacketsNumber, width, height, packetSize);
    
    // Check if the message is already in the map
    auto it = message_map.find(messageId);
    if (it == message_map.end()) {
      // If not present, create and add the new message
      Serial.printf("No entry with message id %d. Adding new message to the map.\n", messageId);
      Message new_message(messageId, totalPacketsNumber, totalPacketsNumber * payloadSize);
      it = message_map.emplace(messageId, std::move(new_message)).first;
    }

    // Add data to the message
    if (!it->second.addData(packetNumber, payload, payloadSize)) {
      Serial.printf("Data added to message %d: %d/%d\n", messageId, packetNumber, totalPacketsNumber);
    }
    
    if (it->second.isComplete()) {
      Serial.printf("Message %d complete!\n", messageId);
    }
    
    // Clear buffer for next packet
    memset(packetBuffer, 0, packetSize);

    delay(50);
  }
}
