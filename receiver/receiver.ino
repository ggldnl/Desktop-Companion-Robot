#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <U8g2lib.h>

// Wifi settings
const char *ssid = "ERR_EMPTY_RESPONSE";
const char *password = "[Pr0tocol_3rror]={}";
const unsigned int localPort = 5005;

// Data transfer protocol
WiFiUDP udp;
const int maxMessages = 5; // Maximum number of messages to store

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

  byte packetBuffer[1024]; // Buffer to hold incoming packet
  int packetSize = udp.parsePacket();

  if (packetSize) {

    // Serial.printf("Received packet of size %d\n", packetSize);

    // Read packet into buffer
    udp.read(packetBuffer, sizeof(packetBuffer));

    // Extract header from packet
    int arrayId = packetBuffer[0];
    int seqNum = packetBuffer[1];
    int numPackets = packetBuffer[2];
    int width = packetBuffer[3];
    int height = packetBuffer[4];

    // Calculate payload size
    int payloadSize = packetSize - 5;

    // Display packet information
    Serial.printf("Array ID: %d, Sequence Number: %d, Num Packets: %d, Width: %d, Height: %d\n", arrayId, seqNum, numPackets, width, height);
    
    // Print packet data (for demonstration purposes)
    Serial.print("Packet Data: ");
    for (int i = 0; i < payloadSize; i++)
    {
      Serial.print(packetBuffer[5 + i]);
      Serial.print(" ");
    }
    Serial.println();

    // Process received data and plot the array (you can implement your logic here)
    // ...

    // Clear buffer for next packet
    memset(packetBuffer, 0, sizeof(packetBuffer));

  }
}
