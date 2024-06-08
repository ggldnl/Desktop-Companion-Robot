#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <U8g2lib.h>

// WiFi credentials
const char* ssid = "ERR_EMPTY_RESPONSE";
const char* password = "[Pr0tocol_3rror]={}";

// Server settings
WiFiServer server(12345);
WiFiClient client;

// OLED display settings
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
    Serial.begin(115200);
    
    // Initialize the OLED display
    u8g2.begin();

    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("connected");

    // Start the server
    server.begin();
    Serial.println("Server started");
}

void loop() {
    // Check if a client has connected
    client = server.available();
    if (client) {
        Serial.println("Client connected");
        uint8_t displayBuffer[1024];  // Buffer to hold incoming data
        int bytesRead = 0;

        // Read data from the client
        while (bytesRead < sizeof(displayBuffer)) {
            if (client.available()) {
                bytesRead += client.read(displayBuffer + bytesRead, sizeof(displayBuffer) - bytesRead);
            } else {
                delay(1); // Small delay to yield to other processes
            }
        }

        if (bytesRead == sizeof(displayBuffer)) {
            // Display data on the OLED
            Serial.println("Update display");
            u8g2.clearBuffer();
            u8g2.drawXBMP(0, 0, 128, 64, displayBuffer);
            u8g2.sendBuffer();
        }

        client.stop();
        Serial.println("Client disconnected");
    }
}
