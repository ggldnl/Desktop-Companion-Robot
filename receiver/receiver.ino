#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// WiFi credentials
const char* ssid = "ERR_EMPTY_RESPONSE";
const char* password = "[Pr0tocol_3rror]={}";

// Battery
float v;

// IMU settings
Adafruit_MPU6050 mpu;
float t;
float ax, ay, az, gx, gy, gz;

// Create a buffer to hold the serialized data to send back to the server.
// The data includes voltage, temperature, ax, ay, az, gx, gy, gz
// 8 floats, 4 bytes each, 32 bytes total
uint8_t replyBuffer[8 * sizeof(float)];

// Server settings
const int port = 12345;
WiFiServer server(port);
WiFiClient client;

// OLED display settings
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
const uint8_t displayWidth = 128;
const uint8_t displayHeight = 64;

void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize all the other stuff
  setupServer();
  setupBatteryShield();
  setupOLED();
  setupIMU();
}

void setupServer() {
  // Connect to WiFi

  WiFi.begin(ssid, password);
  Serial.print("[INFO] Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("[INFO] Connected to: ");
  Serial.println(ssid);
  Serial.print("[INFO] IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
  Serial.print("[INFO] Server listening on port: ");
  Serial.println(port);
}

void setupBatteryShield() {
  // Set A0 as INPUT in order to read battery status

  pinMode(A0, INPUT);
  Serial.println("[INFO] Battery shield initialized");
  Serial.print("[INFO] Battery level: ");
  updateBatteryLevel(v);
  Serial.println(v);
}

void setupOLED() {
  // Setup the OLED display
  
  u8g2.begin();
  Serial.println("[INFO] OLED display initialized");
}

void setupIMU() {
  // Setup the IMU

  if (!mpu.begin()) {
    Serial.println("[ERROR] Unable to find IMU");
  }
  Serial.println("[INFO] IMU Initialized");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("[INFO] Accelerometer range set to +- 8G");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("[INFO] Gyro range set to +- 500 deg/s");

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("[INFO] Filter bandwidth set to 5 Hz");

}

void loop() {

  // Check if a client has connected 
  client = server.available();
  if (client) {
    Serial.println("[INFO] Client connected");
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
      // Serial.println("[INFO] Updating display");
      u8g2.clearBuffer();
      u8g2.drawXBMP(0, 0, displayWidth, displayHeight, displayBuffer);
      u8g2.sendBuffer();
    }

    // Reply by writing the status back to the server
    sendData();

    // TODO condition to close the connection
    if (true) {
      client.stop();
      Serial.println("[INFO] Client disconnected");
    }
  }
}

void updateBatteryLevel(float &b) {
  b = analogRead(A0) / 1024.0 * 100;
}

void updateIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &temp) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  temp = t.temperature;
}

void sendData() {
    
  updateBatteryLevel(v);
  updateIMU(ax, ay, az, gx, gy, gz, t);
  
  /*
  // Print the variables
  Serial.print("Battery Level: ");
  Serial.println(v);
  Serial.print("Acceleration X: ");
  Serial.print(ax);
  Serial.print(", Y: ");
  Serial.print(ay);
  Serial.print(", Z: ");
  Serial.println(az);
  Serial.print("Gyroscope X: ");
  Serial.print(gx);
  Serial.print(", Y: ");
  Serial.print(gy);
  Serial.print(", Z: ");
  Serial.println(gz);
  Serial.print("Temperature: ");
  Serial.println(t);
  */

  // Copy the data into the buffer
  memcpy(replyBuffer, &v, sizeof(float));
  memcpy(replyBuffer + sizeof(float), &t, sizeof(float));
  memcpy(replyBuffer + 2 * sizeof(float), &ax, sizeof(float));
  memcpy(replyBuffer + 3 * sizeof(float), &ay, sizeof(float));
  memcpy(replyBuffer + 4 * sizeof(float), &az, sizeof(float));
  memcpy(replyBuffer + 5 * sizeof(float), &gx, sizeof(float));
  memcpy(replyBuffer + 6 * sizeof(float), &gy, sizeof(float));
  memcpy(replyBuffer + 7 * sizeof(float), &gz, sizeof(float));

  /*
  replyBuffer[0] = v;
  replyBuffer[1] = t;
  replyBuffer[2] = ax;
  replyBuffer[3] = ax;
  replyBuffer[4] = ax;
  replyBuffer[5] = ax;
  replyBuffer[6] = ax;
  replyBuffer[7] = ax;
  */

  // Send the data to the client
  client.write(replyBuffer, sizeof(replyBuffer));
}