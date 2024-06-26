#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// WiFi credentials
const char* ssid = "ERR_EMPTY_RESPONSE";
const char* password = "[Pr0tocol_3rror]={}";

// Server settings
const int port = 12345;
WiFiServer server(port);
WiFiClient client;

// Buffer to hold the incoming data from the server.
// The buffer has size 64x128 = 8192 bits = 1024 bytes 
// and matches the size of the oled display.
// We will have a bit for each pixel that can either be 0
// if the pixel is off and 1 if the pixel is on.
const int displayBufferSize = 1024;
uint8_t displayBuffer[displayBufferSize];  // Buffer to hold incoming data
int bytesRead = 0;  // How many bytes has been read at a given time

// Buffer to hold the serialized data to send back to the server.
// The data includes voltage, temperature, ax, ay, az, gx, gy, gz
// as 8 floats, 4 bytes each, 32 bytes total
const int replyBufferSize = 8 * sizeof(float);
uint8_t replyBuffer[replyBufferSize];

// OLED display settings
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
const uint8_t displayWidth = 128;
const uint8_t displayHeight = 64;

// IMU settings and associated variables
Adafruit_MPU6050 mpu;
bool imuInitialized = false; // Flag to check if IMU is initialized

// Temperature from the IMU sensor
float t = 0.0;

// Acceleration values from the IMU sensor (X, Y, Z axes)
float ax = 0.0; // Acceleration along the X-axis
float ay = 0.0; // Acceleration along the Y-axis
float az = 0.0; // Acceleration along the Z-axis

// Gyroscope values from the IMU sensor (X, Y, Z axes)
float gx = 0.0; // Angular velocity around the X-axis
float gy = 0.0; // Angular velocity around the Y-axis
float gz = 0.0; // Angular velocity around the Z-axis

// Battery voltage
float v;


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
  /*
   * Connect to WiFi, initialize the server and log IP address
   * and the port where we are listening.
   */

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
  /*
   * The Wemos battery shield v2 has a built int resistor
   * connected to the A0 pin we can use to monitor the
   * battery status. Set A0 as INPUT in order to read
   * the value and log the battery level.
   */

  pinMode(A0, INPUT);
  Serial.println("[INFO] Battery shield initialized");

  updateBatteryLevel();
  Serial.print("[INFO] Battery level: ");
  Serial.println(v);
}

void setupOLED() {
  /* 
   * Setup the OLED display.
   */
  
  u8g2.begin();
  Serial.println("[INFO] OLED display initialized");
}

void setupIMU() {
  /*
   * Setup the IMU and log the ranges if the IMU is found.
   */

  if (!mpu.begin()) {
    Serial.println("[ERROR] Unable to find IMU");
  } else {
    Serial.println("[INFO] IMU Initialized");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.println("[INFO] Accelerometer range set to +- 8G");

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.println("[INFO] Gyro range set to +- 500 deg/s");

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.println("[INFO] Filter bandwidth set to 5 Hz");

    imuInitialized = true;
  }
}

void loop() {

  // Check if a client has connected 
  client = server.available();
  if (client) {

    Serial.println("[INFO] Client connected");
    bytesRead = 0;

    // Read data from the client
    while (bytesRead < displayBufferSize) {
      if (client.available()) {
        bytesRead += client.read(displayBuffer + bytesRead, displayBufferSize - bytesRead);
      } else {
        delay(1); // Small delay to yield to other processes
      }
    }

    if (bytesRead == displayBufferSize) {

      // Display data on the OLED
      u8g2.clearBuffer();
      u8g2.drawXBMP(0, 0, displayWidth, displayHeight, displayBuffer);
      u8g2.sendBuffer();
    }

    // Reply by writing the status back to the server
    sendData();

    if (termination()) {
      client.stop();
      Serial.println("[INFO] Client disconnected");
    }
  }
}

bool termination() {
  /*
   * As a simple termination condition we check if the received
   * array only contains 1s
   */

  for (size_t i = 0; i < replyBufferSize; i++) {
    if (replyBuffer[i] != 1) {
        return false;
    }
  }
  return true;
}

void updateBatteryLevel() {
  /*
   * Update the battery level status.
   */

  v = analogRead(A0) / 1024.0 * 100;
}

void updateIMU() {
  /*
   * Update the IMU data if the IMU has been found.
   */

  if (imuInitialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
    t = temp.temperature;
  }
}

void sendData() {
    
  updateBatteryLevel();
  updateIMU();
  
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