#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "Chandan";
const char* password = "Billionaire";
const char* server_ip = "192.168.45.162";
// Server configuration
const int server_port = 12345;

// flex pin instances
const int flexPin = A0;
// Define smoothing factor (alpha value)
const float alpha = 0.8; // Adjust as needed, between 0 and 1

// Variables to store the last sensor readings
float smoothed_ax = 0;
float smoothed_ay = 0;
float smoothed_az = 0;


// Sensor instances
MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // You can use any address here

// Function prototypes
void connectToWiFi();
void readSensorData(JsonObject& json);
void sendData(const StaticJsonDocument<200>& jsonDoc);

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.print("Setup");
  connectToWiFi();

  // Initialize I2C bus
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();

  // Initialize magnetometer
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);
  }
}
void loop() {
  StaticJsonDocument<200> jsonDoc;
  readSensorData(jsonDoc);
  sendData(jsonDoc);
}

void connectToWiFi() {
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
const int screenWidth = 1920;
const int screenHeight = 1080;

const int maxSensorValue = 16384;
void readSensorData(StaticJsonDocument<200>& json) {
  int flexValue = 0;
  flexValue = analogRead(flexPin);
  json["flex_sen"]=flexValue;
  // Read accelerometer and gyroscope data from MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Apply smoothing
  applySmoothing(ax, ay, az);

  int cursorX = map(ax + maxSensorValue / 2, 0, maxSensorValue, 0, screenWidth);
  int cursorY = map(ay + maxSensorValue / 2, 0, maxSensorValue, 0, screenHeight);
  
  sensors_event_t event;
  mag.getEvent(&event);

  
  json["accel_x"] = ax;
  json["accel_y"] = ay;
  json["velocity_x"]=cursorX;
  json["velocity_y"]=cursorY;
  json["accel_z"] = az;
  json["gyro_x"] = gx;
  json["gyro_y"] = gy;
  json["gyro_z"] = gz;
  json["mag_x"] = event.magnetic.x;
  json["mag_y"] = event.magnetic.y;
  json["mag_z"] = event.magnetic.z;
}
void applySmoothing(int16_t raw_ax, int16_t raw_ay, int16_t raw_az) {
  // Apply exponential moving average (EMA)
  smoothed_ax = alpha * raw_ax + (1 - alpha) * smoothed_ax;
  smoothed_ay = alpha * raw_ay + (1 - alpha) * smoothed_ay;
  smoothed_az = alpha * raw_az + (1 - alpha) * smoothed_az;
}

void sendData(const StaticJsonDocument<200>& jsonDoc) {
  WiFiClient client;

  if (!client.connect(server_ip, server_port)) {
    Serial.println("Connection failed.");
    return;
  }
 
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  Serial.println("Connected to server");
  client.println(jsonString);
  Serial.println("JSON data sent to server:");
  Serial.println(jsonString);
  delay(150);
}
