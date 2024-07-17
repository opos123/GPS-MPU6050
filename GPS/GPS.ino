#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ArduinoJson.h>

// GPS settings
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
const int timezoneOffset = 7;

// MPU6050 settings
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
struct MyData {
  float Roll;
  float Pitch;
  float Yaw;
};
MyData data;
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
const float alpha = 0.1;
unsigned long previousTime = 0;
float gz_offset = 0;
const float gyroscopeThreshold = 2; // Sesuaikan threshold gyroscopic
float initialRoll = 0, initialPitch = 0;
const float accelerometerThreshold = 2; // Sesuaikan threshold accelerometer

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 terhubung");
  } else {
    Serial.println("MPU6050 tidak terhubung");
    while (1);
  }

  Serial.println("Kalibrasi giroskop, harap jangan gerakkan sensor...");
  delay(1000);

  int numSamples = 100;
  long gz_sum = 0;

  for (int i = 0; i < numSamples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gz_sum += gz;
    delay(10);
  }

  gz_offset = gz_sum / numSamples / 131.0;

  Serial.println("Kalibrasi selesai");

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  filtered_ax = ax;
  filtered_ay = ay;
  filtered_az = az;

  initialRoll = atan2(filtered_ay, filtered_az) * 180 / PI;
  initialPitch = atan2(-filtered_ax, sqrt(filtered_ay * filtered_ay + filtered_az * filtered_az)) * 180 / PI;
}

void loop() {
  // Handle GPS data
  while (ss.available() > 0) {
    char c = ss.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        // Read GPS data
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();

        // Read MPU6050 data
        bacaMPU6050();

        // Prepare JSON data
        StaticJsonDocument<100> doc;
        doc["latitude"] = latitude;
        doc["longitude"] = longitude;
        doc["roll"] = data.Roll;
        doc["pitch"] = data.Pitch;
        doc["yaw"] = data.Yaw;

        // Serialize JSON to string and print
        String jsonString;
        serializeJson(doc, jsonString);
        Serial.println(jsonString);
      }
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("Tidak ada GPS terdeteksi: periksa kabel.");
    while (true);
  }
}

void bacaMPU6050() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Filter accelerometer data
  filtered_ax = alpha * ax + (1 - alpha) * filtered_ax;
  filtered_ay = alpha * ay + (1 - alpha) * filtered_ay;
  filtered_az = alpha * az + (1 - alpha) * filtered_az;

  // Calculate Roll and Pitch angles
  float currentRoll = atan2(filtered_ay, filtered_az) * 180 / PI;
  float currentPitch = atan2(-filtered_ax, sqrt(filtered_ay * filtered_ay + filtered_az * filtered_az)) * 180 / PI;

  // Check if changes in Roll or Pitch are significant
  if (abs(currentRoll - initialRoll) < accelerometerThreshold) {
    data.Roll = 0; // Return to zero if in the initial position
  } else {
    data.Roll = currentRoll;
  }

  if (abs(currentPitch - initialPitch) < accelerometerThreshold) {
    data.Pitch = 0; // Return to zero if in the initial position
  } else {
    data.Pitch = currentPitch;
  }

  // Calculate deltaTime for Yaw integration
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Calibrate Gyroscope
  float gz_calibrated = gz / 131.0 - gz_offset;

  // Apply threshold to ignore small changes
  if (abs(gz_calibrated) > gyroscopeThreshold) {
    // Integrate angular velocity to get Yaw
    data.Yaw += gz_calibrated * deltaTime;

    // Normalize Yaw angle to [-180, 180]
    while (data.Yaw > 180) data.Yaw -= 360;
    while (data.Yaw < -180) data.Yaw += 360;
  }
}
