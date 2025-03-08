#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

// Pin Definitions
#define CAN_CS_PIN    5    // CAN CS pin
#define GPS_RX_PIN    16   // GPS RX pin (connected to TX of GPS)
#define GPS_TX_PIN    17   // GPS TX pin (connected to RX of GPS)

// Initialize objects
MCP_CAN CAN0(CAN_CS_PIN);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Use hardware serial 2 for GPS

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // Initialize CAN Bus
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
        Serial.println("CAN Init Failed");
        while (1);
    }
    Serial.println("CAN Init OK");
    CAN0.setMode(MCP_NORMAL);

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1);
    }
    Serial.println("MPU6050 Found");
}

void readAccelerometer(float &accelX, float &accelY, float &accelZ) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;
}

void sendData(float accelX, float accelY, float accelZ, float latitude, float longitude) {
    byte data[8];

    // Pack accelerometer data into CAN message
    int16_t ax = (int16_t)(accelX * 100);
    int16_t ay = (int16_t)(accelY * 100);
    int16_t az = (int16_t)(accelZ * 100);
    int32_t lat = (int32_t)(latitude * 100000);
    int32_t lon = (int32_t)(longitude * 100000);

    data[0] = ax >> 8;
    data[1] = ax & 0xFF;
    data[2] = ay >> 8;
    data[3] = ay & 0xFF;
    data[4] = az >> 8;
    data[5] = az & 0xFF;

    // Send the accelerometer data
    if (CAN0.sendMsgBuf(0x101, 0, 6, data) == CAN_OK) {
        Serial.println("Accelerometer Data Sent Successfully");
    } else {
        Serial.println("Error Sending Accelerometer Data");
    }

    byte gpsData[8];
    gpsData[0] = lat >> 24;
    gpsData[1] = lat >> 16;
    gpsData[2] = lat >> 8;
    gpsData[3] = lat & 0xFF;
    gpsData[4] = lon >> 24;
    gpsData[5] = lon >> 16;
    gpsData[6] = lon >> 8;
    gpsData[7] = lon & 0xFF;

    // Send the GPS data
    if (CAN0.sendMsgBuf(0x102, 0, 8, gpsData) == CAN_OK) {
        Serial.println("GPS Data Sent Successfully");
    } else {
        Serial.println("Error Sending GPS Data");
    }
}

void loop() {
    // Read accelerometer data
    float accelX, accelY, accelZ;
    readAccelerometer(accelX, accelY, accelZ);

    // Read GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    float latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
    float longitude = gps.location.isValid() ? gps.location.lng() : 0.0;

    // Print accelerometer and GPS data to Serial
    Serial.print("AccelX: ");
    Serial.print(accelX);
    Serial.print(" m/s^2, AccelY: ");
    Serial.print(accelY);
    Serial.print(" m/s^2, AccelZ: ");
    Serial.print(accelZ);
    Serial.print(" m/s^2, Latitude: ");
    Serial.print(latitude, 6);
    Serial.print(", Longitude: ");
    Serial.println(longitude, 6);

    // Send accelerometer and GPS data via CAN
    sendData(accelX, accelY, accelZ, latitude, longitude);

    delay(2000);  // Update every 2 seconds
}