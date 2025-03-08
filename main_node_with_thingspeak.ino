#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Pin Definitions
#define CAN_CS_PIN    5    // CAN CS pin
#define SCREEN_WIDTH  128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64   // OLED display height, in pixels
#define OLED_RESET    -1   // Reset pin # (or -1 if sharing Arduino reset pin)

// WiFi credentials
const char* ssid = "OVT-0892A1";
const char* password = "12345678";

// ThingSpeak settings
const char* server = "api.thingspeak.com";
const char* apiKey = "EYRV7BM0U05V6VCS";

// Initialize objects
MCP_CAN CAN0(CAN_CS_PIN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables to store sensor data
float temperature = 0.0;
float humidity = 0.0;
float distance = 0.0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float latitude = 0.0;
float longitude = 0.0;

void setup() {
    Serial.begin(115200);

    // Initialize CAN Bus
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
        Serial.println("CAN Init Failed");
        while (1);
    }
    Serial.println("CAN Init OK");
    CAN0.setMode(MCP_NORMAL);

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        while (1);
    }
    display.display();
    delay(2000);  // Pause for 2 seconds
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" connected");
}

void loop() {
    unsigned long rxId;
    byte len = 0;
    byte rxBuf[8];

    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        if (rxId == 0x100) {
            int16_t temp = (rxBuf[0] << 8) | rxBuf[1];
            int16_t hum = (rxBuf[2] << 8) | rxBuf[3];
            int16_t dist = (rxBuf[4] << 8) | rxBuf[5];

            temperature = temp / 100.0;
            humidity = hum / 100.0;
            distance = dist / 100.0;

            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.print("°C, Humidity: ");
            Serial.print(humidity);
            Serial.print("%, Distance: ");
            Serial.println(distance);
        } else if (rxId == 0x101) {
            int16_t ax = (rxBuf[0] << 8) | rxBuf[1];
            int16_t ay = (rxBuf[2] << 8) | rxBuf[3];
            int16_t az = (rxBuf[4] << 8) | rxBuf[5];

            accelX = ax / 100.0;
            accelY = ay / 100.0;
            accelZ = az / 100.0;

            Serial.print("AccelX: ");
            Serial.print(accelX);
            Serial.print(" m/s^2, AccelY: ");
            Serial.print(accelY);
            Serial.print(" m/s^2, AccelZ: ");
            Serial.println(accelZ);
        } else if (rxId == 0x102) {
            int32_t lat = (rxBuf[0] << 24) | (rxBuf[1] << 16) | (rxBuf[2] << 8) | rxBuf[3];
            int32_t lon = (rxBuf[4] << 24) | (rxBuf[5] << 16) | (rxBuf[6] << 8) | rxBuf[7];

            latitude = lat / 100000.0;
            longitude = lon / 100000.0;

            Serial.print("Latitude: ");
            Serial.print(latitude, 6);
            Serial.print(", Longitude: ");
            Serial.println(longitude, 6);
        }
    }

    // Update OLED display
    display.clearDisplay();
    display.setCursor(0, 0);

    display.print("Temp: ");
    display.print(temperature);
    display.println(" C");
    display.print("Hum: ");
    display.print(humidity);
    display.println(" %");
    display.print("Dist: ");
    display.print(distance);
    display.println(" cm");
    display.print("AccelX: ");
    display.print(accelX);
    display.println(" m/s^2");
    display.print("AccelY: ");
    display.print(accelY);
    display.println(" m/s^2");
    display.print("AccelZ: ");
    display.print(accelZ);
    display.println(" m/s^2");
    display.print("Lat: ");
    display.print(latitude, 6);
    display.println();
    display.print("Long: ");
    display.print(longitude, 6);
    display.println();

    display.display();

    // Send data to ThingSpeak
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String url = String("http://") + server + "/update?api_key=" + apiKey + "&field1=" + temperature + "&field2=" + humidity + "&field3=" + distance + "&field4=" + accelX + "&field5=" + accelY + "&field6=" + accelZ + "&field7=" + latitude + "&field8=" + longitude;
        http.begin(url);
        int httpCode = http.GET();
        if (httpCode > 0) {
            Serial.printf("ThingSpeak update code: %d\n", httpCode);
        } else {
            Serial.printf("ThingSpeak update failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
    }

    delay(20000);  // Update every 20 seconds
}