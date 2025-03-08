#include <SPI.h>
#include <mcp_can.h>
#include <DHT.h>

// Pin Definitions
#define CAN_CS_PIN    5    // CAN CS pin
#define DHT_PIN       4    // DHT11 data pin
#define TRIG_PIN      13   // HC-SR04 trigger pin
#define ECHO_PIN      12   // HC-SR04 echo pin
#define DHT_TYPE      DHT11 // DHT11 sensor type

// Initialize objects
MCP_CAN CAN0(CAN_CS_PIN);
DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
    Serial.begin(115200);

    // Initialize CAN Bus
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
        Serial.println("CAN Init Failed");
        while (1);
    }
    Serial.println("CAN Init OK");
    CAN0.setMode(MCP_NORMAL);

    // Initialize DHT11
    dht.begin();

    // Initialize HC-SR04 pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

float readDistance() {
    // Clear trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10μs pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo pulse duration
    long duration = pulseIn(ECHO_PIN, HIGH, 50000); // Timeout after 50ms

    // Calculate distance in cm
    float distance = duration * 0.034 / 2;
    if (distance >= 2 && distance <= 400) {
        return distance;
    } else {
        return -1.0; // Invalid reading
    }
}

void sendData(float temperature, float humidity, float distance) {
    byte data[6];

    // Pack data into CAN message
    int16_t temp = (int16_t)(temperature * 100);
    int16_t hum = (int16_t)(humidity * 100);
    int16_t dist = (int16_t)(distance * 100);

    data[0] = temp >> 8;
    data[1] = temp & 0xFF;
    data[2] = hum >> 8;
    data[3] = hum & 0xFF;
    data[4] = dist >> 8;
    data[5] = dist & 0xFF;

    // Send the message
    if (CAN0.sendMsgBuf(0x100, 0, 6, data) == CAN_OK) {
        Serial.println("DHT11 and HC-SR04 Data Sent Successfully");
    } else {
        Serial.println("Error Sending Data");
    }
}

void loop() {
    // Read sensor data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    float distance = readDistance();

    // Check for valid readings
    // if (!isnan(temperature) && !isnan(humidity) && distance != -1.0) {
        // Print sensor data to Serial
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print("°C, Humidity: ");
        Serial.print(humidity);
        Serial.print("%, Distance: ");
        Serial.println(distance);

        // Send sensor data via CAN
        sendData(temperature, humidity, distance);
    // } else {
    //     Serial.println("Invalid sensor readings detected. Skipping transmission.");
    // }

    delay(2000);  // Update every 2 seconds
}