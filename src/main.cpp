#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <WiFi.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// MCP2515 pins
#define CAN_CS 5
#define CAN_INT 4
MCP_CAN CAN(CAN_CS); // Initialize MCP2515

// GPS Pins
#define TX_PIN 17
#define RX_PIN 18
#define GPS_BAUD 115200
#define SerialAT Serial1
TinyGsm modem(SerialAT);

// Motor Pins
#define BUTTON_PIN 12
#define PWM_PIN 25
#define LED_PIN 13

// Server Information
const char server[] = "45.88.223.145"; // Your server IP
const int port = 3000;                // Server port
const char resource[] = "/api/data";  // API endpoint

WiFiClient wifiClient;
TinyGsmClient gsmClient(modem);
HttpClient http(wifiClient, server, port);
HttpClient httpGSM(gsmClient, server, port);

// WiFi Credentials
const char* ssid = "WiFi_SSID";
const char* password = "WiFi_Password";

// GSM Configuration
#define GSM_APN "apn"
#define GSM_USER "user"
#define GSM_PASS "password"

bool useGSM = false;

// Global Variables
float pumpSpeed = 0.0;
bool pumpState = false;
unsigned long rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
float lat = 0, lon = 0, speed = 0, alt = 0;
float accuracy = 0;
int vehicleSpeed = 0;
float fuelLevel = 0.0;

// GPS Setup
void setupGPS() {
    SerialAT.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    modem.restart();
    modem.enableGPS();
    delay(15000); // Allow GPS to acquire signal
}

// Handle Pump Control
void handlePumpControl() {
    if (digitalRead(BUTTON_PIN)) {
        pumpState = true;
        digitalWrite(LED_PIN, HIGH);
        pumpSpeed = 75.0;
        ledcWrite(0, map(pumpSpeed, 0, 100, 0, 255));
    } else {
        pumpState = false;
        digitalWrite(LED_PIN, LOW);
        pumpSpeed = 0.0;
        ledcWrite(0, 0);
    }
}

// Read GPS Data
void getGPSLocation() {
    float gpsLat = 0, gpsLon = 0, gpsSpeed = 0, gpsAlt = 0;
    int vsat = 0, usat = 0;

    if (modem.getGPS(&gpsLat, &gpsLon, &gpsSpeed, &gpsAlt, &vsat, &usat, &accuracy)) {
        lat = gpsLat;
        lon = gpsLon;
        speed = gpsSpeed * 3.6; // Convert m/s to km/h
        alt = gpsAlt;
    }
}

// Read CAN Bus Data
void readCANBusData() {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
        CAN.readMsgBuf(&rxId, &len, rxBuf);

        if (rxId == 0x123) { // CAN ID for vehicle speed
            vehicleSpeed = rxBuf[0]; // Example: Vehicle speed
        }
        if (rxId == 0x456) { // CAN ID for fuel level
            fuelLevel = rxBuf[0]; // Example: Fuel level (0-100)
        }
    }
}

// Send Data to Server
void sendDataToServer() {
    String payload = "{";
    payload += "\"device_id\": \"cm655r1jc0003zpf969r069ru\",";
    payload += "\"lat\": " + String(lat, 8) + ",";
    payload += "\"lon\": " + String(lon, 8) + ",";
    payload += "\"speed\": " + String(speed) + ",";
    payload += "\"alt\": " + String(alt) + ",";
    payload += "\"accuracy\": " + String(accuracy) + ",";
    payload += "\"pump_speed\": " + String(pumpSpeed) + ",";
    payload += "\"fuel\": " + String(fuelLevel) + ",";
    payload += "\"vehicle_speed\": " + String(vehicleSpeed);
    payload += "}";

    if (!useGSM) {
        http.beginRequest();
        http.post(resource);
        http.sendHeader("Content-Type", "application/json");
        http.sendHeader("Content-Length", payload.length());
        http.beginBody();
        http.print(payload);
        http.endRequest();
    } else {
        httpGSM.beginRequest();
        httpGSM.post(resource);
        httpGSM.sendHeader("Content-Type", "application/json");
        httpGSM.sendHeader("Content-Length", payload.length());
        httpGSM.beginBody();
        httpGSM.print(payload);
        httpGSM.endRequest();
    }

    int statusCode = useGSM ? httpGSM.responseStatusCode() : http.responseStatusCode();
    String response = useGSM ? httpGSM.responseBody() : http.responseBody();
    Serial.println("Status Code: " + String(statusCode));
    Serial.println("Response: " + response);
}

// Setup WiFi or GSM
void setupConnection() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
        Serial.print(".");
        delay(1000);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        useGSM = false;
    } else {
        Serial.println("\nWiFi failed. Connecting to GSM...");
        modem.gprsConnect(GSM_APN, GSM_USER, GSM_PASS);
        if (modem.isGprsConnected()) {
            Serial.println("GSM connected");
            useGSM = true;
        } else {
            Serial.println("GSM connection failed");
            while (true); // Halt if no connection
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // CAN Bus Setup
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("MCP2515 initialized successfully");
    } else {
        Serial.println("Error initializing MCP2515");
        while (1);
    }
    CAN.setMode(MCP_NORMAL);

    // Motor Pin Setup
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    ledcSetup(0, 5000, 8);
    ledcAttachPin(PWM_PIN, 0);

    // GPS Setup
    setupGPS();

    // Setup Connection
    setupConnection();
}

void loop() {
    // Handle Pump Control
    handlePumpControl();

    // Get GPS Data
    getGPSLocation();

    // Read CAN Bus Data
    readCANBusData();

    // Send Data to Server
    sendDataToServer();

    delay(5000); // Adjust delay as needed
}
