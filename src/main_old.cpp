#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#define TINY_GSM_MODEM_SIM7600  // Defines the modem model, goes before declaring the library
#include <TinyGsmClient.h>

//MCP2515 pins
#define CAN_CS 5       // MCP2515 CS Pin
#define CAN_INT 4      // MCP2515 INT Pin
MCP_CAN CAN(CAN_CS);   // Initialize MCP2515

//GPS Pins
#define TX_PIN 17
#define RX_PIN 18
#define GPS_BAUD 115200
#define SerialAT Serial1
TinyGsm modem(SerialAT);

// Motor Pins
#define BUTTON_PIN 12  // Button to turn the pump on/off
#define PWM_PIN 25     // PWM output for the pump
#define LED_PIN 13     // LED pump status indicator

// Global Variables
float pumpSpeed = 0.0;  // Pump speed (0-100%)
bool pumpState = false; // Pump status (ON/OFF)
unsigned long rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// GPS Settings
void setupGPS() {

    SerialAT.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // Communication with SIM7600

    Serial.println("Initializing module...");

    SerialAT.println("AT+CFUN=1,1"); //Restart modem 
    delay(5000);

    // Display modem information
    String modemInfo = modem.getModemInfo();
    Serial.println("Modem Information: " + modemInfo);

    // Turn on GPS
    modem.sendAT("+SGPIO=0,4,1,1"); // Turn on GPS power
    if (modem.waitResponse(10000L) != 1) {
        Serial.println("Error turning on GPS");
    }

    modem.enableGPS(); // Activate GPS
    delay(15000);      // Wait for the GPS to get a signal
}

//  Pump control
void handlePumpControl() {
    if (digitalRead(BUTTON_PIN)) {
        pumpState = true;
        digitalWrite(LED_PIN, HIGH);
        pumpSpeed = 75.0;  // Engine speed at 75%
        ledcWrite(0, map(pumpSpeed, 0, 100, 0, 255)); // PWM based on speed
        Serial.println("Bomba encendida");
    } else {
        pumpState = false;
        digitalWrite(LED_PIN, LOW);
        pumpSpeed = 0.0;
        ledcWrite(0, 0); // Turn off engine
        Serial.println("Bomba apagada");
    }
}

// Read GPS data
void getGPSLocation() {
    float lat = 0, lon = 0, speed = 0, alt = 0;
    int vsat = 0, usat = 0;
    float accuracy = 0;
    int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

    Serial.println("Getting GPS location...");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                     &year, &month, &day, &hour, &min, &sec)) {
        Serial.println("---- GPS data ----");
        Serial.println("Latitude: " + String(lat, 8) + "\tLength: " + String(lon, 8));
        Serial.println("Speed: " + String(speed) + " m/s\tAltitude: " + String(alt) + " m");
        Serial.println("Visible satellites: " + String(vsat) + "\tUsed satellites: " + String(usat));
        Serial.println("Precision: " + String(accuracy) + " m");
        Serial.println("Date: " + String(year) + "-" + String(month) + "-" + String(day));
        Serial.println("Hour: " + String(hour) + ":" + String(min) + ":" + String(sec));
        Serial.println("-------------------");
    } else {
        Serial.println("Could not get GPS location. Retrying...");
    }
}

// Reading CAN Bus data
void readCANBusData() {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
        CAN.readMsgBuf(&rxId, &len, rxBuf);

        Serial.print("CAN ID: 0x");
        Serial.println(rxId, HEX);

        Serial.print("Datos: ");
        for (int i = 0; i < len; i++) {
            Serial.print(rxBuf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Iniciando sistema...");

    // CAN Bus Configuration
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("MCP2515 inicializado correctamente");
    } else {
        Serial.println("Error al inicializar MCP2515");
        while (1);
    }
    CAN.setMode(MCP_NORMAL);

    // Motor pinout configuration
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    ledcSetup(0, 5000, 8); // Set channel 0, frequency 5 kHz, resolution 8 bits
    ledcAttachPin(PWM_PIN, 0); // Associate channel 0 to the PWM_PIN pin


    // GPS Configuration
    setupGPS();
}

void loop() {
    // 1. Pump control
    handlePumpControl();

    // 2. Read GPS data
    getGPSLocation();

    // 3. Reading CAN Bus data
    readCANBusData();

    delay(100);
}
