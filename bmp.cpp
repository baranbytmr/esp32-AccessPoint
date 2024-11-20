#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Create BMP280 instance
Adafruit_BMP280 bmp; 

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Initializing BMP280...");

    // Initialize BMP280
    if (!bmp.begin(0x76)) { // Default I2C address is 0x76; use 0x77 if CSB is connected to VCC
        Serial.println("BMP280 not detected. Check wiring or I2C address.");
        while (1); // Stop program if sensor is not found
    }

    // Configure BMP280
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Normal mode
                    Adafruit_BMP280::SAMPLING_X2,      // Temperature oversampling x2
                    Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling x16
                    Adafruit_BMP280::FILTER_X16,       // Filter coefficient x16
                    Adafruit_BMP280::STANDBY_MS_500);  // Standby time 500ms

    Serial.println("BMP280 initialized successfully.");
}

void loop() {
    // Read temperature and pressure
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure(); // In Pascals
    float altitude = bmp.readAltitude(1013.25); // Approx. altitude (hPa at sea level)

    // Print the readings
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(pressure / 100); // Convert to hPa
    Serial.println(" hPa");

    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" meters");

    delay(1000); // Delay to avoid flooding the serial monitor
}
