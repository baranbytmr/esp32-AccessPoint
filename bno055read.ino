#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Replace 0x28 with 0x29 if needed

void setup() {
    Serial.begin(115200);
    while (!Serial); 

   
    Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22

    Serial.println("Initializing BNO055...");
    if (!bno.begin()) {
        Serial.println("BNO055 not detected. Check wiring and I2C address.");
        while (1); 
    }

    // Use external crystal oscillator for better accuracy
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized successfully.");
}

void loop() {
    sensors_event_t event;
    bno.getEvent(&event);

    // Print orientation data
    Serial.print("Orientation: ");
    Serial.print("X: "); Serial.print(event.orientation.x);
    Serial.print(", Y: "); Serial.print(event.orientation.y);
    Serial.print(", Z: "); Serial.println(event.orientation.z);

    delay(500); // Delay to avoid flooding the serial monitor
}
