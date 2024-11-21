#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bmp; 
const int mpu_adres = 0x68; // I2C
int16_t accX, accY, accZ;  // accelerometer 
int16_t gyroX, gyroY, gyroZ; // gyroscope 
int16_t tempRaw;          
float xlConv(int16_t rawVal){
  const float scale = 2/32768;
  return rawVal*scale*9.8;
}


void setup() {
Serial.begin(115200);
while (!Serial); 


 // temperature 

Wire.begin(21, 22);

Serial.println("starting BNO055");
  if (!bno.begin()) {
      Serial.println("BNO055 not detected.");
      while (1); 
  }


Serial.println("Initializing BMP280...");
    if (!bmp.begin(0x76)) { 
        Serial.println("BMP280 not detected.");
        while (1);
  }
bmp.setSampling(
  Adafruit_BMP280::MODE_NORMAL,      // Normal mode
  Adafruit_BMP280::SAMPLING_X2,      // Temperature oversampling x2
  Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling x16
  Adafruit_BMP280::FILTER_X16,       // Filter coefficient x16
  Adafruit_BMP280::STANDBY_MS_500);  // Standby time 500ms
Wire.beginTransmission(mpu_adres);
Wire.setClock(400000);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
}
void loop() {
  Wire.endTransmission(false);
  Wire.beginTransmission(mpu_adres);

  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_adres,2,true);
  accX=Wire.read()<<8|Wire.read();
  Serial.println("acc from mpu ");
  Serial.println(xlConv(accX));
  Serial.print(",");

  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));
    delay(500); 
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure(); // In Pascals
    float altitude = bmp.readAltitude(1013.25); // Approx. altitude (hPa at sea level)

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure / 100); // Convert to hPa
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");
  Serial.println(" ----------------------------------- ");

  delay(1000);

}
