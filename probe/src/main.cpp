#include <Arduino.h>
#include <LSM6DS3.h>
#include <Sensors.h>

AccGyroSensor Sensor(0x6A); // Default I2C address for LSM6DS3 is 0x6A

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing LSM6DS3...");

  if (Sensor.initialize() != 0) {
    Serial.println("Failed to initialize LSM6DS3!");
  } else {
    Serial.println("LSM6DS3 initialized successfully.");
    Serial.print("I2C Address:");
    Serial.println(Sensor.getI2CAddress(), HEX);
    Serial.println("Reading sensor data...");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float ax, ay, az;
  float gx, gy, gz;
  Sensor.read(ax, ay, az, gx, gy, gz);
  Serial.print("Accel: (X, Y, Z) = ");
  Serial.print(ax, 2);
  Serial.print(", ");
  Serial.print(ay, 2);
  Serial.print(", ");
  Serial.print(az, 2);
  Serial.print(" | Gyro: (X, Y, Z) = ");
  Serial.print(gx, 2);
  Serial.print(", ");
  Serial.print(gy, 2);
  Serial.print(", ");
  Serial.print(gz, 2);
  Serial.println();
}
