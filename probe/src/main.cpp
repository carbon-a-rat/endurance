#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Sensors.h>

#define GATEWAY_MAC_ADDRESS "34:AB:95:1A:75:37"

BarometerSensor sensor(0x77);
AccGyroSensor accGyroSensor(0x6A);

bool shouldSendData = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Probe MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.print("Gateway MAC Address:  ");
  Serial.println(GATEWAY_MAC_ADDRESS);
  Serial.println("Initializing sensor...");

  if (sensor.initialize() == false) {
    Serial.println("Failed to initialize sensor!");
  } else {
    Serial.println("Barometer sensor initialized successfully.");
    Serial.print("I2C Address:");
    Serial.println(sensor.getI2CAddress(), HEX);
    Serial.println("Reading sensor data...");
  }

  if (accGyroSensor.initialize() == false) {
    Serial.println("Failed to initialize accelerometer/gyroscope sensor!");
  } else {
    Serial.println("Accelerometer/Gyroscope sensor initialized successfully.");
    Serial.print("I2C Address:");
    Serial.println(accGyroSensor.getI2CAddress(), HEX);
    Serial.println("Reading sensor data...");
  }
}

void loop() {
  float p, t, a;
  sensor.read(p, t, a);
  Serial.print("Pressure: ");
  Serial.print(p);
  Serial.print(" hPa, Temperature: ");
  Serial.print(t);
  Serial.print(" °C, Altitude: ");
  Serial.print(a);
  Serial.print(" m,");
  float ax, ay, az, gx, gy, gz;
  accGyroSensor.read(ax, ay, az, gx, gy, gz);
  Serial.print("Accelerometer: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print(" Gyroscope: ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gz);
  Serial.println(" dps");
}
