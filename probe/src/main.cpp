#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Sensors.h>
#include <Servo.h>
#include <espnow.h>

#define GATEWAY_MAC_ADDRESS "34:AB:95:1A:75:37"

Servo servo;
int servoPin = 14;

BarometerSensor sensor(0x77);
AccGyroSensor accGyroSensor(0x6A);

bool shouldSendData = false;
bool isInFlight = false;

void macAddressToByteArray(const char *macAddress, uint8_t *byteArray) {
  int i = 0;
  char *macAddressCopy = strdup(macAddress);
  char *token = strtok(macAddressCopy, ":");
  while (token != NULL && i < 6) {
    byteArray[i] = (uint8_t)strtol(token, NULL, 16);
    token = strtok(NULL, ":");
    i++;
  }
}

void initializeSensors() {
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

void printMacAdresses() {
  Serial.print("Probe MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.print("Gateway MAC Address:  ");
  Serial.println(GATEWAY_MAC_ADDRESS);
  Serial.println("Initializing sensor...");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println();
  printMacAdresses();

  initializeSensors();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("ESP-NOW initialized successfully.");
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  uint8_t gatewayAddress[6];
  const char *gatewayMacAddress = GATEWAY_MAC_ADDRESS;
  macAddressToByteArray(gatewayMacAddress, gatewayAddress);
  esp_now_add_peer(gatewayAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);

  servo.attach(servoPin);
}

void printData(float p, float t, float a, float ax, float ay, float az,
               float gx, float gy, float gz) {
  Serial.print("Pressure: ");
  Serial.print(p);
  Serial.print(" hPa, Temperature: ");
  Serial.print(t);
  Serial.print(" °C, Altitude: ");
  Serial.print(a);
  Serial.print(" m, Accelerometer: ");
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

void loop() {
  if (shouldSendData) {
    float p, t, a;
    sensor.read(p, t, a);
    float ax, ay, az, gx, gy, gz;
    accGyroSensor.read(ax, ay, az, gx, gy, gz);
    printData(p, t, a, ax, ay, az, gx, gy, gz);
  }
  servo.write(0);
  delay(1000);
  servo.write(90);
  delay(1000);
  servo.write(180);
  delay(1000);
}
