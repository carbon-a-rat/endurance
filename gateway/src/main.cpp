#include <Arduino.h>
#include <ESP8266WiFi.h>

#define PROBE_MAC_ADDRESS "4C:75:25:03:EA:DC"

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Probe MAC Address:  ");
  Serial.println(PROBE_MAC_ADDRESS);
  Serial.print("Gateway MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void loop() {}
