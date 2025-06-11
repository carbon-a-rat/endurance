#include "wifi_utils.h"
#include "EnduranceConfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

void initWiFi(const std::function<void()> &disconnectedCallback,
              const std::function<void()> &connectedCallback,
              const std::function<void()> &gotIPCallback) {

  // Initialize WiFi with the provided callbacks, except for the
  // disconnectedCallback
  WiFi.onEvent([disconnectedCallback, connectedCallback, gotIPCallback](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == SYSTEM_EVENT_STA_CONNECTED) {
      Serial.println("WiFi connected!");
      connectedCallback();
    } else if (event == SYSTEM_EVENT_STA_GOT_IP) {
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      gotIPCallback();
    }
  });
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 30000;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > wifiTimeout) {
      Serial.println("\nWiFi connection timed out!");
      Serial.println("Restarting WIFI in 2 seconds...");
      delay(2000);
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      startAttemptTime = millis(); // Reset attempt time
    }
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up event handlers for WiFi events
  // This will handle disconnection events and attempt to reconnect
  WiFi.onEvent([disconnectedCallback, connectedCallback, gotIPCallback](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
      Serial.println("WiFi disconnected, attempting to reconnect...");
      disconnectedCallback();
      WiFi.reconnect();
    } else if (event == SYSTEM_EVENT_STA_CONNECTED) {
      Serial.println("WiFi connected!");
      connectedCallback();
    } else if (event == SYSTEM_EVENT_STA_GOT_IP) {
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      gotIPCallback();
    }
  });
}
