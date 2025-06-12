#include "pocketbase_utils.h"
#include "EnduranceConfig.h"
#include <Arduino.h>

void onLauncherUpdate(SubscriptionEvent &ev, void *ctx);

void initPocketbase(PocketbaseArduino &pocketbaseConnection,
                    PocketbaseState &pocketbaseState) {
  bool connected = false;
  int retryCount = 0;
  const int maxRetries = 10;
  Serial.println("Connecting to Pocketbase...");
  while (!connected && retryCount < maxRetries) {
    connected = pocketbaseConnection.login_passwd(
        DEATH_STAR_POCKETBASE_IDENTITY, DEATH_STAR_POCKETBASE_PASSWORD,
        "launchers");
    Serial.print("Pocketbase login attempt: ");
    Serial.println(connected);
    if (!connected) {
      Serial.println("Pocketbase login failed, retrying in 2s...");
      delay(2000);
      retryCount++;
    } else {
      Serial.println("Pocketbase login successful!");
      pocketbaseState.launcherRecord =
          pocketbaseConnection.getConnectionRecord();
      pocketbaseState.isConnected = true;
      Serial.println("Pocketbase launcher record: " +
                     pocketbaseState.launcherRecord.as<String>());
      pocketbaseConnection.subscribe(
          "launchers",
          pocketbaseState.launcherRecord["record"]["id"].as<String>().c_str(),
          onLauncherUpdate, &pocketbaseState);
    }
  }
  if (!connected) {
    Serial.println("ERROR: Could not connect to Pocketbase after retries.");
  }
}

void onLauncherUpdate(SubscriptionEvent &ev, void *ctx) {
  // Implementation for handling launcher updates
}

void pocketbaseLoop(PocketbaseState &pocketbaseState,
                    PocketbaseArduino &pocketbaseConnection) {
  if (pocketbaseState.isConnected) {
    pocketbaseConnection.update_subscription();
  }
}
