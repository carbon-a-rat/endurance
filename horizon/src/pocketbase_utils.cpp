#include "pocketbase_utils.h"
#include "EnduranceConfig.h"
#include "state.h"
#include <Arduino.h>

void onLaunchUpdate(SubscriptionEvent &ev, void *ctx);

void initPocketbase() {
  bool connected = false;
  int retryCount = 0;
  const int maxRetries = 10;
  Serial.println("Connecting to Pocketbase...");
  while (!connected && retryCount < maxRetries) {
    connected = pocketbaseUploadConnection.login_passwd(
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
      pocketbaseState.launcherRecord.clear();
      pocketbaseState.launcherRecord.set(
          pocketbaseUploadConnection.getConnectionRecord());
      pocketbaseState.isConnected = true;
      Serial.println("Pocketbase launcher record: " +
                     pocketbaseState.launcherRecord.as<String>());
      pocketbaseUploadConnection.subscribe("launches", "*", onLaunchUpdate,
                                           &pocketbaseState);
    }
  }
  if (!connected) {
    Serial.println("ERROR: Could not connect to Pocketbase after retries.");
  }
}

void onLaunchUpdate(SubscriptionEvent &ev, void *ctx) {
  PocketbaseState &pocketbaseState = *static_cast<PocketbaseState *>(ctx);
  // Handle the new launch event
  DynamicJsonDocument launchUpdate(1024); // Adjust size as needed
  DeserializationError error = deserializeJson(launchUpdate, ev.data);
  if (error) {
    Serial.print("Failed to parse launch update: ");
    Serial.println(error.c_str());
    return;
  }
  if (pocketbaseState.launchRecord.isNull()) {
    if (launchUpdate.containsKey("action") &&
        launchUpdate["action"].as<String>() == "create") {
      // Store the new launch record in the state
      if (launchUpdate["record"]["launcher"] ==
          pocketbaseState.launcherRecord["record"]["id"]) {
        pocketbaseState.launchRecord = launchUpdate;
        // Call the launch control callback if set
        if (pocketbaseState.launchControlCallback) {
          pocketbaseState.launchControlCallback();
        }
      }
    }
  } else {
    if (launchUpdate.containsKey("action") &&
        launchUpdate["action"].as<String>() == "update") {
      if (launchUpdate["record"]["id"] ==
          pocketbaseState.launchRecord["record"]["id"]) {
        // Update the launch record in the state
        pocketbaseState.launchRecord = launchUpdate;
        // Call the launch control callback if set
        if (pocketbaseState.launchControlCallback) {
          pocketbaseState.launchControlCallback();
        }
      }
    }
  }
}

String epochToPocketbaseTime(unsigned long epochTime) {
  // Convert NTP epoch time to Pocketbase format using strftime
  static String pocketbaseTime;
  time_t rawtime = epochTime;
  struct tm *timeinfo = localtime(&rawtime);
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  pocketbaseTime = String(buffer);
  return pocketbaseTime;
}
