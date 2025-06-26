#include "state.h"

PocketbaseState pocketbaseState;
FlightDataState flightDataState;
LoadingDataState loadingDataState;
CommandQueue padCommandQueue;
CommandQueue gatewayCommandQueue;

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t requestQueueMutex;
SemaphoreHandle_t ntpClientMutex;        // Mutex for NTP client
SemaphoreHandle_t uploadPocketbaseMutex; // Mutex for Pocketbase uploads

WiFiUDP ntpUdp;
NTPClient ntpClient(ntpUdp, "pool.ntp.org", 0,
                    60000); // NTP client for time synchronization

PocketbaseArduino pocketbaseUploadConnection(DEATH_STAR_POCKETBASE_HOST);

horizonStates currentHorizonState = HZ_STAND_BY; // Initial state
unsigned long lastStateChangeTime = 0;           // Track last state change time
unsigned long lastProbeUpdateTime = 0;           // Track last probe update time

void onFlightDataUpdate() {
  lastProbeUpdateTime = millis(); // Update last probe update time
  if (currentHorizonState == HZ_FLYING) {
    if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
      // enqueueFlightData(flightDataState.currentFlightData);
      xSemaphoreGive(requestQueueMutex);
    } else {
      Serial.println("Failed to take request queue mutex for flight data.");
    }
    if (flightDataState.currentFlightData.isDeployed &&
        !flightDataState.previousFlightData.isDeployed) {
      LauncherLog log;

      if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
        log.timestamp = ntpClient.getEpochTime();
        xSemaphoreGive(ntpClientMutex);
      } else {
        Serial.println("Failed to take NTP client mutex for flight data log.");
      }
      log.level = LOG_INFO;
      log.event = "probe_deployed";
      log.message = "Probe deployed successfully.";
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        enqueueLog(log);
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for log.");
      }
    } else if (flightDataState.currentFlightData.isLanded &&
               !flightDataState.previousFlightData.isLanded) {
      currentHorizonState = HZ_LANDED; // Update state to LANDED
      lastStateChangeTime = millis();
      LaunchUpdate launchUpdate;
      launchUpdate.type = LANDED;
      if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
        launchUpdate.timestamp = ntpClient.getEpochTime();
        xSemaphoreGive(ntpClientMutex);
      } else {
        Serial.println("Failed to take NTP client mutex for launch update.");
      }
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        enqueueLaunchUpdate(launchUpdate);
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for launch update.");
      }
    }
  }
}

void onLoadingDataUpdate() {
  if (currentHorizonState == HZ_LOADING) {
    if (loadingDataState.launchState == FILLING_WATER ||
        loadingDataState.launchState == FILLED_WATER) {
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        // enqueueWaterLoadingData(loadingDataState.currentWaterLoadingData);
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for water data.");
      }
    } else if (loadingDataState.launchState == FILLING_AIR ||
               loadingDataState.launchState == READY_TO_LAUNCH) {
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        // enqueueAirLoadingData(loadingDataState.currentAirLoadingData);
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for air data.");
      }
    }
    if (loadingDataState.launchState == FILLING_WATER &&
        loadingDataState.previousLaunchState != FILLING_WATER) {
      LaunchUpdate launchUpdate;
      launchUpdate.type = STARTED_WATER_LOADING;

      if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
        launchUpdate.timestamp = ntpClient.getEpochTime();
        xSemaphoreGive(ntpClientMutex);
      } else {
        Serial.println("Failed to take NTP client mutex for launch update.");
      }
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        enqueueLaunchUpdate(launchUpdate);
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for launch update.");
      }
    } else if (loadingDataState.launchState == FILLED_WATER &&
               loadingDataState.previousLaunchState != FILLED_WATER) {
      float airPressure =
          pocketbaseState.launchRecord["record"]["pressure"].as<float>();
      padCommandQueue.enqueue("fill_air " + String(airPressure));
    }
  } else if (loadingDataState.launchState == FILLING_AIR &&
             loadingDataState.previousLaunchState != FILLING_AIR) {
    LaunchUpdate launchUpdate;
    launchUpdate.type = STARTED_AIR_LOADING;

    if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
      launchUpdate.timestamp = ntpClient.getEpochTime();
      xSemaphoreGive(ntpClientMutex);
    } else {
      Serial.println("Failed to take NTP client mutex for launch update.");
    }
    if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
      enqueueLaunchUpdate(launchUpdate);
      xSemaphoreGive(requestQueueMutex);
    } else {
      Serial.println("Failed to take request queue mutex for launch update.");
    }
  } else if (loadingDataState.launchState == READY_TO_LAUNCH &&
             loadingDataState.previousLaunchState != READY_TO_LAUNCH) {
    currentHorizonState = HZ_READY;
    lastStateChangeTime = millis();
    LaunchUpdate launchUpdate;
    launchUpdate.type = LOAD_COMPLETED;
    if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
      launchUpdate.timestamp = ntpClient.getEpochTime();
      xSemaphoreGive(ntpClientMutex);
    } else {
      Serial.println("Failed to take NTP client mutex for launch update.");
    }
    if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
      enqueueLaunchUpdate(launchUpdate);
      xSemaphoreGive(requestQueueMutex);
    } else {
      Serial.println("Failed to take request queue mutex for launch update.");
    }
  } else if (loadingDataState.launchState == LAUNCHING &&
             loadingDataState.previousLaunchState != LAUNCHING) {
    currentHorizonState = HZ_FLYING;
    lastStateChangeTime = millis();
    LaunchUpdate launchUpdate;
    launchUpdate.type = FIRED;
    if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
      launchUpdate.timestamp = ntpClient.getEpochTime();
      xSemaphoreGive(ntpClientMutex);
    } else {
      Serial.println("Failed to take NTP client mutex for launch update.");
    }
    if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
      enqueueLaunchUpdate(launchUpdate);
      xSemaphoreGive(requestQueueMutex);
    } else {
      Serial.println("Failed to take request queue mutex for launch update.");
    }
  } else if (loadingDataState.launchState == CANCELLED &&
             loadingDataState.previousLaunchState != CANCELLED) {
    currentHorizonState = HZ_CANCELLED; // Update state to CANCELLED
    LaunchUpdate launchUpdate;
    launchUpdate.type = CANCELLED_;

    if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
      launchUpdate.timestamp = ntpClient.getEpochTime();
      xSemaphoreGive(ntpClientMutex);
    } else {
      Serial.println("Failed to take NTP client mutex for launch update.");
    }
    if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
      enqueueLaunchUpdate(launchUpdate);
      xSemaphoreGive(requestQueueMutex);
    } else {
      Serial.println("Failed to take request queue mutex for launch update.");
    }
  }
}

void onLaunchUpdateCallback() {
  Serial.println("=== Launch control callback triggered ===");

  // Debug current states
  Serial.print("Current loading state: ");
  Serial.print(loadingDataState.launchState);
  Serial.print(" (");
  Serial.print(launchStateToString(loadingDataState.launchState));
  Serial.println(")");

  Serial.print("Current horizon state: ");
  Serial.print(currentHorizonState);
  Serial.print(" (");
  Serial.print(horizonStateToString(currentHorizonState));
  Serial.println(")");

  // Debug launch record contents
  Serial.println("Launch record contents:");
  if (!pocketbaseState.launchRecord.isNull()) {
    Serial.println(pocketbaseState.launchRecord.as<String>());
    Serial.print("should_load: ");
    Serial.println(
        pocketbaseState.launchRecord["record"]["should_load"].as<bool>());
    Serial.print("should_launch: ");
    Serial.println(
        pocketbaseState.launchRecord["record"]["should_launch"].as<bool>());
    Serial.print("should_cancel: ");
    Serial.println(
        pocketbaseState.launchRecord["record"]["should_cancel"].as<bool>());
  } else {
    Serial.println("Launch record is NULL!");
  }

  // First condition: STAND_BY + should_load
  if (loadingDataState.launchState == STAND_BY &&
      pocketbaseState.launchRecord["record"]["should_load"].as<bool>()) {
    Serial.println(">>> CONDITION 1 MET: Starting water loading process");

    currentHorizonState = HZ_LOADING; // Transition to loading state
    lastStateChangeTime = millis();
    Serial.print("Changed horizon state to HZ_LOADING, timestamp: ");
    Serial.println(lastStateChangeTime);

    float waterVolumicPercentage =
        pocketbaseState.launchRecord["record"]["water_volumic_percentage"]
            .as<float>();
    Serial.print("Water volumic percentage from record: ");
    Serial.println(waterVolumicPercentage);

    float rocketVolume;
    /*String serializedRocketRecord;
    if (xSemaphoreTake(uploadPocketbaseMutex, portMAX_DELAY) == pdTRUE) {
      serializedRocketRecord =
          pocketbaseUploadConnection.collection("rockets").getOne(
              pocketbaseState.launchRecord["record"]["rocket"]
                  .as<String>()
                  .c_str(),
              nullptr, "volume");
      xSemaphoreGive(uploadPocketbaseMutex);
    } else {
      Serial.println("Failed to take upload mutex for rocket record.");
      rocketVolume = 0.0;
    }

        DynamicJsonDocument rocketDoc(1024);
    DeserializationError error =
        deserializeJson(rocketDoc, serializedRocketRecord);
    if (!error) {
      pocketbaseState.rocketRecord = rocketDoc;
      rocketVolume = pocketbaseState.rocketRecord["volume"].as<float>();
      Serial.println(pocketbaseState.rocketRecord.as<String>());
      Serial.print("Rocket volume: ");
      Serial.println(rocketVolume);
    } else {
      Serial.println("Failed to deserialize rocket record JSON");
      rocketVolume = 0.0;
    }*/
    rocketVolume = 2.0;
    Serial.print("Using hardcoded rocket volume: ");
    Serial.println(rocketVolume);

    float calculatedWaterVolume = waterVolumicPercentage * rocketVolume;
    Serial.print("Calculated water volume: ");
    Serial.println(calculatedWaterVolume);

    String command = "fill_water " + String(calculatedWaterVolume, 2);
    Serial.print("Enqueueing pad command: ");
    Serial.println(command);
    padCommandQueue.enqueue(command);
    Serial.println("Water loading command sent successfully");

  } else if (loadingDataState.launchState == READY_TO_LAUNCH &&
             pocketbaseState.launchRecord["record"]["should_launch"]
                 .as<bool>()) {
    Serial.println(">>> CONDITION 2 MET: Starting launch sequence");

    String command = "launch";
    Serial.print("Enqueueing pad command: ");
    Serial.println(command);
    padCommandQueue.enqueue(command);

    Serial.println("Enqueueing gateway command: GO");
    gatewayCommandQueue.enqueue("GO");
    Serial.println("Launch commands sent successfully");
    currentHorizonState = HZ_FLYING; // Update state to FLYING

  } else if (loadingDataState.launchState != STAND_BY &&
             pocketbaseState.launchRecord["record"]["should_cancel"]
                 .as<bool>()) {
    Serial.println(">>> CONDITION 3 MET: Cancelling launch");

    String command = "cancel";
    Serial.print("Enqueueing pad command: ");
    Serial.println(command);
    padCommandQueue.enqueue(command);

    Serial.println("Enqueueing gateway command: RESET");
    gatewayCommandQueue.enqueue("RESET");
    Serial.println("Cancel commands sent successfully");
    currentHorizonState = HZ_CANCELLED; // Update state to CANCELLED

  } else {
    Serial.println(">>> NO CONDITIONS MET");
    Serial.println("Condition 1 check:");
    Serial.print("  - loadingDataState.launchState == STAND_BY: ");
    Serial.println(loadingDataState.launchState == STAND_BY ? "TRUE" : "FALSE");
    Serial.print("  - should_load: ");
    Serial.println(
        pocketbaseState.launchRecord["record"]["should_load"].as<bool>()
            ? "TRUE"
            : "FALSE");

    Serial.println("Condition 2 check:");
    Serial.print("  - loadingDataState.launchState == READY_TO_LAUNCH: ");
    Serial.println(loadingDataState.launchState == READY_TO_LAUNCH ? "TRUE"
                                                                   : "FALSE");
    Serial.print("  - should_launch: ");
    Serial.println(
        pocketbaseState.launchRecord["record"]["should_launch"].as<bool>()
            ? "TRUE"
            : "FALSE");

    Serial.println("Condition 3 check:");
    Serial.print("  - loadingDataState.launchState != STAND_BY: ");
    Serial.println(loadingDataState.launchState != STAND_BY ? "TRUE" : "FALSE");
    Serial.print("  - should_cancel: ");
    Serial.println(
        pocketbaseState.launchRecord["record"]["should_cancel"].as<bool>()
            ? "TRUE"
            : "FALSE");
  }

  Serial.println("=== Launch control callback finished ===");
}

void resetEnduranceState() {
  currentHorizonState = HZ_STAND_BY; // Reset to standby state
  lastStateChangeTime = millis();
  padCommandQueue.enqueue("reset");
  gatewayCommandQueue.enqueue("RESET");
  pocketbaseState.launchRecord.clear();
}

void stateTask(void *pvParameters) {
  while (true) {
    if ((currentHorizonState == HZ_LANDED ||
         currentHorizonState == HZ_CANCELLED) &&
        millis() - lastStateChangeTime > 5000) {
      // Reset state after landing or cancellation
      Serial.println(
          "Resetting endurance state after 5 seconds of inactivity.");
      LauncherLog log;
      if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
        log.timestamp = ntpClient.getEpochTime();
        xSemaphoreGive(ntpClientMutex);
      } else {
        Serial.println("Failed to take NTP client mutex for log.");
      }
      log.level = LOG_INFO;
      log.event = "endurance_reset";
      log.message = "Endurance state reset after landing or cancellation.";
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        enqueueLog(log);
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for log.");
      }
      resetEnduranceState();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void launcherDataTask(void *pvParameters) {
  static Timer launcherDataTimer(2000);
  while (true) {
    if (launcherDataTimer.expired()) {
      if (pocketbaseState.isConnected) {
        LauncherData launcherData;
        launcherData.isProbeConnected = millis() - lastProbeUpdateTime < 5000;
        launcherData.probeBatteryLevel =
            flightDataState.currentFlightData.batteryLevel;
        launcherData.lastPressure = flightDataState.currentFlightData.pressure;
        launcherData.lastWaterFlow =
            loadingDataState.currentWaterLoadingData.waterFlowRate;
        launcherData.lastAcc =
            pow(pow(flightDataState.currentFlightData.ax, 2) +
                    pow(flightDataState.currentFlightData.ay, 2) +
                    pow(flightDataState.currentFlightData.az, 2),
                0.5);
        launcherData.lastAltitude = flightDataState.currentFlightData.altitude;
        if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
          enqueueLauncherData(launcherData);
          xSemaphoreGive(requestQueueMutex);
        } else {
          Serial.println(
              "Failed to take request queue mutex for launcher data.");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
  }
}

// Helper function to convert launch states to readable strings
const char *launchStateToString(launchStates state) {
  switch (state) {
  case STAND_BY:
    return "STAND_BY";
  case FILLING_WATER:
    return "FILLING_WATER";
  case FILLED_WATER:
    return "FILLED_WATER";
  case FILLING_AIR:
    return "FILLING_AIR";
  case READY_TO_LAUNCH:
    return "READY_TO_LAUNCH";
  case LAUNCHING:
    return "LAUNCHING";
  case LAUNCHED:
    return "LAUNCHED";
  case CANCELLING:
    return "CANCELLING";
  case CANCELLED:
    return "CANCELLED";
  default:
    return "UNKNOWN";
  }
}

// Helper function to convert horizon states to readable strings
const char *horizonStateToString(horizonStates state) {
  switch (state) {
  case HZ_STAND_BY:
    return "HZ_STAND_BY";
  case HZ_LOADING:
    return "HZ_LOADING";
  case HZ_READY:
    return "HZ_READY";
  case HZ_FLYING:
    return "HZ_FLYING";
  case HZ_LANDED:
    return "HZ_LANDED";
  case HZ_CANCELLED:
    return "HZ_CANCELLED";
  default:
    return "UNKNOWN";
  }
}
