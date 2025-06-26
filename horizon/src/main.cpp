#include "i2c_stats.h"
#include "i2c_utils.h"
#include "ntp_utils.h"
#include "pocketbase_utils.h"
#include "state.h"
#include "wifi_utils.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Timer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>

// Core Endurance library
#include "queue/enqueue_functions.h"
#include "queue/process_queue.h"
#include "queue/queue_logic.h"
#include <EnduranceConfig.h>
#include <core.h>

// Global state structures
DataRateCounters dataRateCounters = {0, 0, 0, 0, 0};

// Pocketbase connection

// Debug utils

// Reads a command from Serial and sends it to the gateway
void debugSendCommandToGateway() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      sendCommandToGateway(command);
    }
  }
}

void debugSendCommandToPad() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      sendCommandToPad(command);
      sendCommandToGateway(command); // Also send to gateway for debugging
    }
  }
}

void pocketbaseHeartbeatTask(void *pvParameters) {
  Serial.println("Pocketbase Heartbeat Task started."); // Debug print

  while (true) {
    if (pocketbaseState.isConnected) {
      Heartbeat heartbeat;

      // Protect ntpClient access with the mutex
      if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
        heartbeat.timestamp = ntpClient.getEpochTime();
        xSemaphoreGive(ntpClientMutex);
      } else {
        Serial.println("Failed to take NTP client mutex.");
        continue; // Skip this iteration if mutex acquisition fails
      }
      if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
        enqueueHeartbeat(heartbeat); // Enqueue heartbeat data
        xSemaphoreGive(requestQueueMutex);
      } else {
        Serial.println("Failed to take request queue mutex for heartbeat.");
      }
    } else {
      Serial.println(
          "Pocketbase is not connected. Skipping heartbeat."); // Debug print
    }

    vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for heartbeat interval
  }
}

void gatewayLinkTask(void *pvParameters) {
  while (true) {
    // Take the mutex before accessing the I2C bus
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      requestFlightDataChunk(dataRateCounters, flightDataState);
      // Release the mutex after accessing the I2C bus
      xSemaphoreGive(i2cMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(DATA_SEND_INTERVAL / 4)); // Poll every 250ms
  }
}

void padLinkTask(void *pvParameters) {
  while (true) {
    // Take the mutex before accessing the I2C bus
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      requestPadDataChunk(dataRateCounters, loadingDataState);
      // Release the mutex after accessing the I2C bus
      xSemaphoreGive(i2cMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(PAD_DATA_SEND_INTERVAL / 2)); // Poll every 500ms
  }
}

int deathStarLinkTaskFrequency = 2; // Frequency in Hz
int deathStarLinkTaskInterval =
    1000 / deathStarLinkTaskFrequency; // Interval in milliseconds

void deathStarLinkTask(void *pvParameters) {
  Serial.println("Death Star Link Task started."); // Debug print
  while (true) {

    // Take the mutex before accessing the I2C bus
    String batch;
    if (xSemaphoreTake(requestQueueMutex, portMAX_DELAY) == pdTRUE) {
      batch = processQueueBatch(
          20); // Process up to 20 requests at a time (reduced from 50)
      xSemaphoreGive(requestQueueMutex);
    } else {
      Serial.println(
          "Failed to take request queue mutex for batch processing.");
      vTaskDelay(
          pdMS_TO_TICKS(deathStarLinkTaskInterval)); // Wait before retrying
      continue; // Skip sending batch if mutex acquisition fails
    }

    Serial.print("Batch size: ");
    Serial.println(batch.length());
    if (!batch.isEmpty()) {
      // Check batch size before sending
      if (batch.length() > 2000) {
        Serial.print("WARNING: Large batch size: ");
        Serial.println(batch.length());
      }

      // Send the batch to Pocketbase
      Serial.println("Task heap memory: " +
                     String(uxTaskGetStackHighWaterMark(NULL))); // Debug print

      if (pocketbaseState.isConnected) {
        if (xSemaphoreTake(uploadPocketbaseMutex, portMAX_DELAY) == pdTRUE) {
          pocketbaseUploadConnection.batch(batch);
          xSemaphoreGive(uploadPocketbaseMutex);
        } else {
          Serial.println("Failed to take upload Pocketbase mutex.");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(
        deathStarLinkTaskInterval)); // Delay for batch processing interval
  }
}

void commandQueuesTask(void *pvParameters) {
  while (true) {
    // Process commands from the pad command queue
    String padCommand = padCommandQueue.dequeue();
    if (!padCommand.isEmpty()) {
      // Send the command to the pad
      Serial.println("Sending command to pad: " + padCommand);
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        sendCommandToPad(padCommand);
        xSemaphoreGive(i2cMutex);
      } else {
        Serial.println("Failed to take I2C mutex for sending command to pad.");
      }
    }

    // Process commands from the gateway command queue
    String gatewayCommand = gatewayCommandQueue.dequeue();
    if (!gatewayCommand.isEmpty()) {
      // Send the command to the gateway
      Serial.println("Sending command to gateway: " + gatewayCommand);
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        sendCommandToGateway(gatewayCommand);
        xSemaphoreGive(i2cMutex);
      } else {
        Serial.println(
            "Failed to take I2C mutex for sending command to gateway.");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
  }
}

void setupTasks() {
  Serial.println("Setting up tasks...");
  /*Serial.print("Free Heap before task setup: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");*/
  /*xTaskCreatePinnedToCore(pocketbaseHeartbeatTask, "PocketbaseHeartbeat",
     4096, NULL, 2, NULL, 1); // Task function*/
  /*Serial.println("Pocketbase heartbeat task created on core 1"); // Debug
  print Serial.print("Free Heap after hearthbeat task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");*/
  xTaskCreatePinnedToCore(gatewayLinkTask, "GatewayLink", 16 * 1024, NULL, 1,
                          NULL,
                          0); // Create gateway link task on core 1
  /*Serial.println("Gateway link task created on core 0"); // Debug print
  Serial.print("Free Heap after gateway link task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");*/
  xTaskCreatePinnedToCore(padLinkTask, "PadLink", 16 * 1024, NULL, 1, NULL,
                          0); // Create pad link task on core 1
  /*Serial.println("Pad link task created on core 0"); // Debug print
  Serial.print("Free Heap after pad link task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");*/
  /*xTaskCreatePinnedToCore(deathStarLinkTask, "DeathStarLink", 64 * 1024, NULL,
                          1, NULL,
                          1); // Create Death Star link task on core 0;*/
  /*Serial.println("Death Star link task created on core 1"); // Debug print
  Serial.print("Free Heap after Death Star link task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);

  Serial.println(" KB");*/
  /*xTaskCreatePinnedToCore(stateTask, "StateTask", 1024 * 32, NULL, 1, NULL,
                          0);                     // Create state task on core 0
  Serial.println("State task created on core 0"); // Debug print
  Serial.print("Free Heap after state task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");*/
  /*xTaskCreatePinnedToCore(launcherDataTask, "LauncherDataTask", 8192, NULL, 1,
                          NULL,
                          1); // Create launcher data task on core 1

  Serial.println("Launcher data task created on core 1"); // Debug print
  Serial.print("Free Heap after launcher data task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");*/
  xTaskCreatePinnedToCore(commandQueuesTask, "CommandQueues", 8192, NULL, 1,
                          NULL,
                          0); // Create command queues task on core 0
  Serial.println("Command queues task created on core 0"); // Debug print
  Serial.print("Free Heap after command queues task creation: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");
}

void setup() {
  Serial.begin(115200);
  initI2C();
  /*initWiFi([]() {},                     // disconnectedCallback
           []() {},                     // connectedCallback
           []() { initNtp(ntpClient); } // gotIPCallback
  );*/

  Serial.println("Horizon starting...");
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");

  /*pocketbaseState.launchControlCallback =
      onLaunchUpdateCallback; // Set the callback

  initPocketbase();

  Serial.println("Pocketbase initialized.");
  Serial.print("Free Heap after Pocketbase init: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");
*/
  // Ensure the mutexes are created before initializing tasks
  i2cMutex = xSemaphoreCreateMutex();
  requestQueueMutex = xSemaphoreCreateMutex();
  ntpClientMutex = xSemaphoreCreateMutex(); // Create NTP client mutex
  uploadPocketbaseMutex =
      xSemaphoreCreateMutex(); // Create upload Pocketbase mutex

  if (i2cMutex == NULL || requestQueueMutex == NULL || ntpClientMutex == NULL ||
      uploadPocketbaseMutex == NULL) {
    Serial.println("Failed to create mutexes!");
    while (true) {
      delay(1000); // Wait indefinitely if mutex creation fails
    }
  }

  setupTasks(); // Initialize tasks
  Serial.println("Horizon ready");
}

void loop() {
  static Timer dataRateTimer(1000); // Data rate print timer

  computeI2CDataRates(dataRateCounters);

  if (dataRateTimer.expired()) {
    printI2CDataRates(dataRateCounters);
    //  printFlightData(flightDataState.currentFlightData);
    printAirLoadingData(loadingDataState.currentAirLoadingData);
    printWaterLoadingData(loadingDataState.currentWaterLoadingData);
    String launchState;
    switch (loadingDataState.launchState // Print launch state
    ) {
    case STAND_BY:
      launchState = "Standby";
      break;
    case FILLING_WATER:
      launchState = "Filling Water";
      break;
    case FILLED_WATER:
      launchState = "Filled Water";
      break;
    case FILLING_AIR:
      launchState = "Filling Air";
      break;
    case READY_TO_LAUNCH:
      launchState = "Ready to Launch";
      break;
    case LAUNCHED:
      launchState = "Launched";
      break;
    case CANCELLING:
      launchState = "Cancelling";
      break;
    case LAUNCHING:
      launchState = "Launching";
      break;
    case CANCELLED:
      launchState = "Cancelled";
      break;
    default:
      launchState = "Invalid State: " + String(loadingDataState.launchState);
      break;
    }
    Serial.print("Launch State: ");
    Serial.println(launchState);
    /*
    // Monitor FreeRTOS tasks
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    Serial.print("Number of tasks: ");
    Serial.println(taskCount);

    // Print free heap
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");

    // Print current task stack usage (simplified approach)
    Serial.println("Current Task Stack Usage:");
    Serial.print("Loop task free stack: ");
    Serial.print(uxTaskGetStackHighWaterMark(NULL));
    Serial.println(" bytes");

    // Print system information
    Serial.print("Total Heap Size: ");
    Serial.print(ESP.getHeapSize() / 1024);
    Serial.println(" KB");

    Serial.print("Minimum Free Heap: ");
    Serial.print(ESP.getMinFreeHeap() / 1024);
    Serial.println(" KB");

    Serial.print("Free PSRAM: ");
    Serial.print(ESP.getFreePsram() / 1024);
    Serial.println(" KB");
  }
    */
    /*// Protect ntpClient update with the mutex
    if (xSemaphoreTake(ntpClientMutex, portMAX_DELAY) == pdTRUE) {
      ntpClient.update(); // Update NTP time
      xSemaphoreGive(ntpClientMutex);
    } else {
      Serial.println("Failed to take NTP client mutex for update.");
    }*/

    // debugSendCommandToGateway();
    debugSendCommandToPad();

    /*if (xSemaphoreTake(uploadPocketbaseMutex, portMAX_DELAY) == pdTRUE) {
      pocketbaseUploadConnection.update_subscription();
      xSemaphoreGive(uploadPocketbaseMutex);
    } else {
      Serial.println("Failed to take upload Pocketbase mutex for processing.");
    }*/
  }
}