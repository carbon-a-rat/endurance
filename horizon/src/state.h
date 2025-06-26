#pragma once
#include "core.h"
#include "json_utils.h"
#include "queue/enqueue_functions.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <EnduranceConfig.h>
#include <NTPClient.h>
#include <Timer.h>
#include <WiFi.h>
#include <freertos/semphr.h>
#include <pocketbase.hpp>

#include <functional>
#include <stddef.h>
#include <stdint.h>

// Holds the state for flight data transfer
struct FlightDataState {
  FlightData flightDataReceived;
  FlightData currentFlightData = {0};
  FlightData previousFlightData = {0};
  uint8_t *flightDataBytes = (uint8_t *)&flightDataReceived;
  size_t flightDataOffset = 0;
  bool hasReceivedFlightData = false;
};

struct LoadingDataState {
  WaterLoadingData currentWaterLoadingData = {0};
  WaterLoadingData previousWaterLoadingData = {0};
  AirLoadingData currentAirLoadingData = {0};
  AirLoadingData previousAirLoadingData = {0};
  launchStates launchState = STAND_BY;
  launchStates previousLaunchState = STAND_BY;
  bool hasReceivedLoadingData = false;
}; // This struct is simpler than FlightDataState because it doesn't need to
   // handle byte offsets or partial data.

struct PocketbaseState {
  bool isConnected = false;
  DynamicJsonDocument launcherRecord =
      DynamicJsonDocument(2048); // Adjust size as needed
  DynamicJsonDocument launchRecord =
      DynamicJsonDocument(2048); // Adjust size as needed
  DynamicJsonDocument rocketRecord =
      DynamicJsonDocument(400); // Adjust size as needed
  std::function<void()> launchControlCallback;
};

enum horizonStates : uint8_t {
  HZ_STAND_BY,
  HZ_LOADING,
  HZ_READY,
  HZ_FLYING,
  HZ_LANDED,
  HZ_CANCELLED,
};

class CommandQueue {
public:
  CommandQueue() : head(0), tail(0) {
    // Initialize the queue to empty
    for (size_t i = 0; i < MAX_QUEUE_SIZE; ++i) {
      queue[i] = "";
    }
  }

  void enqueue(const String &command) {
    if ((tail + 1) % MAX_QUEUE_SIZE == head) {
      // Queue is full, drop the command
      return;
    }
    queue[tail] = command;
    tail = (tail + 1) % MAX_QUEUE_SIZE;
  }

  String dequeue() {
    if (head == tail) {
      // Queue is empty
      return "";
    }
    String command = queue[head];
    head = (head + 1) % MAX_QUEUE_SIZE;
    return command;
  }
  bool isEmpty() const { return head == tail; }

private:
  static const size_t MAX_QUEUE_SIZE = 10;
  String queue[MAX_QUEUE_SIZE];
  size_t head;
  size_t tail;
};

void stateTask(void *pvParameters);
void launcherDataTask(void *pvParameters);
void onFlightDataUpdate();
void onLoadingDataUpdate();
void onLaunchUpdateCallback();

// Helper functions for debugging - convert enums to readable strings
const char *launchStateToString(launchStates state);
const char *horizonStateToString(horizonStates state);

// Global state instances
extern PocketbaseState pocketbaseState;
extern FlightDataState flightDataState;
extern LoadingDataState loadingDataState;
extern CommandQueue padCommandQueue;
extern CommandQueue gatewayCommandQueue;

// Mutexes
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t requestQueueMutex;
extern SemaphoreHandle_t ntpClientMutex;        // Mutex for NTP client
extern SemaphoreHandle_t uploadPocketbaseMutex; // Mutex for Pocketbase uploads

extern WiFiUDP ntpUdp;
extern NTPClient ntpClient;

extern PocketbaseArduino pocketbaseUploadConnection;
