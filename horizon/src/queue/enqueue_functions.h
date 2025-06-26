#pragma once
#include "queue_logic.h"
#include <ArduinoJson.h>

extern RequestQueue requestQueue;

// Enqueue flight data
void enqueueFlightData(const FlightData &data);
void enqueueWaterLoadingData(const WaterLoadingData &data);
void enqueueAirLoadingData(const AirLoadingData &data);
void enqueueLauncherData(const LauncherData &data);
void enqueueHeartbeat(const Heartbeat &data);
void enqueueLaunchUpdate(const LaunchUpdate &data);
void enqueueLog(const LauncherLog &data);