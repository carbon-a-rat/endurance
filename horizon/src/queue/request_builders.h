#pragma once
#include "json_utils.h"
#include "queue_logic.h"
#include "state.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <pocketbase_utils.h>

extern RequestQueue requestQueue;

// Function to process and send batch requests
String buildFlightDataRequest(const FlightDataRequest &flightDataRequest);
String
buildWaterLoadingDataRequest(const WaterLoadingDataRequest &waterDataRequest);
String buildAirLoadingDataRequest(const AirLoadingDataRequest &airDataRequest);
String buildLauncherDataRequest(const LauncherDataRequest &launcherDataRequest);
String buildHeartbeatRequest(const HeartbeatRequest &heartbeatRequest);
String buildLaunchUpdateRequest(const LaunchUpdateRequest &launchUpdateRequest);
String buildLogRequest(const LogRequest &logRequest);

String getEndpoint(const String &collectionName, const String &recordId);

String buildRequest(const QueuedRequestWrapper &requestWrapper);
