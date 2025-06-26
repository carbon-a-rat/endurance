#include "request_builders.h"
#include "queue_logic.h"

String collectionsEndpoint = "/api/collections/";

String getEndpoint(const String &collectionName, const String &recordId = "") {
  String endpoint = collectionsEndpoint + collectionName + "/records";
  if (!recordId.isEmpty()) {
    endpoint += "/" + recordId;
  }
  return endpoint;
}

String buildFlightDataRequest(const FlightDataRequest &flightDataRequest) {
  String request;

  // Estimate the required size for the JSON document
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(12) + 512;
  DynamicJsonDocument requestJson(capacity);

  requestJson["method"] = "POST";
  requestJson["url"] = getEndpoint("flight_data");
  requestJson["body"]["launch"] = pocketbaseState.launchRecord["record"]["id"];
  requestJson["body"]["time_rel"] = flightDataRequest.data.timestamp;
  requestJson["body"]["acc_x"] = flightDataRequest.data.ax;
  requestJson["body"]["acc_y"] = flightDataRequest.data.ay;
  requestJson["body"]["acc_z"] = flightDataRequest.data.az;
  requestJson["body"]["gyro_x"] = flightDataRequest.data.gx;
  requestJson["body"]["gyro_y"] = flightDataRequest.data.gy;
  requestJson["body"]["gyro_z"] = flightDataRequest.data.gz;
  requestJson["body"]["altitude"] = flightDataRequest.data.altitude;
  requestJson["body"]["pressure"] = flightDataRequest.data.pressure;
  requestJson["body"]["temperature"] = flightDataRequest.data.temperature;

  return serializeJson(requestJson, request) ? request : "";
}

String
buildWaterLoadingDataRequest(const WaterLoadingDataRequest &waterDataRequest) {
  String request;

  // Estimate the required size for the JSON document
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(6) + 256;
  DynamicJsonDocument requestJson(capacity);

  requestJson["method"] = "POST";
  requestJson["url"] = getEndpoint("water_loading_data");
  requestJson["body"]["launch"] = pocketbaseState.launchRecord["record"]["id"];
  requestJson["body"]["time_rel"] = waterDataRequest.data.timestamp;
  requestJson["body"]["volume"] = waterDataRequest.data.waterVolume;
  requestJson["body"]["error"] = waterDataRequest.data.error;
  requestJson["body"]["command"] =
      pocketbaseState.launchRecord["record"]["water_volumic_percentage"]
          .as<float>() *
      pocketbaseState.rocketRecord["volume"].as<float>();

  return serializeJson(requestJson, request) ? request : "";
}

String buildAirLoadingDataRequest(const AirLoadingDataRequest &airDataRequest) {
  String request;

  // Estimate the required size for the JSON document
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(5) + 256;
  DynamicJsonDocument requestJson(capacity);

  requestJson["method"] = "POST";
  requestJson["url"] = getEndpoint("air_loading_data");
  requestJson["body"]["launch"] = pocketbaseState.launchRecord["record"]["id"];
  requestJson["body"]["time_rel"] = airDataRequest.data.timestamp;
  requestJson["body"]["pressure"] = airDataRequest.data.pressure;
  requestJson["body"]["error"] = airDataRequest.data.error;
  requestJson["body"]["command"] =
      pocketbaseState.launchRecord["record"]["pressure"].as<float>();

  return serializeJson(requestJson, request) ? request : "";
}

String
buildLauncherDataRequest(const LauncherDataRequest &launcherDataRequest) {
  String request;

  // Estimate the required size for the JSON document
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(7) + 256;
  DynamicJsonDocument requestJson(capacity);

  requestJson["method"] = "PATCH";
  requestJson["url"] = getEndpoint(
      "launchers", pocketbaseState.launcherRecord["record"]["id"].as<String>());
  requestJson["body"]["rocket_connected"] =
      launcherDataRequest.data.isProbeConnected;
  requestJson["body"]["rocket_battery_level"] =
      launcherDataRequest.data.probeBatteryLevel;
  requestJson["body"]["last_pressure"] = launcherDataRequest.data.lastPressure;
  requestJson["body"]["last_water_flow"] =
      launcherDataRequest.data.lastWaterFlow;
  requestJson["body"]["last_acc"] = launcherDataRequest.data.lastAcc;
  requestJson["body"]["last_altitude"] = launcherDataRequest.data.lastAltitude;

  return serializeJson(requestJson, request) ? request : "";
}

String buildHeartbeatRequest(const HeartbeatRequest &heartbeatRequest) {
  String request;

  // Estimate the required size for the JSON document
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + 128;
  DynamicJsonDocument requestJson(capacity);

  requestJson["method"] = "PATCH";
  requestJson["url"] = getEndpoint(
      "launchers", pocketbaseState.launcherRecord["record"]["id"].as<String>());
  requestJson["body"]["online"] = true;
  requestJson["body"]["last_ping_at"] =
      epochToPocketbaseTime(heartbeatRequest.data.timestamp);

  return serializeJson(requestJson, request) ? request : "";
}

String
buildLaunchUpdateRequest(const LaunchUpdateRequest &launchUpdateRequest) {
  String request;

  // Estimate the required size for the JSON document
  const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(2) + 128;
  DynamicJsonDocument requestJson(capacity);

  requestJson["method"] = "PATCH";
  requestJson["url"] = getEndpoint(
      "launches", pocketbaseState.launchRecord["record"]["id"].as<String>());

  String timestamp = epochToPocketbaseTime(launchUpdateRequest.data.timestamp);
  switch (launchUpdateRequest.data.type) {
  case STARTED_WATER_LOADING:
    requestJson["body"]["water_loading_started_at"] = timestamp;
    break;
  case STARTED_AIR_LOADING:
    requestJson["body"]["air_loading_started_at"] = timestamp;
    break;
  case LOAD_COMPLETED:
    requestJson["body"]["loaded_at"] = timestamp;
    break;
  case FIRED:
    requestJson["body"]["fired_at"] = timestamp;
    break;
  case LANDED:
    requestJson["body"]["landed_at"] = timestamp;
    break;
  case CANCELLED_:
    requestJson["body"]["cancelled_at"] = timestamp;
    break;
  default:
    Serial.println("Unknown launch update type");
    return "";
  }
  return serializeJson(requestJson, request) ? request : "";
}

String buildLogRequest(const LogRequest &logRequest) {
  String request;
  DynamicJsonDocument requestJson(1024);
  requestJson["method"] = "POST";
  requestJson["url"] = getEndpoint("launcher_logs");
  requestJson["body"]["launcher"] =
      pocketbaseState.launcherRecord["record"]["id"];
  if (!pocketbaseState.launchRecord.isNull()) {
    requestJson["body"]["launch"] =
        pocketbaseState.launchRecord["record"]["id"];
  }
  requestJson["body"]["timestamp"] =
      epochToPocketbaseTime(logRequest.data.timestamp);
  String levelStr;
  switch (logRequest.data.level) {
  case LOG_DEBUG:
    levelStr = "debug";
    break;
  case LOG_INFO:
    levelStr = "info";
    break;
  case LOG_WARNING:
    levelStr = "warning";
    break;
  case LOG_ERROR:
    levelStr = "error";
    break;
  default:
    Serial.println("Unknown log level");
    return "";
  }
  requestJson["body"]["level"] = levelStr;
  requestJson["body"]["event"] = logRequest.data.event;
  // Ensure the event string is not empty
  if (logRequest.data.event.isEmpty()) {
    Serial.println("Log event is empty");
    return "";
  }

  requestJson["body"]["message"] = logRequest.data.message;

  return serializeJson(requestJson, request) ? request : "";
}

String buildRequest(const QueuedRequestWrapper &wrapper) {
  switch (wrapper.type) {
  case FLIGHT_DATA:
    return buildFlightDataRequest(wrapper.request.flightDataRequest);
  case WATER_LOADING_DATA:
    return buildWaterLoadingDataRequest(
        wrapper.request.waterLoadingDataRequest);
  case AIR_LOADING_DATA:
    return buildAirLoadingDataRequest(wrapper.request.airLoadingDataRequest);
  case LAUNCHER_DATA:
    return buildLauncherDataRequest(wrapper.request.launcherDataRequest);
  case HEARTBEAT:
    return buildHeartbeatRequest(wrapper.request.heartbeatRequest);
  case LAUNCH_UPDATE:
    return buildLaunchUpdateRequest(wrapper.request.launchUpdateRequest);
  case LOG:
    return buildLogRequest(wrapper.request.logRequest);
  default:
    Serial.println("Unknown request type");
    return "";
  }
}