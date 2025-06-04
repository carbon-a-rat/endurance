#include "PocketBaseClient.h"
#include <ArduinoJson.h>

PocketBaseClient::PocketBaseClient(const String &host, uint16_t port)
    : _host(host), _port(port), _token(""), _authCollection("users") {
  _client.setInsecure(); // For self-signed certs; use setCACert for production
                         // on ESP32
}

void PocketBaseClient::setAuthCollection(const String &collectionName) {
  _authCollection = collectionName;
}

bool PocketBaseClient::login(const String &identity, const String &password) {
  int status = _client.connect(_host.c_str(), _port);
  if (!status) {
    Serial.println("[PocketBase] Connection failed: " + String(status));
    return false;
  }
  String url = "/api/collections/" + _authCollection + "/auth-with-password";
  String payload =
      "{\"identity\":\"" + identity + "\",\"password\":\"" + password + "\"}";
  String request = "POST " + url + " HTTP/1.1\r\n";
  request += "Host: " + _host + "\r\n";
  request += "Content-Type: application/json\r\n";
  request += "Content-Length: " + String(payload.length()) + "\r\n";
  request += "Connection: close\r\n\r\n";
  request += payload;
  _client.print(request);
  String response = _client.readStringUntil('\0');
  int bodyIdx = response.indexOf("\r\n\r\n");
  if (bodyIdx < 0)
    return false;
  String body = response.substring(bodyIdx + 4);
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.println("[PocketBase] JSON parse error: " + String(err.c_str()));
    return false;
  }
  if (doc["token"]) {
    _token = doc["token"].as<String>();
    if (doc["record"]) {
      // Store the auth record as JSON string
      String recordJson;
      serializeJson(doc["record"], recordJson);
      _authRecordJson = recordJson;
    } else {
      _authRecordJson = "";
    }
    return true;
  }
  _authRecordJson = "";
  Serial.println("[PocketBase] Login failed, no token in response");
  return false;
}

bool PocketBaseClient::isAuthenticated() const { return _token.length() > 0; }

String PocketBaseClient::getToken() const { return _token; }

String PocketBaseClient::getAuthRecordJson() const { return _authRecordJson; }

// Returns true if the underlying client is connected to the realtime API (SSE)
bool PocketBaseClient::isRealtimeConnected() const {
  return _realtimeClientId.length() > 0;
}

// --- Internal HTTP helper ---
String PocketBaseClient::_sendHttpRequest(const String &method,
                                          const String &url,
                                          const String &body) {
  if (!_client.connect(_host.c_str(), _port))
    return "";
  String request = method + " " + url + " HTTP/1.1\r\n";
  request += "Host: " + _host + "\r\n";
  if (method == "POST" || method == "PATCH")
    request += "Content-Type: application/json\r\n";
  if (_token.length())
    request += "Authorization: " + _token + "\r\n";
  if (method == "POST" || method == "PATCH")
    request += "Content-Length: " + String(body.length()) + "\r\n";
  request += "Connection: close\r\n\r\n";
  if (method == "POST" || method == "PATCH")
    request += body;
  _client.print(request);
  String response = _client.readStringUntil('\0');
  int bodyIdx = response.indexOf("\r\n\r\n");
  if (bodyIdx < 0)
    return "";
  return response.substring(bodyIdx + 4);
}

// --- CRUD methods using the helper ---
String PocketBaseClient::getRecord(const String &collection,
                                   const String &recordId) {
  String url = "/api/collections/" + collection + "/records/" + recordId;
  return _sendHttpRequest("GET", url, "");
}

String PocketBaseClient::createRecord(const String &collection,
                                      const String &jsonData) {
  String url = "/api/collections/" + collection + "/records";
  return _sendHttpRequest("POST", url, jsonData);
}

String PocketBaseClient::updateRecord(const String &collection,
                                      const String &recordId,
                                      const String &jsonData) {
  String url = "/api/collections/" + collection + "/records/" + recordId;
  return _sendHttpRequest("PATCH", url, jsonData);
}

bool PocketBaseClient::deleteRecord(const String &collection,
                                    const String &recordId) {
  String url = "/api/collections/" + collection + "/records/" + recordId;
  if (!_client.connect(_host.c_str(), _port))
    return false;
  String request = "DELETE " + url + " HTTP/1.1\r\n";
  request += "Host: " + _host + "\r\n";
  if (_token.length())
    request += "Authorization: " + _token + "\r\n";
  request += "Connection: close\r\n\r\n";
  _client.print(request);
  String response = _client.readStringUntil('\0');
  return response.indexOf("204 No Content") > 0;
}

// --- SSE (Realtime) ---
void PocketBaseClient::addRealtimeSubscription(const String &collection,
                                               const String &topic) {
  auto &topics = _realtimeSubscriptions[collection];
  if (std::find(topics.begin(), topics.end(), topic) == topics.end()) {
    topics.push_back(topic);
    _notifyRealtimeSubscriptions();
  }
}

void PocketBaseClient::removeRealtimeSubscription(const String &collection,
                                                  const String &topic) {
  auto it = _realtimeSubscriptions.find(collection);
  if (it != _realtimeSubscriptions.end()) {
    auto &topics = it->second;
    topics.erase(std::remove(topics.begin(), topics.end(), topic),
                 topics.end());
    if (topics.empty()) {
      _realtimeSubscriptions.erase(it);
    }
    _notifyRealtimeSubscriptions();
  }
}

void PocketBaseClient::clearRealtimeSubscriptions() {
  _realtimeSubscriptions.clear();
  _notifyRealtimeSubscriptions();
}

void PocketBaseClient::_notifyRealtimeSubscriptions() {
  if (_realtimeClientId.length() == 0) {
    Serial.println("[PocketBase] No realtime clientId yet");
    return;
  }
  if (!_client.connect(_host.c_str(), _port)) {
    Serial.println("[PocketBase] Failed to connect for subscriptions");
    return;
  }
  String url = "/api/realtime";
  String body = "{\"clientId\":\"" + _realtimeClientId + "\"";
  std::vector<String> topics;
  for (const auto &pair : _realtimeSubscriptions) {
    const String &collection = pair.first;
    for (const String &topic : pair.second) {
      if (topic.length() > 0) {
        topics.push_back(collection + "/" + topic);
      } else {
        topics.push_back(collection);
      }
    }
  }
  if (!topics.empty()) {
    body += ",\"subscriptions\": [";
    for (size_t i = 0; i < topics.size(); ++i) {
      body += "\"" + topics[i] + "\"";
      if (i < topics.size() - 1)
        body += ",";
    }
    body += "]";
  }
  body += "}";
  String request = "POST " + url + " HTTP/1.1\r\n";
  request += "Host: " + _host + "\r\n";
  request += "Content-Type: application/json\r\n";
  if (_token.length())
    request += "Authorization: " + _token + "\r\n";
  request += "Content-Length: " + String(body.length()) + "\r\n";
  request += "Connection: close\r\n\r\n";
  request += body;
  _client.print(request);
  String response = _client.readStringUntil('\0');
  // Optionally check for errors in respBody
}

void PocketBaseClient::startRealtime(void (*onMessage)(const String &)) {
  if (!_token.length()) {
    Serial.println("[PocketBase] Not authenticated, cannot start realtime");
    return;
  }
  if (!_client.connect(_host.c_str(), _port)) {
    Serial.println("[PocketBase] Failed to connect for SSE");
    return;
  }
  String url = "/api/realtime";
  String request = "GET " + url + " HTTP/1.1\r\n";
  request += "Host: " + _host + "\r\n";
  request += "Authorization: " + _token + "\r\n";
  request += "Accept: text/event-stream\r\n";
  request += "Connection: keep-alive\r\n\r\n";
  _client.print(request);
  _sseBuffer = "";
  _sseCallback = onMessage;
}

void PocketBaseClient::pollRealtime() {
  while (_client.connected() && _client.available()) {
    String line = _client.readStringUntil('\n');
    if (line.startsWith("event: PB_CONNECT")) {
      // Next line should be data: {"clientId":"..."}
      String dataLine = _client.readStringUntil('\n');
      if (dataLine.startsWith("data: ")) {
        String jsonStr = dataLine.substring(6);
        DynamicJsonDocument doc(256);
        DeserializationError err = deserializeJson(doc, jsonStr);
        if (!err && doc["clientId"]) {
          _realtimeClientId = doc["clientId"].as<String>();
          Serial.print("[PocketBase] Realtime clientId: ");
          Serial.println(_realtimeClientId);
          // Notify server of current subscriptions as soon as we get clientId
          _notifyRealtimeSubscriptions();
        }
      }
    } else if (line.startsWith("data: ")) {
      String data = line.substring(6);
      if (_sseCallback) {
        _sseCallback(data);
      } else {
        Serial.println("[SSE] " + data);
      }
    }
  }
}
