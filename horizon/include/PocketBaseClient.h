#ifndef POCKETBASE_CLIENT_H
#define POCKETBASE_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <map>
#include <vector>

class PocketBaseClient {
public:
  PocketBaseClient(const String &host, uint16_t port);
  void setAuthCollection(const String &collectionName);
  bool login(const String &identity, const String &password);
  String getRecord(const String &collection, const String &recordId);
  String createRecord(const String &collection, const String &jsonData);
  String updateRecord(const String &collection, const String &recordId,
                      const String &jsonData);
  bool deleteRecord(const String &collection, const String &recordId);
  bool subscribeRealtime(const String &topic,
                         void (*onMessage)(const String &));
  void pollRealtime();
  bool isAuthenticated() const;
  String getToken() const;
  String getAuthRecordJson() const;
  String getRealtimeClientId() const;
  bool setRealtimeSubscriptions(const std::vector<String> &topics);
  // Add/Remove subscriptions
  void addRealtimeSubscription(const String &collection, const String &topic);
  void removeRealtimeSubscription(const String &collection,
                                  const String &topic);
  void clearRealtimeSubscriptions();
  // Start the realtime SSE connection and set the callback
  void startRealtime(void (*onMessage)(const String &));
  // Returns true if the underlying client is connected to the realtime API
  // (SSE)
  bool isRealtimeConnected() const;

private:
  String _host;
  uint16_t _port;
  String _token;
  String _authCollection = "users";
  WiFiClientSecure _client;
  String _sseBuffer;
  void (*_sseCallback)(const String &) = nullptr;
  String _authRecordJson;
  String _realtimeClientId;
  std::map<String, std::vector<String>> _realtimeSubscriptions;
  void _notifyRealtimeSubscriptions();
  // Internal HTTP helper
  String _sendHttpRequest(const String &method, const String &url,
                          const String &body);
};

#endif // POCKETBASE_CLIENT_H
