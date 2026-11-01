// EE4216 Lab 3 - ESP32-S3 B: Bidirectional IoT gateway (AP+STA)
//   Sensor node --POST /api/sensor--> Gateway --POST /update--> ThingSpeak
//   Sensor node <--POST /api/led----- Gateway <--GET last.json-- ThingSpeak

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "secrets.h"

// DigiCert Global Root G2 - root CA of api.thingspeak.com (valid until 2038-01-15)
static const char ROOT_CA[] PROGMEM = R"PEM(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)PEM";

const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

const unsigned long UPLOAD_INTERVAL_MS    = 20000; // ThingSpeak free tier: >= 15 s
const unsigned long POLL_INTERVAL_MS      = 5000;
const unsigned long RECONNECT_INTERVAL_MS = 30000; // fallback only; setAutoReconnect does the fast retries
const unsigned long READING_MAX_AGE_MS    = 15000; // don't upload stale readings
const uint16_t      HTTP_TIMEOUT_MS       = 4000;  // keep short so the local server stays responsive
const unsigned long LED_RETRY_INTERVAL_MS = 3000;
// Shorter than the sensor node's 3 s POST timeout: if both boards send to each other
// at the same moment, the gateway gives up first and the deadlock clears
const uint16_t      LED_TIMEOUT_MS        = 1500;

const float MIN_DISTANCE_CM = 2.0;   // HC-SR04 valid range
const float MAX_DISTANCE_CM = 400.0;

WebServer server(80);
WiFiClientSecure tlsClient;

// Latest sensor reading
bool          hasReading      = false;
float         latestDistance  = 0;
String        latestNode;
unsigned long latestRxMs      = 0;    // reception timestamp (millis)
unsigned long lastUploadedRxMs = 0;   // avoid re-uploading the same reading
IPAddress     sensorNodeIP;           // learned from the sensor node's POST

// LED command state (entry_id is the command identifier)
long lastCompletedEntryId = -1;
bool hasPendingCmd        = false;
long pendingEntryId       = 0;
int  pendingIndex         = 0;
int  pendingColor         = 0;

unsigned long lastUploadMs    = 0;
unsigned long lastPollMs      = 0;
unsigned long lastReconnectMs = 0;
bool          staWasConnected = false;

// ---------------------------------------------------------------- local server

void sendJson(int code, const char *status, const char *message) {
  JsonDocument doc;
  doc["status"] = status;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

void handleSensorPost() {
  if (!server.hasArg("plain")) {
    sendJson(400, "error", "missing body");
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    Serial.printf("[SENSOR] Invalid JSON: %s\n", err.c_str());
    sendJson(400, "error", "invalid JSON");
    return;
  }

  if (!doc["distance"].is<float>()) {
    Serial.println("[SENSOR] Rejected: missing or non-numeric distance");
    sendJson(400, "error", "missing or invalid distance");
    return;
  }
  float d = doc["distance"].as<float>();
  if (isnan(d) || d < MIN_DISTANCE_CM || d > MAX_DISTANCE_CM) {
    Serial.printf("[SENSOR] Rejected: distance %.1f out of range\n", d);
    sendJson(400, "error", "distance out of range");
    return;
  }

  latestDistance = d;
  latestNode     = doc["node"] | "unknown";
  latestRxMs     = millis();
  hasReading     = true;
  sensorNodeIP   = server.client().remoteIP();

  Serial.printf("[SENSOR] node=%s ip=%s distance=%.1f cm t=%lu ms\n",
                latestNode.c_str(), sensorNodeIP.toString().c_str(), latestDistance, latestRxMs);
  sendJson(200, "ok", "reading stored");
}

void handleStatus() {
  JsonDocument doc;
  doc["ap_ip"]            = WiFi.softAPIP().toString();
  doc["sta_ip"]           = WiFi.localIP().toString();
  doc["sta_connected"]    = WiFi.isConnected();
  doc["sensor_node_ip"]   = sensorNodeIP.toString();
  doc["distance"]         = latestDistance;
  doc["reading_age_ms"]   = hasReading ? millis() - latestRxMs : -1;
  doc["last_led_entry"]   = lastCompletedEntryId;
  String out;
  serializeJsonPretty(doc, out);
  server.send(200, "application/json", out);
}

// ---------------------------------------------------------------- Wi-Fi

void maintainStaConnection() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !staWasConnected) {
    Serial.printf("[WIFI] STA connected, IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else if (!connected && staWasConnected) {
    Serial.println("[WIFI] STA lost upstream connection; local network still running");
  }
  if (connected) lastReconnectMs = millis(); // fallback timer counts from when the link dropped
  staWasConnected = connected;

  // Non-blocking retry: never wait in a loop, so handleClient() keeps running
  if (!connected && millis() - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
    lastReconnectMs = millis();
    Serial.println("[WIFI] Retrying STA connection...");
    WiFi.reconnect(); // begin() fails with "sta is connecting" while an attempt is in progress
  }
}

// ---------------------------------------------------------------- cloud upload

void uploadToThingSpeak() {
  if (!hasReading || latestRxMs == lastUploadedRxMs) return;
  if (millis() - latestRxMs > READING_MAX_AGE_MS) {
    Serial.println("[UPLOAD] Skipped: latest reading is stale");
    return;
  }

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(tlsClient, "https://api.thingspeak.com/update")) {
    Serial.println("[UPLOAD] http.begin failed");
    return;
  }
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = String("api_key=") + SENSOR_WRITE_KEY +
                "&field1=" + String(latestDistance, 1) +
                "&field2=" + String(WiFi.RSSI());
  int code = http.POST(body);

  if (code <= 0) {
    Serial.printf("[UPLOAD] Transport error: %s\n", http.errorToString(code).c_str());
  } else {
    String resp = http.getString();
    resp.trim();
    long entryId = resp.toInt();
    // HTTP 200 alone isn't enough: ThingSpeak answers "0" when the update was not stored
    if (code == 200 && entryId > 0) {
      lastUploadedRxMs = latestRxMs;
      Serial.printf("[UPLOAD] OK distance=%.1f cm -> entry_id=%ld\n", latestDistance, entryId);
    } else {
      Serial.printf("[UPLOAD] Rejected: HTTP %d body=\"%s\"\n", code, resp.c_str());
    }
  }
  http.end();
}

// ---------------------------------------------------------------- cloud command poll

// ThingSpeak returns fields as strings ("3") or null; accept integers only
bool parseIntField(JsonVariant v, int minVal, int maxVal, int &out) {
  const char *s = v.as<const char *>();
  if (s == nullptr || *s == '\0') return false;
  char *end;
  long n = strtol(s, &end, 10);
  if (*end != '\0' || n < minVal || n > maxVal) return false;
  out = (int)n;
  return true;
}

void pollLedCommand() {
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  String url = String("https://api.thingspeak.com/channels/") + LED_CHANNEL_ID +
               "/feeds/last.json?api_key=" + LED_READ_KEY;
  if (!http.begin(tlsClient, url)) {
    Serial.println("[POLL] http.begin failed");
    return;
  }
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[POLL] HTTP error: %s\n",
                  code > 0 ? String(code).c_str() : http.errorToString(code).c_str());
    http.end();
    return;
  }
  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("[POLL] Invalid JSON (%s): %s\n", err.c_str(), payload.c_str());
    return;
  }
  if (!doc["entry_id"].is<long>()) {
    Serial.printf("[POLL] No entry_id in response: %s\n", payload.c_str()); // e.g. "-1" for empty channel
    return;
  }

  long entryId = doc["entry_id"];
  if (entryId <= lastCompletedEntryId) return;              // already executed
  if (hasPendingCmd && entryId == pendingEntryId) return;   // already queued, awaiting ack

  int index, color;
  if (!parseIntField(doc["field1"], 0, 7, index) || !parseIntField(doc["field2"], 0, 3, color)) {
    Serial.printf("[POLL] entry %ld invalid (field1=%s field2=%s), skipping\n", entryId,
                  doc["field1"] | "null", doc["field2"] | "null");
    lastCompletedEntryId = entryId; // don't re-check a bad command forever
    return;
  }

  hasPendingCmd  = true;
  pendingEntryId = entryId;
  pendingIndex   = index;
  pendingColor   = color;
  Serial.printf("[POLL] New command entry_id=%ld LED %d color %d\n", entryId, index, color);
}

// ---------------------------------------------------------------- forward to sensor node

void colorToRgb(int code, int &r, int &g, int &b) {
  r = g = b = 0;
  switch (code) {
    case 1: r = 255; break; // RED
    case 2: g = 255; break; // GREEN
    case 3: b = 255; break; // BLUE
    default: break;         // OFF
  }
}

void forwardPendingCommand() {
  if (!hasPendingCmd) return;
  if (sensorNodeIP == IPAddress()) {
    Serial.println("[LED] Sensor node IP unknown yet (no sensor POST received)");
    return;
  }

  int r, g, b;
  colorToRgb(pendingColor, r, g, b);

  JsonDocument doc;
  doc["index"] = pendingIndex;
  doc["r"] = r;
  doc["g"] = g;
  doc["b"] = b;
  String body;
  serializeJson(doc, body);

  HTTPClient http;
  http.setConnectTimeout(LED_TIMEOUT_MS);
  http.setTimeout(LED_TIMEOUT_MS);
  String url = "http://" + sensorNodeIP.toString() + "/api/led";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  String resp = code > 0 ? http.getString() : http.errorToString(code);
  http.end();

  if (code == 200) {
    // Mark completed only after the sensor node acknowledges
    lastCompletedEntryId = pendingEntryId;
    hasPendingCmd = false;
    Serial.printf("[LED] entry %ld acknowledged: %s\n", pendingEntryId, resp.c_str());
  } else {
    Serial.printf("[LED] entry %ld failed (%d): %s - will retry\n", pendingEntryId, code, resp.c_str());
  }
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // don't stall when no Serial Monitor is reading the USB port
#endif
  delay(500);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setAutoReconnect(true);
  WiFi.begin(STA_SSID, STA_PASS);
  lastReconnectMs = millis();

  tlsClient.setCACert(ROOT_CA);

  server.on("/api/sensor", HTTP_POST, handleSensorPost);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.onNotFound([]() { sendJson(404, "error", "not found"); });
  server.begin();

  Serial.println("\n=== EE4216 Lab 3 Gateway ===");
  Serial.printf("AP SSID: %s  AP IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("STA connecting to %s ...\n", STA_SSID);
}

void loop() {
  server.handleClient();
  maintainStaConnection();

  // Local forwarding works without Internet
  static unsigned long lastForwardMs = 0;
  if (hasPendingCmd && millis() - lastForwardMs >= LED_RETRY_INTERVAL_MS) {
    lastForwardMs = millis();
    forwardPendingCommand();
  }

  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() - lastUploadMs >= UPLOAD_INTERVAL_MS) {
    lastUploadMs = millis();
    uploadToThingSpeak();
  }
  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();
    pollLedCommand();
  }
}
