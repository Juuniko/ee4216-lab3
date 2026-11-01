// EE4216 Lab 3 - ESP32-S3 A: Sensor node
// Based on Lab 1 Task3_2d_DistanceBar (HC-SR04 + RGB LED bar)
//   HTTP client: POST /api/sensor to gateway every 5 s
//   HTTP server: POST /api/led from gateway to set one LED

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NewPing.h>
#include <Adafruit_NeoPixel.h>
#include "secrets.h"

const int TRIG_PIN = 4;
const int ECHO_PIN = 5;
const int LED_PIN = 14;
const int NUM_LEDS = 8;
const int MAX_DISTANCE_CM = 400;

const char *NODE_ID     = "sensor01";
const char *GATEWAY_URL = "http://192.168.4.1/api/sensor";

const unsigned long SEND_INTERVAL_MS      = 5000;
const unsigned long RECONNECT_INTERVAL_MS = 10000;

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE_CM);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);

unsigned long lastSendMs      = 0;
unsigned long lastReconnectMs = 0;
bool          wasConnected    = false;

// Returns distance in cm (1 decimal place), or -1 if no echo
float measureDistanceCm() {
  unsigned long us = sonar.ping();
  if (us == 0) return -1;  // no echo: don't report a fake "far away" value to the cloud
  return us * 0.0343f / 2.0f;
}

void sendJson(int code, const char *status, const char *message) {
  JsonDocument doc;
  doc["status"] = status;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

bool readInt(JsonDocument &doc, const char *key, int maxVal, int &out) {
  if (!doc[key].is<int>()) return false;
  int v = doc[key];
  if (v < 0 || v > maxVal) return false;
  out = v;
  return true;
}

// POST /api/led  {"index":3,"r":255,"g":0,"b":0}
void handleLedPost() {
  if (!server.hasArg("plain")) {
    sendJson(400, "error", "missing body");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    Serial.printf("[LED] Invalid JSON: %s\n", err.c_str());
    sendJson(400, "error", "invalid JSON");
    return;
  }

  int index, r, g, b;
  if (!readInt(doc, "index", NUM_LEDS - 1, index)) {
    sendJson(400, "error", "index must be an integer 0-7");
    return;
  }
  if (!readInt(doc, "r", 255, r) || !readInt(doc, "g", 255, g) || !readInt(doc, "b", 255, b)) {
    sendJson(400, "error", "r, g, b must be integers 0-255");
    return;
  }

  // Update only the specified LED; others keep their colour
  strip.setPixelColor(index, strip.Color(r, g, b));
  strip.show();

  JsonDocument ack;
  ack["status"] = "ok";
  ack["index"] = index;
  ack["r"] = r;
  ack["g"] = g;
  ack["b"] = b;
  String out;
  serializeJson(ack, out);
  server.send(200, "application/json", out);  // ack first, log after
  Serial.printf("[LED] LED %d set to (%d,%d,%d)\n", index, r, g, b);
}

void sendReading() {
  float cm = measureDistanceCm();
  if (cm < 0) {
    Serial.println("[SENSOR] No echo, skipping");
    return;
  }

  JsonDocument doc;
  doc["node"] = NODE_ID;
  doc["distance"] = serialized(String(cm, 1));
  String body;
  serializeJson(doc, body);

  HTTPClient http;
  http.setTimeout(3000);
  http.begin(GATEWAY_URL);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code > 0) {
    Serial.printf("[SENSOR] Sent %s -> HTTP %d %s\n", body.c_str(), code, http.getString().c_str());
  } else {
    Serial.printf("[SENSOR] Send failed: %s\n", http.errorToString(code).c_str());
  }
  http.end();
}

void maintainConnection() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    Serial.printf("[WIFI] Connected to gateway. Sensor node IP: %s  Gateway: %s\n",
                  WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str());
  } else if (!connected && wasConnected) {
    Serial.println("[WIFI] Lost connection to gateway");
  }
  wasConnected = connected;

  // Non-blocking retry so the loop keeps running
  if (!connected && millis() - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
    lastReconnectMs = millis();
    WiFi.reconnect(); // begin() fails with "sta is connecting" while an attempt is in progress
  }
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // don't stall when no Serial Monitor is reading the USB port
#endif
  strip.begin();
  strip.setBrightness(50);
  strip.clear();
  strip.show();

  WiFi.mode(WIFI_STA);
  WiFi.begin(GATEWAY_SSID, GATEWAY_PASS);
  lastReconnectMs = millis();

  server.on("/api/led", HTTP_POST, handleLedPost);
  server.onNotFound([]() { sendJson(404, "error", "not found"); });
  server.begin();

  Serial.println("\n=== EE4216 Lab 3 Sensor Node ===");
}

void loop() {
  server.handleClient();
  maintainConnection();

  if (WiFi.status() == WL_CONNECTED && millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    sendReading();
  }
}
