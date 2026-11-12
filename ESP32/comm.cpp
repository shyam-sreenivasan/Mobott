#include "comm.h"

static bool parseVelocity(const String &msg, float &v, float &w) {
  int comma = msg.indexOf(',');
  if (comma <= 0) return false;
  v = msg.substring(0, comma).toFloat();
  w = msg.substring(comma + 1).toFloat();
  return true;
}

#ifdef COMM_WIFI

#include <WiFi.h>
#include "secrets.h"

static const char* SSID     = WIFI_SSID;
static const char* PASSWORD = WIFI_PASSWORD;

static WiFiServer server(1234);
static WiFiClient client;

static bool serverStarted = false;
static unsigned long lastRetryTime = 0;
static const unsigned long RETRY_INTERVAL_MS = 5000;

// Keeps WiFi and the TCP server alive. Called on every loop via commRead().
// Never blocks, so the control loop (and the motor safety timeout) keep running.
static void commMaintain() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !serverStarted) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    server.begin();
    serverStarted = true;
  }

  if (!connected && serverStarted) {
    Serial.println("WiFi lost. Will keep retrying...");
    client.stop();
    server.end();
    serverStarted = false;
    lastRetryTime = millis();
  }

  if (!connected && millis() - lastRetryTime >= RETRY_INTERVAL_MS) {
    lastRetryTime = millis();
    Serial.println("Retrying WiFi...");
    WiFi.disconnect();
    WiFi.begin(SSID, PASSWORD);
  }
}

void commInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);    // clear any stored config from a previous mode
  WiFi.setSleep(false);     // keep the radio awake: much lower command delay
  WiFi.setAutoReconnect(true);

  Serial.println("Connecting WiFi (will keep retrying in the background)...");
  WiFi.begin(SSID, PASSWORD);
  lastRetryTime = millis();
}

bool commRead(float &v, float &w) {
  commMaintain();
  if (!serverStarted) return false;

  if (!client || !client.connected()) {
    client = server.available();
    return false;
  }

  if (!client.available()) return false;

  String msg = client.readStringUntil('\n');
  return parseVelocity(msg, v, w);
}

void commWrite(const String &line) {
  if (client && client.connected()) {
    client.println(line);
  }
}

#else  // Serial

void commInit() {
  // Serial already started in robot.ino
}

bool commRead(float &v, float &w) {
  if (!Serial.available()) return false;
  String msg = Serial.readStringUntil('\n');
  return parseVelocity(msg, v, w);
}

void commWrite(const String &line) {
  Serial.println(line);
}

#endif
