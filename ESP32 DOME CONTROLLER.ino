/*
  OPTIMIZED ESP32 DOME CONTROLLER (REV 28 - Sensor-mode compile flag)
  - REMOVED: AsyncUDP (Discovery) to free resources and prevent stack conflicts.
  - OPTIMIZATION: Pure TCP focus for maximum polling speed.
  - CONFIG: Requires manual IP entry in NINA (Discovery disabled).
  - REV 28: Added SENSOR_TYPE compile flag to support either the original
           magnetic reed sensors or IR obstacle sensors. See "SENSOR MODE
           REFERENCE" below the config block for wiring and semantics.
*/
 
#include <WiFi.h>
// #include <AsyncUDP.h> // REMOVED: Discovery disabled
#include <ESPAsyncWebServer.h> 
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPmDNS.h>
#include <Preferences.h>
 
// ========= USER SETTINGS =========
const char* ssid     = "SSID";
const char* password = "PASSWORD";
 
#define DEFAULT_ALPACA_PORT    11111
// #define DEFAULT_DISCOVERY_PORT 32227 // REMOVED
#define WIFI_CHECK_INTERVAL     30000UL
#define WIFI_RETRY_INTERVAL     5000UL
 
// ========= HARDWARE CONFIG =========
// --- Pin assignments ---
#define RELAY_PIN       26   // Motor-control relay. Single pulse toggles direction at the driver.
#define ROOF_OPEN_PIN   33   // Magnetic: open-limit reed.  IR: either of the two IR sensors.
#define ROOF_CLOSED_PIN 32   // Magnetic: closed-limit reed. IR: the other IR sensor.
#define PARK_SAFE_PIN   27   // Park-safe input (scope parked / safe to move roof).

// --- Sensor technology ---
// Pick one. See "SENSOR MODE REFERENCE" below for wiring, placement and logic.
#define SENSOR_MAGNETIC_REED 0   // Two NC reed switches + magnets on the roof (original).
#define SENSOR_IR_BEAM       1   // Two NO IR sensors looking up at the roof underside.
#define SENSOR_TYPE          SENSOR_MAGNETIC_REED

// Logic level read when a sensor reports "triggered".
//   Reed NC with INPUT_PULLUP:                    LOW
//   Typical 5V IR obstacle module (OC / pulled):  LOW
//   Active-high push-pull IR module (rare):       HIGH
#define SENSOR_ACTIVE_LEVEL  LOW

// ============================================================================
// SENSOR MODE REFERENCE
// ============================================================================
// SENSOR_MAGNETIC_REED (original behaviour)
// ------------------------------------------
//   Two NC reed switches are fixed to the wall. Two magnets ride on the roof.
//     * One magnet lands over the CLOSED reed when the roof is fully shut.
//     * The other magnet lands over the OPEN reed at the open-stop position
//       (typically ~75 % travel, chosen to expose the pier only).
//   Only one reed is triggered at a time:
//       CLOSED_PIN active, OPEN_PIN inactive  -> CLOSED
//       OPEN_PIN   active, CLOSED_PIN inactive -> OPEN
//       neither active (while slewing)        -> transit; >60 s -> ERROR
//   Caveats:
//     * ~5 mm placement tolerance. Roof drift of ~1 cm makes the reed miss the
//       magnet, the driver reports "lost" and NINA sequences abort.
//     * Cannot be used to detect a roof that fully overshoots the open-stop
//       (e.g. to expose pier AND roof eaves): both reeds read inactive, which
//       is indistinguishable from the lost state.
//
// SENSOR_IR_BEAM
// --------------
//   Two NO IR obstacle/reflection modules are mounted on the wall, looking up
//   at the roof underside. The roof itself is the target - no magnets needed.
//   Place them so that:
//     * both sensors sit under the roof when it is fully CLOSED, and
//     * both sensors are clear of the roof at the desired fully-OPEN position.
//   Logic (deliberately trivial, matches the "Planned" diagram):
//       both triggered   -> CLOSED
//       both UNtriggered -> OPEN
//       exactly one      -> in transit (counts toward the 60 s lost timer)
//   Why this mode exists:
//     * Tolerates ~1-2 cm of roof drift - the beam hits the roof anywhere over
//       a wide footprint, so minor misalignment no longer drops the signal.
//     * Supports roofs that fully overshoot the opening. "Both inactive" is
//       now an explicit OPEN state instead of an ambiguous lost state.
//   Notes:
//     * SENSOR_ACTIVE_LEVEL is LOW for most cheap 5V IR modules. Verify yours
//       with a multimeter: output goes toward 0 V when an obstacle is near.
//     * Shield the IR receiver from direct sunlight on the sensor face.
//     * In this mode the two sensors are symmetric - either physical sensor
//       may be wired to either pin.
// ============================================================================
 
// ========= STATE =========
enum ShutterState {
  SHUTTER_OPEN = 0,
  SHUTTER_CLOSED = 1,
  SHUTTER_OPENING = 2,
  SHUTTER_CLOSING = 3,
  SHUTTER_ERROR = 4
};
 
ShutterState currentShutterState = SHUTTER_CLOSED;
bool isSlewing = false;
unsigned long lastCommandTime = 0;
bool roofLost = false;
bool domeConnected = false;
bool safetyConnected = false;
bool relayActive = false;
unsigned long relayOffTime = 0;
 
volatile int serverTransactionID = 0;
unsigned long lastWifiCheck = 0;
bool wifiConnecting = false;
 
int currentAlpacaPort = DEFAULT_ALPACA_PORT;
 
// ========= OBJECTS =========
AsyncWebServer *server = NULL;
// AsyncUDP udp; // REMOVED
Preferences preferences;
 
// ========= HTML (Minified) =========
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8" /><meta name="viewport" content="width=device-width, initial-scale=1" /><title>ESP32 Dome Control</title><style> *, *::before, *::after { box-sizing: border-box; } body, html { margin: 0; padding: 0; height: 100%; background-color: #121212; color: #eee; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Ubuntu, sans-serif; display: flex; justify-content: center; align-items: flex-start; padding: 1rem 0.5rem; user-select: none; } main { width: 100%; max-width: 400px; background: #1c1c1c; border: 1px solid #333; box-shadow: inset 0 0 8px #222, 0 0 16px #0a84ff33; padding: 16px 20px; display: flex; flex-direction: column; gap: 1rem; color: #ddd; } header { text-align: center; } header h1 { font-weight: 600; font-size: 1.4rem; letter-spacing: 0.12em; margin-bottom: 6px; color: #0af; } header p { font-size: 0.8rem; color: #555; margin-top: -4px; letter-spacing: 0.08em; } #domestatus { margin-top: 8px; font-weight: 700; text-transform: uppercase; font-size: 1.2rem; letter-spacing: 0.12em; padding: 6px 0; border-top: 1px solid #333; border-bottom: 1px solid #333; color: #0af; } .status-open { color: #6cf98b; } .status-closed { color: #5c98fc; } .status-moving { color: #ffcc00; } .status-error { color: #ff5252; } section { display: flex; flex-direction: column; gap: 0.6rem; } .sensor { display: flex; justify-content: space-between; font-family: monospace; font-size: 0.95rem; padding: 4px 0; border-bottom: 1px solid #2a2a2a; } .sensor:last-child { border-bottom: none; } .sensor-name { color: #999; text-transform: uppercase; letter-spacing: 0.07em; font-weight: 600; } .sensor-value { font-weight: 700; text-transform: capitalize; } .trig { color: #5ce35c; } .notrig { color: #666; } .error { color: #ff4f4f; } .safe-status { text-align: center; font-weight: 700; text-transform: uppercase; letter-spacing: 0.12em; padding: 6px 0; user-select: none; } .safe-status.safe { color: #6cf98b; border-top: 1px solid #3a7d3a; border-bottom: 1px solid #3a7d3a; } .safe-status.notSafe { color: #ff5252; border-top: 1px solid #7d3a3a; border-bottom: 1px solid #7d3a3a; } button { width: 100%; padding: 14px 12px; background: #222; border: none; color: #ddd; font-size: 1rem; font-weight: 600; letter-spacing: 0.1em; text-transform: uppercase; cursor: pointer; transition: background 0.15s ease; user-select: none; border-left: 3px solid transparent; outline-offset: 2px; } button:focus-visible { outline: 2px solid #0af; outline-offset: 3px; } button:active:not(:disabled) { background: #333; } button:disabled { background: #111; color: #555; cursor: default; border-left: 3px solid #444; } #sopenbtn { border-left-color: #6cf98b; margin-bottom: 10px; } #sclosebtn { border-left-color: #5c98fc; } .override-btn { font-size: 0.84rem; font-weight: 500; padding: 10px 12px; background: #111; color: #999; border-left: 3px solid transparent; } .override-btn:enabled:hover { color: #0af; border-left-color: #0af; background: #191919; } .override-btn:disabled { color: #333; border-left-color: #333; cursor: default; background: #111; } .override-wrap { display: flex; justify-content: space-between; gap: 10px; margin-top: 0.6rem; } .override-wrap div { flex: 1; } .note { font-size: 0.8rem; color: #555; user-select: none; padding-top: 0.6rem; border-top: 1px solid #333; line-height: 1.3; letter-spacing: 0.02em; } </style></head><body><main role="main"><header><h1>ESP32 Dome Control</h1><p>Remote roof controller</p><div id="domestatus" class="status-error">Loading...</div></header><section><div id="safetystatus" class="safe-status notSafe">Loading...</div></section><section><button id="sopenbtn" onclick="postCmd('/sopen')">Open Dome</button><button id="sclosebtn" onclick="postCmd('/sclose')">Close Dome</button></section><section class="override-wrap"><div><button id="openbtn" onclick="postCmd('/open')" class="override-btn">Override Open</button></div><div><button id="closebtn" onclick="postCmd('/close')" class="override-btn">Override Close</button></div></section><section><div class="sensor"><div class="sensor-name">Roof Open</div><div id="roofopen" class="sensor-value notrig">Loading</div></div><div class="sensor"><div class="sensor-name">Roof Closed</div><div id="roofclosed" class="sensor-value notrig">Loading</div></div><div class="sensor"><div class="sensor-name">Park Sensor</div><div id="parksensor" class="sensor-value error">Loading</div></div></section><div class="note"><strong>Note:</strong> Safe dome open/close buttons require the mount to be parked. Override commands ignore the park sensor and work regardless.</div></main><script> function postCmd(url) { fetch(url, { method: 'POST' }) .then(response => { if (!response.ok) alert('Command failed'); }) .catch(error => console.error('Error:', error)); } function updateStatus() { fetch('/status', {cache: 'no-store'}) .then(res => res.json()) .then(data => { const domeStatusEl = document.getElementById('domestatus'); domeStatusEl.textContent = data.domeStatus || 'ERROR'; domeStatusEl.className = 'status-' + ((data.domeStatus || '').toLowerCase()); const safeEl = document.getElementById('safetystatus'); safeEl.textContent = data.safetyStatus || 'Error'; safeEl.className = data.safetySafe ? 'safe-status safe' : 'safe-status notSafe'; const roofOpenEl = document.getElementById('roofopen'); roofOpenEl.textContent = data.roofOpen ? 'Triggered' : 'Not triggered'; roofOpenEl.className = 'sensor-value ' + (data.roofOpen ? 'trig' : 'notrig'); const roofClosedEl = document.getElementById('roofclosed'); roofClosedEl.textContent = data.roofClosed ? 'Triggered' : 'Not triggered'; roofClosedEl.className = 'sensor-value ' + (data.roofClosed ? 'trig' : 'notrig'); const parkEl = document.getElementById('parksensor'); parkEl.textContent = data.safetySafe ? 'Scope parked' : 'Scope NOT parked'; parkEl.className = 'sensor-value ' + (data.safetySafe ? 'trig' : 'error'); document.getElementById('sopenbtn').disabled = !!data.roofOpen || !data.safetySafe; document.getElementById('sclosebtn').disabled = !!data.roofClosed || !data.safetySafe; document.getElementById('openbtn').disabled = !!data.roofOpen; document.getElementById('closebtn').disabled = !!data.roofClosed; }) .catch(() => { document.getElementById('domestatus').textContent = 'ERROR (Conn)'; }); } setInterval(updateStatus, 1000); window.addEventListener('load', updateStatus); </script></body></html>)rawliteral";
 
const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><body><form action="/setup/save" method="POST" style="background:#222;color:#eee;padding:20px;font-family:sans-serif">
<h1>Setup</h1><label>TCP</label><input name="tcp" value="%TCP%"><br><br>
<button>Save & Reboot</button></form></body></html>
)rawliteral";
 
// ========= SENSOR HELPERS =========
static inline bool sensorTriggered(int pin) {
  return digitalRead(pin) == SENSOR_ACTIVE_LEVEL;
}

// Logical roof position. Abstracts the physical sensor layout so the rest of
// the file treats both sensor modes identically.
bool isRoofAtClosed() {
#if SENSOR_TYPE == SENSOR_IR_BEAM
  return sensorTriggered(ROOF_CLOSED_PIN) && sensorTriggered(ROOF_OPEN_PIN);
#else
  return sensorTriggered(ROOF_CLOSED_PIN);
#endif
}

bool isRoofAtOpen() {
#if SENSOR_TYPE == SENSOR_IR_BEAM
  return !sensorTriggered(ROOF_CLOSED_PIN) && !sensorTriggered(ROOF_OPEN_PIN);
#else
  return sensorTriggered(ROOF_OPEN_PIN);
#endif
}

// ========= UTILS =========
const char* domeStatusStr() {
  if (roofLost) return "ERROR";
  if (isRoofAtOpen())   return "OPEN";
  if (isRoofAtClosed()) return "CLOSED";
  if (isSlewing) return (currentShutterState == SHUTTER_OPENING) ? "OPENING" : "CLOSING";
  return "CLOSED";
}

bool isSystemSafe() { return (digitalRead(PARK_SAFE_PIN) == LOW); }
 
void triggerRelay() {
  if (!relayActive) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive   = true;
    relayOffTime  = millis() + 1000;
  }
}
 
// ========= STATE LOGIC =========
void roofOpen(bool enforceSafety) {
  if (enforceSafety && !isSystemSafe()) return;
  if (isRoofAtOpen()) return;
  triggerRelay();
  currentShutterState = SHUTTER_OPENING;
  isSlewing = true; lastCommandTime = millis();
}

void roofClose(bool enforceSafety) {
  if (enforceSafety && !isSystemSafe()) return;
  if (isRoofAtClosed()) return;
  triggerRelay();
  currentShutterState = SHUTTER_CLOSING;
  isSlewing = true; lastCommandTime = millis();
}

void updateRoofState() {
  if (relayActive && millis() > relayOffTime) { digitalWrite(RELAY_PIN, LOW); relayActive = false; }
  bool ro = isRoofAtOpen();
  bool rc = isRoofAtClosed();
  if (isSlewing) {
    if (currentShutterState == SHUTTER_OPENING && ro) { currentShutterState=SHUTTER_OPEN; isSlewing=false; roofLost=false; }
    if (currentShutterState == SHUTTER_CLOSING && rc) { currentShutterState=SHUTTER_CLOSED; isSlewing=false; roofLost=false; }
  }
  if (!ro && !rc && isSlewing && (millis() - lastCommandTime > 60000UL)) {
    roofLost = true; currentShutterState = SHUTTER_ERROR; isSlewing = false;
  }
}
 
// ========= ID Helper =========
uint32_t getClientID(AsyncWebServerRequest *r) {
  if(r->hasArg("ClientTransactionID")) return r->arg("ClientTransactionID").toInt();
  if(r->hasArg("ClientID")) return r->arg("ClientID").toInt();
  return 0;
}
 
// ========= TCP Optimization Helper =========
void sendFastResponse(AsyncWebServerRequest *r, const char* jsonStr) {
    AsyncWebServerResponse *response = r->beginResponse(200, "application/json", jsonStr);
    //response->addHeader("Connection", "keep-alive"); // FORCE KEEP ALIVE
    r->send(response);
}
 
// ========= HANDLERS =========
void registerHandlers() {
 
  // 1. WEB UI
  server->on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", INDEX_HTML); });
 
  // 2. INTERNAL STATUS API (Web UI Polling)
  server->on("/status", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncJsonResponse *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();
    root["domeStatus"] = domeStatusStr();
    root["safetyStatus"] = isSystemSafe() ? "SAFE" : "UNSAFE";
    root["safetySafe"] = isSystemSafe();
    root["roofOpen"] = isRoofAtOpen();
    root["roofClosed"] = isRoofAtClosed();
    resp->setLength(); r->send(resp);
  });
 
  // 3. UI COMMANDS
  server->on("/sopen", HTTP_POST, [](AsyncWebServerRequest *r){ roofOpen(true); r->send(200); });
  server->on("/sclose", HTTP_POST, [](AsyncWebServerRequest *r){ roofClose(true); r->send(200); });
  server->on("/open", HTTP_POST, [](AsyncWebServerRequest *r){ roofOpen(false); r->send(200); });
  server->on("/close", HTTP_POST, [](AsyncWebServerRequest *r){ roofClose(false); r->send(200); });
 
  // 4. SETUP
  server->on("/setup", HTTP_GET, [](AsyncWebServerRequest *r){
    String h = FPSTR(SETUP_HTML);
    h.replace("%TCP%", String(currentAlpacaPort));
    r->send(200, "text/html", h);
  });
  server->on("/setup/save", HTTP_POST, [](AsyncWebServerRequest *r){
    if(r->hasArg("tcp")) {
      preferences.putInt("alpaca_tcp", r->arg("tcp").toInt());
      r->send(200, "text/plain", "Saved. Rebooting...");
      delay(1000); ESP.restart();
    }
  });
 
  // 5. ALPACA MANAGEMENT
  server->on("/management/apiversions", HTTP_GET, [](AsyncWebServerRequest *r){ 
    sendFastResponse(r, "{\"Value\":[1],\"ClientTransactionID\":0,\"ServerTransactionID\":0,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); 
  });
 
  server->on("/management/v1/configureddevices", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncJsonResponse *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();
    JsonArray arr = root.createNestedArray("Value");
    JsonObject d1 = arr.createNestedObject(); d1["DeviceName"]="ESP32 Dome"; d1["DeviceType"]="Dome"; d1["DeviceNumber"]=0; d1["UniqueID"]="ESP32-DOME";
    JsonObject d2 = arr.createNestedObject(); d2["DeviceName"]="ESP32 Safety"; d2["DeviceType"]="SafetyMonitor"; d2["DeviceNumber"]=0; d2["UniqueID"]="ESP32-SAFE";
    root["ClientTransactionID"] = getClientID(r);
    root["ServerTransactionID"] = serverTransactionID++;
    root["ErrorNumber"]=0; 
    root["ErrorMessage"]=""; 
    resp->setLength(); r->send(resp);
  });
 
  server->on("/management/v1/description", HTTP_GET, [](AsyncWebServerRequest *r){
      char buf[192];
      snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Observatory\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
      sendFastResponse(r, buf);
  });
 
  // 6. ALPACA DOME (Device 0)
 
  server->on("/api/v1/dome/0/connected", HTTP_GET, [](AsyncWebServerRequest *r){ 
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", 
             domeConnected ? "true" : "false", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/connected", HTTP_PUT, [](AsyncWebServerRequest *r){ 
    if(r->hasArg("Connected")) domeConnected = (r->arg("Connected") == "true" || r->arg("Connected") == "True");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/name", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Dome\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/description", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"Roll-off Roof\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Async Driver\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/driverversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"1.0\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":2,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/shutterstatus", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%d,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", (int)currentShutterState, getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/slewing", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", isSlewing?"true":"false", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/altitude", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":0.0,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/azimuth", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":0.0,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/athome", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", currentShutterState==SHUTTER_CLOSED?"true":"false", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/atpark", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", currentShutterState==SHUTTER_CLOSED?"true":"false", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/cansetshutter", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":true,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/slaved", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/slaved", HTTP_PUT, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Cannot Slave\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansetaltitude", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansetazimuth", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansetpark", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/canfindhome", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/canpark", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/canslave", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/dome/0/cansyncazimuth", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":false,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/openshutter", HTTP_PUT, [](AsyncWebServerRequest *r){
    char buf[128];
    if(!isSystemSafe()){ 
       snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1025,\"ErrorMessage\":\"Unsafe\"}", getClientID(r), serverTransactionID++);
       sendFastResponse(r, buf); return; 
    }
    if(currentShutterState == SHUTTER_CLOSED) { 
        roofOpen(true); 
        snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
    } else {
        snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Not Closed\"}", getClientID(r), serverTransactionID++);
    }
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/closeshutter", HTTP_PUT, [](AsyncWebServerRequest *r){
    char buf[128];
    if(!isSystemSafe()){ 
       snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1025,\"ErrorMessage\":\"Unsafe\"}", getClientID(r), serverTransactionID++);
       sendFastResponse(r, buf); return; 
    }
    if(currentShutterState == SHUTTER_OPEN) { 
        roofClose(true); 
        snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
    } else {
        snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Not Open\"}", getClientID(r), serverTransactionID++);
    }
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/dome/0/abortslew", HTTP_PUT, [](AsyncWebServerRequest *r){
      triggerRelay(); isSlewing=false; 
      char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
      sendFastResponse(r, buf);
  });
 
  auto notImpl = [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Not Implemented\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  };
  server->on("/api/v1/dome/0/findhome", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/park", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/setpark", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/slewtoaltitude", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/slewtoazimuth", HTTP_PUT, notImpl);
  server->on("/api/v1/dome/0/synctoazimuth", HTTP_PUT, notImpl);
 
  server->on("/api/v1/dome/0/supportedactions", HTTP_GET, [](AsyncWebServerRequest *r){ 
      char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":[],\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
      sendFastResponse(r, buf);
  });
 
  // 7. ALPACA SAFETY (Device 0)
 
  server->on("/api/v1/safetymonitor/0/connected", HTTP_GET, [](AsyncWebServerRequest *r){ 
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", 
             safetyConnected ? "true" : "false", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/connected", HTTP_PUT, [](AsyncWebServerRequest *r){ 
    if(r->hasArg("Connected")) safetyConnected = (r->arg("Connected") == "true" || r->arg("Connected") == "True");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/safetymonitor/0/issafe", HTTP_GET, [](AsyncWebServerRequest *r){ 
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", 
             isSystemSafe() ? "true" : "false", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
 
  server->on("/api/v1/safetymonitor/0/name", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Safety\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/description", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Safety Monitor\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"ESP32 Async Driver\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/driverversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":\"1.0\",\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
  server->on("/api/v1/safetymonitor/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest *r){
     char buf[128]; snprintf(buf, sizeof(buf), "{\"Value\":1,\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}", getClientID(r), serverTransactionID++);
     sendFastResponse(r, buf);
  });
 
  server->onNotFound([](AsyncWebServerRequest *r){ 
    char buf[128]; snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%d,\"ServerTransactionID\":%d,\"ErrorNumber\":1024,\"ErrorMessage\":\"Not Implemented\"}", getClientID(r), serverTransactionID++);
    sendFastResponse(r, buf);
  });
}
 
// ========= SETUP & LOOP =========
 
void initWiFi() {
  WiFi.mode(WIFI_STA); 
  WiFi.setSleep(false); 
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
}
 
void handleWiFi() {
  unsigned long now = millis();
 
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiConnecting && (now - lastWifiCheck > WIFI_RETRY_INTERVAL)) {
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      lastWifiCheck = now;
      wifiConnecting = true;
    }
  } else {
    if (wifiConnecting) {
      Serial.println("\nWiFi Connected!");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      wifiConnecting = false;
    }
  }
}
 
void setup() {
  setCpuFrequencyMhz(240); 
  Serial.begin(115200);
 
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(ROOF_OPEN_PIN, INPUT_PULLUP);
  pinMode(ROOF_CLOSED_PIN, INPUT_PULLUP);
  pinMode(PARK_SAFE_PIN, INPUT_PULLUP);
 
  preferences.begin("alpaca", false);
  currentAlpacaPort = preferences.getInt("alpaca_tcp", DEFAULT_ALPACA_PORT);
 
  // Initial Connect
  initWiFi();
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 5000) { 
    delay(200); Serial.print("."); 
  }
  Serial.println("");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
 
  server = new AsyncWebServer(currentAlpacaPort);
  registerHandlers();
  server->begin();
}
 
void loop() {
  updateRoofState();
  handleWiFi(); 
 
  // static unsigned long lastDiag = 0;
  // unsigned long now = millis();
  // if (now - lastDiag > 30000) {
  //   lastDiag = now;
  //   Serial.printf("[Diag] WiFi: %d, RSSI: %d, Free heap: %u, IP: %s\n",
  //    WiFi.status(), WiFi.RSSI(), ESP.getFreeHeap(), WiFi.localIP().toString().c_str());
  // }
}
 
