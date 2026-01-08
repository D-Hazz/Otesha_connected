/*
  serre_controller.ino
  ESP32 controller (DHT22, soil sensor, LDR), Fan/AirPump/SoilPump/Lamp/Heater,
  WiFiManager provisioning + SoftAP fixed IP, SPIFFS persistence for config,
  APIs: /data, /api/update, /api/actuators, /api/history
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <DNSServer.h>
#include <WiFiManager.h>
#include <time.h>

#define DHTPIN 2
#define DHTTYPE DHT22

#define FAN_PIN 12
#define PUMP_AIR 13
#define PUMP_SOIL 14
#define LAMP_PIN 27
#define HEATER_PIN 26

#define SOIL_PIN 34
#define LDR_PIN 35
#define BUTTON_PIN 23

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);
WiFiUDP udp;
IPAddress remoteIP(0,0,0,0);
const uint16_t remotePort = 8888;

// Actuator control types
enum Mode { AUTO_MODE, FORCE_ON, FORCE_OFF };
struct ActuatorCtrl { Mode mode = AUTO_MODE; unsigned long pulseUntilMs = 0; };
ActuatorCtrl fanCtrl, airCtrl, soilCtrl, lampCtrl, heaterCtrl;

bool fanState=false, pumpAirState=false, pumpSoilState=false, lampState=false, heaterState=false;

// enabled global (persisted)
bool enabled = true;

// Thresholds structure
struct Thresholds {
  float thTempMin = 0.0;
  float thTempMax = 100.0;
  float thHumMin  = 0.0;
  float thHumMax  = 100.0;
  int   thSoilMin = 0;
  int   thSoilMax = 100;
  int   thLightMin= 0;
  int   thLightMax= 100;
} thresholds;

// History
#define HISTORY_SIZE 2880
struct DataPoint { float t; float h; int soil; int light; unsigned long ts; };
DataPoint history[HISTORY_SIZE];
int historyCount = 0;

// Prototypes
void saveConfig();
void loadConfig();
void setCors();
void handleData();
void handleHistory();
void handleApiUpdate();
void handleApiActuators();
void handleNotFound();
void sendJson(WebServer &srv, int code, const String &json);
void applyMode(ActuatorCtrl &c, const String &m);

void setup(){
  Serial.begin(115200);
  pinMode(FAN_PIN, OUTPUT); pinMode(PUMP_AIR, OUTPUT); pinMode(PUMP_SOIL, OUTPUT);
  pinMode(LAMP_PIN, OUTPUT); pinMode(HEATER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  dht.begin();

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS mount failed - formatting...");
    SPIFFS.format();
    SPIFFS.begin(true);
  }
  loadConfig();

  // WiFiManager (provisioning)
  WiFiManager wm;
  wm.setHostname("ESP32-Serre");
  bool res = wm.autoConnect("ESP32-Serre-Setup", "password");
  if(res) Serial.print("Connected STA IP: "), Serial.println(WiFi.localIP());
  else Serial.println("WiFiManager: no STA connected (AP available).");

  // Ensure SoftAP fixed IP 192.168.4.1
  IPAddress apIP(192,168,4,1); IPAddress gw(192,168,4,1); IPAddress sn(255,255,255,0);
  WiFi.softAPConfig(apIP, gw, sn);
  WiFi.softAP("ESP32-Serre-AP");

  udp.begin(8888);

  // NTP
  configTime(0,0,"pool.ntp.org","time.nist.gov");

  // HTTP routes
  server.on("/", [](){ File f = SPIFFS.open("/index.html","r"); if(!f){ server.send(500,"text/plain","index not found"); return; } server.streamFile(f,"text/html"); f.close(); });
  server.on("/data", HTTP_GET, handleData);
  server.on("/api/update", HTTP_OPTIONS, [](){ setCors(); server.send(200); });
  server.on("/api/update", HTTP_POST, handleApiUpdate);
  server.on("/api/actuators", HTTP_OPTIONS, [](){ setCors(); server.send(200); });
  server.on("/api/actuators", HTTP_POST, handleApiActuators);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started. AP IP: 192.168.4.1");
}

void loop(){
  static unsigned long lastMeasure = 0;
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounce = 0;
  server.handleClient();

  // physical button handling (debounce) -> toggle enabled
  int reading = digitalRead(BUTTON_PIN);
  if(reading != lastButtonState) lastDebounce = millis();
  if((millis() - lastDebounce) > 50){
    if(reading == LOW && lastButtonState == HIGH){
      enabled = !enabled;
      Serial.printf("Button toggled enabled -> %s\n", enabled ? "ON":"OFF");
      saveConfig();
    }
  }
  lastButtonState = reading;

  unsigned long now = millis();
  if(now - lastMeasure >= 1000){
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    int soilRaw = analogRead(SOIL_PIN);
    int soilPercent = map(soilRaw, 4095, 0, 0, 100); soilPercent = constrain(soilPercent,0,100);
    int ldrRaw = analogRead(LDR_PIN);
    int lightPercent = map(ldrRaw, 0, 4095, 0, 100); lightPercent = constrain(lightPercent,0,100);

    if(isnan(h) || isnan(t)){
      Serial.println("DHT read error");
    } else {
      // store history
      if(historyCount < HISTORY_SIZE) history[historyCount++] = {t,h,soilPercent,lightPercent,millis()};
      else {
        for(int i=1;i<HISTORY_SIZE;i++) history[i-1] = history[i];
        history[HISTORY_SIZE-1] = {t,h,soilPercent,lightPercent,millis()};
      }

      // Auto logic (only when enabled)
      bool tempHeaterAuto = enabled && (t < thresholds.thTempMin);
      bool tempFanAuto    = enabled && (t > thresholds.thTempMax);
      bool humAirAutoPump = enabled && (h < thresholds.thHumMin);
      bool humAirAutoFan  = enabled && (h > thresholds.thHumMax);
      bool soilAuto       = enabled && (soilPercent < thresholds.thSoilMin);
      bool lampAuto       = enabled && (lightPercent < thresholds.thLightMin);

      unsigned long curMs = millis();
      auto applyCtrl = [&](ActuatorCtrl &ctrl, bool autoState){
        if(ctrl.pulseUntilMs && curMs < ctrl.pulseUntilMs) return true;
        if(ctrl.pulseUntilMs && curMs >= ctrl.pulseUntilMs) ctrl.pulseUntilMs = 0;
        if(ctrl.mode == FORCE_ON) return true;
        if(ctrl.mode == FORCE_OFF) return false;
        return autoState;
      };

      heaterState  = applyCtrl(heaterCtrl, tempHeaterAuto);
      fanState     = applyCtrl(fanCtrl, (tempFanAuto || humAirAutoFan));
      pumpAirState = applyCtrl(airCtrl, humAirAutoPump);
      pumpSoilState= applyCtrl(soilCtrl, soilAuto);
      lampState    = applyCtrl(lampCtrl, lampAuto);

      digitalWrite(HEATER_PIN, heaterState ? HIGH : LOW);
      digitalWrite(FAN_PIN, fanState ? HIGH : LOW);
      digitalWrite(PUMP_AIR, pumpAirState ? HIGH : LOW);
      digitalWrite(PUMP_SOIL, pumpSoilState ? HIGH : LOW);
      digitalWrite(LAMP_PIN, lampState ? HIGH : LOW);

      // Broadcast UDP (optional)
      char buf[256];
      snprintf(buf, sizeof(buf), "{\"t\":%.1f,\"h\":%.1f,\"soil\":%d,\"light\":%d,\"enabled\":%s,\"heater\":%s,\"fan\":%s,\"air\":%s,\"soilPump\":%s,\"lamp\":%s}",
               t, h, soilPercent, lightPercent, enabled?"true":"false", heaterState?"true":"false", fanState?"true":"false", pumpAirState?"true":"false", pumpSoilState?"true":"false", lampState?"true":"false");
      if(remoteIP != IPAddress(0,0,0,0)){
        udp.beginPacket(remoteIP, remotePort); udp.write((uint8_t*)buf, strlen(buf)); udp.endPacket();
      } else {
        udp.beginPacket(IPAddress(255,255,255,255), remotePort); udp.write((uint8_t*)buf, strlen(buf)); udp.endPacket();
      }
    }
    lastMeasure = now;
  }

  delay(10);
}

// --------- HTTP / API helpers ----------
void setCors(){
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Access-Control-Allow-Methods","GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers","Content-Type");
}

void handleData(){
  setCors();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int soilRaw = analogRead(SOIL_PIN);
  int soilPercent = map(soilRaw, 4095, 0, 0, 100); soilPercent = constrain(soilPercent,0,100);
  int ldrRaw = analogRead(LDR_PIN);
  int lightPercent = map(ldrRaw, 0, 4095, 0, 100); lightPercent = constrain(lightPercent,0,100);

  StaticJsonDocument<512> doc;
  doc["t"] = isnan(t) ? 0.0 : t;
  doc["h"] = isnan(h) ? 0.0 : h;
  doc["soil"] = soilPercent;
  doc["light"] = lightPercent;
  doc["enabled"] = enabled;
  doc["heater"] = heaterState;
  doc["fan"] = fanState;
  doc["air"] = pumpAirState;
  doc["soilPump"] = pumpSoilState;
  doc["lamp"] = lampState;
  doc["thTempMin"] = thresholds.thTempMin;
  doc["thTempMax"] = thresholds.thTempMax;
  doc["thHumMin"]  = thresholds.thHumMin;
  doc["thHumMax"]  = thresholds.thHumMax;
  doc["thSoilMin"] = thresholds.thSoilMin;
  doc["thSoilMax"] = thresholds.thSoilMax;
  doc["thLightMin"]= thresholds.thLightMin;
  doc["thLightMax"]= thresholds.thLightMax;

  if(WiFi.status() == WL_CONNECTED) doc["ip"] = WiFi.localIP().toString();
  else doc["ip"] = String("n/a");
  doc["apip"] = "192.168.4.1";
  doc["rssi"] = WiFi.RSSI();
  doc["uptime"] = String(millis()/1000) + "s";
  doc["heap"] = ESP.getFreeHeap();
  String out; serializeJson(doc,out);
  sendJson(server,200,out);
}

void handleHistory(){
  setCors();
  StaticJsonDocument<HISTORY_SIZE * 32> doc;
  JsonArray arr = doc.to<JsonArray>();
  for(int i=0;i<historyCount;i++){
    JsonObject p = arr.createNestedObject();
    p["t"] = history[i].t;
    p["h"] = history[i].h;
    p["soil"] = history[i].soil;
    p["light"] = history[i].light;
    p["ts"] = history[i].ts;
  }
  String out; serializeJson(doc,out);
  sendJson(server,200,out);
}

void handleApiUpdate(){
  setCors();
  if(!server.hasArg("plain")){ server.send(400,"text/plain","Bad Request"); return; }
  String body = server.arg("plain");
  StaticJsonDocument<512> doc;
  auto err = deserializeJson(doc, body);
  if(err){ server.send(400,"text/plain","Invalid JSON"); return; }

  // toggle enabled
  if(doc.containsKey("enabled")){
    if(doc["enabled"].is<const char*>()){
      const char* v = doc["enabled"];
      if(String(v) == "toggle"){
        enabled = !enabled;
        saveConfig();
      }
    } else if(doc["enabled"].is<bool>()){
      enabled = doc["enabled"].as<bool>();
      saveConfig();
    }
  }

  // thresholds
  if(doc.containsKey("thresholds")){
    JsonObject th = doc["thresholds"].as<JsonObject>();
    thresholds.thTempMin = th["thTempMin"] | thresholds.thTempMin;
    thresholds.thTempMax = th["thTempMax"] | thresholds.thTempMax;
    thresholds.thHumMin  = th["thHumMin"]  | thresholds.thHumMin;
    thresholds.thHumMax  = th["thHumMax"]  | thresholds.thHumMax;
    thresholds.thSoilMin = th["thSoilMin"] | thresholds.thSoilMin;
    thresholds.thSoilMax = th["thSoilMax"] | thresholds.thSoilMax;
    thresholds.thLightMin= th["thLightMin"]| thresholds.thLightMin;
    thresholds.thLightMax= th["thLightMax"]| thresholds.thLightMax;
    saveConfig();
  }

  String out = "";
  StaticJsonDocument<128> res; res["ok"] = true; res["enabled"] = enabled;
  serializeJson(res, out);
  sendJson(server,200,out);
}

void applyMode(ActuatorCtrl &c, const String &m){
  if(m == "ON") c.mode = FORCE_ON;
  else if(m == "OFF") c.mode = FORCE_OFF;
  else c.mode = AUTO_MODE;
}

void handleApiActuators(){
  setCors();
  if(!server.hasArg("plain")){ server.send(400,"text/plain","Bad Request"); return; }
  String body = server.arg("plain");
  StaticJsonDocument<256> doc;
  auto err = deserializeJson(doc, body);
  if(err){ server.send(400,"text/plain","Invalid JSON"); return; }

  if(doc.containsKey("fan")) applyMode(fanCtrl, String((const char*)doc["fan"]));
  if(doc.containsKey("air")) applyMode(airCtrl, String((const char*)doc["air"]));
  if(doc.containsKey("soil")) applyMode(soilCtrl, String((const char*)doc["soil"]));
  if(doc.containsKey("lamp")) applyMode(lampCtrl, String((const char*)doc["lamp"]));
  if(doc.containsKey("heater")) applyMode(heaterCtrl, String((const char*)doc["heater"]));
  if(doc.containsKey("pulseMs")){
    unsigned long ms = doc["pulseMs"].as<unsigned long>();
    unsigned long until = millis() + ms;
    fanCtrl.pulseUntilMs = until; airCtrl.pulseUntilMs = until; soilCtrl.pulseUntilMs = until; lampCtrl.pulseUntilMs = until; heaterCtrl.pulseUntilMs = until;
  }
  String out; StaticJsonDocument<64> r; r["ok"]=true; serializeJson(r,out); sendJson(server,200,out);
}

void handleNotFound(){
  setCors();
  String path = server.uri(); if(path == "/") path = "/index.html";
  if(SPIFFS.exists(path)){
    File f = SPIFFS.open(path,"r"); if(f){ String ctype="text/plain"; if(path.endsWith(".html")) ctype="text/html"; else if(path.endsWith(".css")) ctype="text/css"; else if(path.endsWith(".js")) ctype="application/javascript"; server.streamFile(f,ctype); f.close(); return; }
  }
  server.send(404,"text/plain","Not found");
}

void sendJson(WebServer &srv, int code, const String &json){
  srv.send(code, "application/json", json);
}

// ---------- Persistence: config (enabled + thresholds) ----------
void saveConfig(){
  StaticJsonDocument<256> doc;
  doc["enabled"] = enabled;
  doc["thTempMin"] = thresholds.thTempMin; doc["thTempMax"] = thresholds.thTempMax;
  doc["thHumMin"] = thresholds.thHumMin; doc["thHumMax"] = thresholds.thHumMax;
  doc["thSoilMin"] = thresholds.thSoilMin; doc["thSoilMax"] = thresholds.thSoilMax;
  doc["thLightMin"] = thresholds.thLightMin; doc["thLightMax"] = thresholds.thLightMax;
  File f = SPIFFS.open("/config.json","w");
  if(!f){ Serial.println("Failed to open config.json for write"); return; }
  serializeJson(doc, f); f.close(); Serial.println("Config saved to SPIFFS.");
}

void loadConfig(){
  if(!SPIFFS.exists("/config.json")){ Serial.println("No config.json, use defaults."); return; }
  File f = SPIFFS.open("/config.json","r"); if(!f){ Serial.println("Failed to open config.json"); return; }
  StaticJsonDocument<256> doc; auto err = deserializeJson(doc, f); f.close();
  if(err){ Serial.println("Failed to parse config.json"); return; }
  enabled = doc["enabled"] | enabled;
  thresholds.thTempMin = doc["thTempMin"] | thresholds.thTempMin;
  thresholds.thTempMax = doc["thTempMax"] | thresholds.thTempMax;
  thresholds.thHumMin  = doc["thHumMin"]  | thresholds.thHumMin;
  thresholds.thHumMax  = doc["thHumMax"]  | thresholds.thHumMax;
  thresholds.thSoilMin = doc["thSoilMin"] | thresholds.thSoilMin;
  thresholds.thSoilMax = doc["thSoilMax"] | thresholds.thSoilMax;
  thresholds.thLightMin= doc["thLightMin"]| thresholds.thLightMin;
  thresholds.thLightMax= doc["thLightMax"]| thresholds.thLightMax;
  Serial.println("Config loaded from SPIFFS.");
}
