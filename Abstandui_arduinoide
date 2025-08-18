// -------------------- Bibliotheken --------------------
#include <MD_Parola.h>  //Virtuelle Displays und direkt Buchstaben
#include <MD_MAX72xx.h> // Grundsatz für Steuerung von Matrizen
#include <SPI.h>
#include <WiFi.h>       //Für Hotspot für WebUI
#include <WebServer.h>  //Webui
#include <Preferences.h>//WLAN konfiguration behalten
#include <HardwareSerial.h>
#include "Font_Data.h"  //Custom double font
#include <math.h>

// -------------------- Anzeige-Konfiguration --------------------
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define DEVICES       32
#define DATA_PIN      4
#define CLK_PIN       6
#define CS_PIN        5
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, DEVICES);

// -------------------- Parameter --------------------
const float Reibung     = 0.7f;       // Reibungskoeffizient (trocken)
const float Reaktionszeit   = 1.0f;   // Reaktionszeit Hintermann (s)
const float g      = 9.81f;           // Erdbeschleunigung (m/s²)

// -------------------- Variablen für Sicherheitsabstandsberechnungen --------------------
uint8_t currentMsgIndex = 0;
unsigned long lastRotationChange = 0; //Zeit seit anderem Text auf matrix
float s_safe = 0.0f;        //Sicherer Abstand (Faustregel)
float s_safe_phys = 0.0f;   //Sicherer Abstand (Physikalisch, idealbedingungen)

// -------------------- Text-Puffer & Status --------------------
char textBuffer[100];
uint16_t abstandCm            = 0;
bool     SystemActive         = true;
char     customMsg[50]        = "";
char     warningMsg[50]       = "";
unsigned long msgTimestamp    = 0;

// -------------------- Netzwerk & Sensoren --------------------
float    speedKmh         = 0.0f;
int      satellitenAnzahl = 0;
bool     newDistanceAvailable = false;

WebServer   server(80);       //Webserver auf port 80 (http)
Preferences prefs;
HardwareSerial SerialGPS(1); 
HardwareSerial SerialTF02(2);

// -------------------- Debug Terminal --------------------
String debugLog = "";
const size_t maxDebugLogSize = 2048; // Maximale Größe des Debug-Logs

// -------------------- Forward Declarations (Functions) --------------------
void updateDisplay(const char* newText);
void initTF02();
void tf02Verarbeiten();
void initGPS();
void gpsVerarbeiten();
void verarbeiteRMC(String nmea);
void verarbeiteGGA(String nmea);

void handleRoot();
void handleDaten();
void handleToggle();
void handleWlanForm();
void handleWlanSave();
void handleSendMsg();
void handleDebug();           // Neuer Handler für Debug-Terminal
void handleDebugData();       // Neuer Handler für Debug-Datenabfrage
void debugPrint(String msg);  // Funktion zum Hinzufügen von Debug-Nachrichten

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  debugPrint("System startet...");

  initGPS();
  initTF02();

  // WLAN AP & Webserver aufstarten
  prefs.begin("gpscfg", false);
  String ssid = prefs.getString("ssid", "ESP32-WLAN");
  String pass = prefs.getString("pass", "12345678");
  WiFi.softAP(ssid.c_str(), pass.c_str());
  debugPrint("AP gestartet: " + ssid);

  server.on("/",         handleRoot);
  server.on("/daten",    handleDaten);
  server.on("/toggle",   handleToggle);
  server.on("/wlan",     handleWlanForm);
  server.on("/wlan_save",handleWlanSave);
  server.on("/sendmsg",  HTTP_POST, handleSendMsg);
  server.on("/debug",    handleDebug);        // Debug-Terminal Seite
  server.on("/debugdata",handleDebugData);    // Debug-Datenabfrage
  server.begin();

  // Display initialisiern (inkl. virtuelle Displays)
  display.begin(2);
  debugPrint("Display initialisiert");
  
  display.setIntensity(5);
  display.setZone(0, 0, 15);
  display.setZone(1, 16, 31);
  debugPrint("Zonen konfiguriert");
  
  display.setFont(0, BigFontLower);
  display.setFont(1, BigFontUpper);
  debugPrint("Schriftarten gesetzt");
  
  updateDisplay("Starting...");
  
  debugPrint("System online");
}

// -------------------- Loop --------------------
void loop() {
  // Sensor-Daten
  gpsVerarbeiten();
  tf02Verarbeiten();

  // Webserver
  server.handleClient();

  // Anzeige-Logik
  unsigned long now = millis();

  // Custom-Message nach 10 Sekunden löschen
  if (customMsg[0] != '\0' && now - msgTimestamp >= 10000) {
    debugPrint("Custom-Message gelöscht");
    customMsg[0] = '\0';  // Nachricht löschen
  }

  // 1) Custom-Message mit Priorität
  if (customMsg[0] != '\0') {
    updateDisplay(customMsg);
  }
  // 2) Gefahrencheck (System wird nur aktiviert wenn: SystemActive = true, mehr als 4 GPS-Sateliten verbunden und das Auto schneller als 15 km/h unterwegs ist.)
  else if (SystemActive && satellitenAnzahl >= 4 && speedKmh >= 15) {
    float abstandM = abstandCm / 100.0f;
    float speedMs = speedKmh / 3.6f;
    s_safe_phys = (speedMs * Reaktionszeit) + (speedMs * speedMs) / (2 * Reibung * g);
    s_safe = fmaxf(s_safe_phys, 0.5f * speedKmh);

    if (abstandM < s_safe && abstandM < s_safe_phys) {
      // Nachrichtenrotation alle 2 Sekunden
      if (lastRotationChange == 0 || now - lastRotationChange >= 2000) {
        switch (currentMsgIndex) {
          case 0: strcpy(warningMsg, "!!ACHTUNG!!"); break;
          case 1: strcpy(warningMsg, "Jetzt:"); break;
          case 2: snprintf(warningMsg, sizeof(warningMsg), "%.1fm", abstandM); break;
          case 3: strcpy(warningMsg, "Empfohlen:"); break;
          case 4: snprintf(warningMsg, sizeof(warningMsg), "%.1fm", s_safe); break;
          case 5: strcpy(warningMsg, "Mindestens:"); break;
          case 6: snprintf(warningMsg, sizeof(warningMsg), "%.1fm", s_safe_phys); break;
        }
        
        currentMsgIndex = (currentMsgIndex + 1) % 7;
        lastRotationChange = now;
        debugPrint("Warnmeldung rotiert: " + String(warningMsg));
      }
      updateDisplay(warningMsg);
    } else {
      // Keine Gefahr -> leere Anzeige
      updateDisplay("");
      // Warnmeldung zurücksetzen
      warningMsg[0] = '\0';
      currentMsgIndex = 0;
      lastRotationChange = 0;
    }
  }
  // 3) Leere Anzeige (wenn System inaktiv oder zu wenige Satelliten)
  else {
    updateDisplay("");
    // Warnmeldung zurücksetzen
    warningMsg[0] = '\0';
    currentMsgIndex = 0;
    lastRotationChange = 0;
  }

  display.displayAnimate();
}

// -------------------- Hilfsfunktionen --------------------
void updateDisplay(const char* newText) {
  // Nur wenn sich der Text ändert, aktualisieren
  if (strncmp(textBuffer, newText, sizeof(textBuffer)-1) != 0) {
    strncpy(textBuffer, newText, sizeof(textBuffer)-1);
    textBuffer[sizeof(textBuffer)-1] = '\0';
    
    display.displayClear();
    display.displayZoneText(0, textBuffer, PA_CENTER, 50, 1000, PA_PRINT, PA_NO_EFFECT);    //2 mal schreiben für beide virtuellen Diasplays
    display.displayZoneText(1, textBuffer, PA_CENTER, 50, 1000, PA_PRINT, PA_NO_EFFECT);
    display.displayReset();
    
    debugPrint("Display aktualisiert: " + String(newText));
  }
}


void initTF02() {
  SerialTF02.begin(115200, SERIAL_8N1, 17, 18);
  debugPrint("LIDAR AKTIV");
}

void tf02Verarbeiten() {
  static uint8_t buf[9], idx = 0;
  while (SerialTF02.available()) {
    uint8_t b = SerialTF02.read();
    if (idx==0 && b!=0x59) continue;
    if (idx==1 && b!=0x59) { idx=0; continue; }
    buf[idx++] = b;
    if (idx==9) {
      uint8_t sum=0;
      for(int i=0;i<8;i++) sum += buf[i];
      if (sum==buf[8]) {
        uint16_t newDistance = buf[2] | (buf[3]<<8);
        if (newDistance != abstandCm) {
          abstandCm = newDistance;
          debugPrint("LIDAR Abstand: " + String(abstandCm) + " cm");
        }
      }
      idx=0;
    }
  }
  if (abstandCm > 4000) {
    abstandCm = 999999;       //Kann laut Herstellerspezifikationen 40m erkennen, dannach, um falschalarme auszulösen auf rediculous hoch setzen
  }
}

void initGPS() {
  SerialGPS.begin(9600, SERIAL_8N1, 16, -1);
  debugPrint("GPS-Modul AKTIV");
}

void gpsVerarbeiten() {       //Manuelle Verarbeitung der GPS-Sätze
  static String line="";
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    if (c=='\n') {
      line.trim();
      if (line.startsWith("$GNRMC")) verarbeiteRMC(line);
      else if (line.startsWith("$GPGGA")||line.startsWith("$GNGGA")) verarbeiteGGA(line);
      line="";
    } else line += c;
  }
}

void verarbeiteRMC(String nmea) {
  int k[10], z=0;
  for (int i=0;i<nmea.length();i++) if (nmea[i]==',') k[z++]=i;
  if (z>=7) {
    String s = nmea.substring(k[6]+1,k[7]);
    float newSpeed = s.toFloat() * 1.852f;
    if (fabs(newSpeed - speedKmh) > 0.5f) {
      speedKmh = newSpeed;
      debugPrint("Geschwindigkeit: " + String(speedKmh, 1) + " km/h");
    }
  }
}

void verarbeiteGGA(String nmea) {
  int k[10], z=0;
  for (int i=0;i<nmea.length();i++) if (nmea[i]==',') k[z++]=i;
  if (z>=7) {
    String s = nmea.substring(k[6]+1,k[7]);
    int newSats = s.toInt();
    if (newSats != satellitenAnzahl) {
      satellitenAnzahl = newSats;
      debugPrint("Satelliten: " + String(satellitenAnzahl));
    }
  }
}

// -------------------- Debug Funktionen --------------------
void debugPrint(String msg) {
  unsigned long now = millis();
  String timestamp = "[" + String(now/1000) + "." + String(now%1000) + "] ";
  debugLog += timestamp + msg + "\n";
  
  // Log auf maximale Größe beschränken
  if (debugLog.length() > maxDebugLogSize) {
    debugLog = debugLog.substring(debugLog.length() - maxDebugLogSize);
  }
  
  Serial.println(msg);
}

// -------------------- Web UI ------------------------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html><html lang="de"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AbstandUI</title>
<style>body{background:#121212;color:#eee;font-family:sans-serif;text-align:center}
.card{background:#1e1e1e;padding:20px;margin:20px auto;width:320px;border-radius:10px}
button,input{background:#333;color:white;border:none;padding:10px 20px;margin:5px;border-radius:5px;cursor:pointer}
input{width:200px}
.wlan-btn{background:#555}
.debug-btn{background:#007bff}
</style></head><body>
<h1>AbstandUI</h1>
<div style="font-size: 10px; color: #888; margin-top: -8px; margin-bottom: 5px;">Version 1.1.0</div>
<div class="card">
  <p><b>Geschw.:</b> <span id="speed">...</span> km/h</p>
  <p><b>Satelliten:</b> <span id="sats">...</span></p>
  <p><b>Abstand:</b> <span id="abstand">...</span> cm</p>
  <p><b>System:</b> <span id="status" class="status off">...</span></p>
</div>
<button onclick="toggleSystem()">System Ein/Aus</button>
<button class="wlan-btn" onclick="location.href='/wlan'">WLAN ändern</button>
<button class="debug-btn" onclick="location.href='/debug'">Debug Terminal</button>
<form id="msgForm" onsubmit="sendMsg(event)">
  <p>Custom Message:<br><input id="msg" name="msg" required></p>
  <input type="submit" value="Senden">
</form>
<script>
async function ladeDaten(){
  const r = await fetch('/daten'); const d = await r.json();
  document.getElementById('speed').innerText   = d.speed.toFixed(1);
  document.getElementById('sats').innerText    = d.sats;
  document.getElementById('abstand').innerText = d.abstand;
  const st = document.getElementById('status');
  st.innerText = d.active? 'Ein':'Aus';
}
function toggleSystem(){ fetch('/toggle').then(ladeDaten) }
async function sendMsg(e){
  e.preventDefault();
  const msg = document.getElementById('msg').value;
  await fetch('/sendmsg',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'msg='+encodeURIComponent(msg)});
  document.getElementById('msgForm').reset();
}
setInterval(ladeDaten,500);
window.onload=ladeDaten;
</script></body></html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleDaten() {
  String json = "{";
  json += "\"speed\":" + String(speedKmh,1) + ",";
  json += "\"sats\":"  + String(satellitenAnzahl) + ",";
  json += "\"abstand\":" + String(abstandCm) + ",";
  json += "\"active\":" + String(SystemActive?1:0);
  json += "}";
  server.send(200, "application/json", json);
}

void handleToggle() {
  SystemActive = !SystemActive;
  debugPrint("Systemstatus geändert: " + String(SystemActive ? "EIN" : "AUS"));
  server.sendHeader("Location","/");
  server.send(302,"text/plain","");
}

void handleWlanForm() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1.0">
  <title>WLAN ändern</title>
  <style>
    body {
      background: #121212;
      color: #eee;
      font-family: sans-serif;
      text-align: center;
      margin: 0;
      padding: 20px;
    }
    .card {
      background: #1e1e1e;
      padding: 20px;
      margin: 20px auto;
      width: 320px;
      border-radius: 10px;
    }
    input, button {
      background: #333;
      color: white;
      border: none;
      padding: 10px 20px;
      margin: 5px 0;
      border-radius: 5px;
      font-size: 1em;
      width: 100%;
      box-sizing: border-box;
      cursor: pointer;
    }
    button.wlan-btn {
      background: #555;
    }
    a.button-link {
      display: inline-block;
      text-decoration: none;
      width: auto;
    }
  </style>
</head>
<body>
  <h1>GPS &amp; LiDAR Tracker</h1>
  <div class="card">
    <h2>WLAN Einstellungen</h2>
    <form action="/wlan_save" method="POST">
      <p>
        <label for="ssid"><b>SSID:</b></label><br>
        <input id="ssid" name="ssid" placeholder="Dein WLAN-Name" required>
      </p>
      <p>
        <label for="pass"><b>Passwort:</b></label><br>
        <input id="pass" name="pass" type="password" placeholder="Passwort" required>
      </p>
      <input type="submit" value="Speichern & Neustarten">
    </form>
    <a href="/" class="button-link">
      <button type="button">Zurück</button>
    </a>
  </div>
</body>
</html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleWlanSave() {         //Ermöglicht das Ändern von SSID und Passwort
  if (server.hasArg("ssid") && server.hasArg("pass")) {
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("pass", server.arg("pass"));
    debugPrint("WLAN Einstellungen gespeichert: " + server.arg("ssid"));
    server.send(200, "text/plain; charset=utf-8", "Gespeichert! Startet neu...");
    delay(1500); ESP.restart();
  } else {
    server.send(400, "text/plain", "Fehler: Felder fehlen");
  }
}

void handleSendMsg() {      //Custom-Message-Logik
  if (server.hasArg("msg")) {
    String m = server.arg("msg");
    m.toCharArray(customMsg, sizeof(customMsg));
    msgTimestamp = millis();
    debugPrint("Custom-Message gesetzt: " + m);
    server.send(200, "text/plain", "Nachricht gesetzt");
  } else {
    server.send(400, "text/plain", "Fehler: msg fehlt");
  }
}

// -------------------- Debug Terminal Handlers --------------------
void handleDebug() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1.0">
  <title>Debug Terminal</title>
  <style>
    body {
      background: #121212;
      color: #0f0;
      font-family: monospace;
      margin: 0;
      padding: 10px;
    }
    #terminal {
      background: #000;
      padding: 15px;
      border-radius: 5px;
      height: 70vh;
      overflow-y: auto;
      white-space: pre-wrap;
      font-size: 14px;
    }
    .controls {
      margin: 10px 0;
    }
    button {
      background: #333;
      color: white;
      border: none;
      padding: 8px 15px;
      border-radius: 3px;
      cursor: pointer;
      margin-right: 5px;
    }
  </style>
</head>
<body>
  <h1>Debug Terminal</h1>
  <div class="controls">
    <button onclick="refreshLog()">Aktualisieren</button>
    <button onclick="clearLog()">Log löschen</button>
    <button onclick="location.href='/'">Zurück</button>
  </div>
  <div id="terminal">Lade Debug-Daten...</div>
  
  <script>
    const terminal = document.getElementById('terminal');
    
    async function loadDebugData() {
      try {
        const response = await fetch('/debugdata');
        const data = await response.text();
        terminal.textContent = data;
        terminal.scrollTop = terminal.scrollHeight;
      } catch (error) {
        terminal.textContent = "Fehler beim Laden der Debug-Daten: " + error;
      }
    }
    
    function refreshLog() {
      loadDebugData();
    }
    
    async function clearLog() {
      if (confirm("Debug-Log wirklich löschen?")) {
        await fetch('/debugdata?clear=1');
        refreshLog();
      }
    }
    
    // Automatische Aktualisierung alle 2 Sekunden
    setInterval(refreshLog, 2000);
    window.onload = loadDebugData;
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleDebugData() {
  // Log löschen wenn Parameter gesetzt
  if (server.hasArg("clear")) {
    debugLog = "";
    server.send(200, "text/plain", "Debug-Log gelöscht\n");
    return;
  }
  
  server.send(200, "text/plain", debugLog);
}
