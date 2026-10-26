// Zeitnahme Start/Ziel per LoRa + WLAN-Webserver am Ziel  (v3)
// Board: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
// Bibliotheken: RadioLib, U8g2 (beide über den Bibliotheksverwalter)
//
// Neu in v3:
//  - Beide Displays (Start und Ziel) zeigen: Verbindung, BEREIT/GESTARTET, Laufnummer
//    und die letzten 3 Zeiten.
//  - Funkpaket enthält die letzten 3 Ergebnisse (PONG und RESULT), daher größer.
//  - Frequenz 869.525 MHz (EU, 10 % Duty-Cycle), weil der Heartbeat auf 868.1 MHz (1 %) zu viel sendet.
//
// Auf BEIDE Boards dieselbe Datei flashen. Die Rolle (START oder ZIEL) steht im Flash.
// PRG-Taste: 2 s halten und loslassen = Rolle wechseln (Neustart),
//            5 s halten und loslassen = Deep Sleep. Aufwecken: PRG kurz drücken oder RST.
// Zu Beginn sind beide Boards START: auf dem Ziel-Board die PRG-Taste 2 s halten.

#include <RadioLib.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "driver/gpio.h"

// ---- Pins Heltec V3 ----
#define LORA_SCK  9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_NSS  8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13
#define LED_PIN   35   // Onboard-LED
#define VEXT_PIN  36   // LOW = OLED-Versorgung an
#define START_PIN 5    // Mikroschalter gegen GND (INPUT_PULLUP)
#define ZIEL_PIN  5    // Lichtschranke/Schalter gegen GND (INPUT_PULLUP)
#define BUTTON_PIN 0   // PRG-Taste (= BOOT): nie beim Einschalten/Reset gedrückt halten!
#define HOLD_ROLE_MS  2000   // so lange halten = Rolle wechseln
#define HOLD_SLEEP_MS 5000   // so lange halten = Deep Sleep

// Funkfrequenz (EU): 869.525 MHz liegt im Band 869.4-869.65 MHz mit 10 % Duty-Cycle
#define LORA_FREQ 869.525

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, 21, 18, 17);   // RST, SCL, SDA

// ---- Funkpaket (feste Größe -> konstante Airtime) ----
#define MAGIC   0xA5
#define T_PING   1   // Start -> Ziel, Lebenszeichen
#define T_PONG   2   // Ziel -> Start (copy Bit0 = Ziel ist scharf, plus letzte Ergebnisse)
#define T_START  3   // Start -> Ziel, Lauf gestartet
#define T_ACK    4   // Ziel -> Start, Start angekommen
#define T_RESULT 5   // Ziel -> Start, Lauf beendet (plus letzte Ergebnisse)

struct __attribute__((packed)) Pkt {
  uint8_t  magic;
  uint8_t  type;
  uint16_t runId;       // START/ACK: zufällige Lauf-ID
  uint8_t  copy;        // START: Kopie 0..7; PONG: Bit0 = Ziel scharf
  uint32_t offsetUs;    // START: Zeit von Auslösung bis Sendebeginn
  uint16_t battMv;      // Akkuspannung des Senders (0 = unbekannt)
  uint16_t runNo;       // Ziel: aktuelle bzw. nächste Laufnummer
  uint16_t resNo;       // Ziel: Nummer des letzten beendeten Laufs
  uint32_t resMs[3];    // Ziel: letzte 3 Zeiten in ms, neueste zuerst (0 = leer)
};

// Feste Verzögerung zwischen transmit()-Aufruf und echtem Sendebeginn (µs).
// Kalibrieren: Start und Ziel nebeneinander, gleichzeitig auslösen, Zeit sollte ~0 sein.
#define TX_CAL_US 1500

uint16_t battMv = 0;
uint16_t runNo = 0;                 // Ziel: nächste Laufnummer; Start: zuletzt vom Ziel gemeldet
uint16_t resNo = 0;                 // Nummer des letzten beendeten Laufs
uint32_t resMs[3] = {0, 0, 0};      // letzte 3 Zeiten in ms, neueste zuerst
volatile bool radioFlag = false;    // DIO1-Interrupt
uint32_t lastDraw = 0;
uint32_t lastBatt = 0;
uint16_t otherBattMv = 0;           // Akkuspannung der Gegenstelle (aus dem Funkpaket)

volatile bool rxFlag = false;       // Ziel: Empfang (Zeitstempel in rxUs)
volatile int64_t rxUs = 0;
uint32_t clashUntil = 0;            // Meldung "ROLLE 2x!" bis zu diesem Zeitpunkt
Preferences prefs;
bool isStart = true;                // Rolle, im Flash gespeichert

void IRAM_ATTR onRadio() { rxUs = esp_timer_get_time(); rxFlag = true; radioFlag = true; }

bool initRadio() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  // freq, bw, sf, cr, syncword, power(dBm), preamble, tcxoV, useLDO
  int st = radio.begin(LORA_FREQ, 125.0, 7, 5, 0x12, 14, 8, 1.8, false);
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(onRadio);
  return st == RADIOLIB_ERR_NONE;
}

void sendPkt(uint8_t type, uint16_t runId, uint8_t copy, uint32_t offsetUs) {
  Pkt p;
  memset(&p, 0, sizeof(p));
  p.magic = MAGIC;
  p.type = type;
  p.runId = runId;
  p.copy = copy;
  p.offsetUs = offsetUs;
  p.battMv = battMv;
  p.runNo = runNo;
  p.resNo = resNo;
  memcpy(p.resMs, resMs, sizeof(resMs));
  radio.transmit((uint8_t*)&p, sizeof(p));
  radioFlag = false;                 // TX-Done-Interrupt verwerfen
}

// Akkuspannung (Heltec V3): VBAT_READ = GPIO 1, ADC_CTRL = GPIO 37.
// V3.1: ADC_CTRL muss LOW sein, damit der Spannungsteiler (390k/100k, Faktor ca. 4,9) misst.
// Bei der V3.2 ist die Beschaltung anders: zeigt das Display Unsinn, VBAT_CTRL_ACTIVE auf HIGH ändern.
#define VBAT_CTRL_PIN    37
#define VBAT_READ_PIN    1
#define VBAT_CTRL_ACTIVE HIGH
#define VBAT_FACTOR_X100 490      // Feinabgleich mit Multimeter (z. B. 485 oder 495)

uint16_t readBattMv() {
  pinMode(VBAT_CTRL_PIN, OUTPUT);
  digitalWrite(VBAT_CTRL_PIN, VBAT_CTRL_ACTIVE);
  delay(10);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogReadMilliVolts(VBAT_READ_PIN); delay(2); }
  pinMode(VBAT_CTRL_PIN, INPUT);            // Teiler wieder abschalten (spart Strom)
  return (uint16_t)((sum / 8) * VBAT_FACTOR_X100 / 100UL);
}

// ---- Display (beide Rollen) ----
void initDisplay() {
  gpio_deep_sleep_hold_dis();                // nach dem Aufwachen: Haltezustand aufheben
  gpio_hold_dis((gpio_num_t)VEXT_PIN);
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);       // Vext an: versorgt das OLED
  delay(50);
  display.begin();
}

// Akkuspannung als Text: "3.92", oder "--" wenn unbekannt bzw. kein Akku angeschlossen
void fmtV(char* out, size_t n, uint16_t mv) {
  if (mv < 2000) snprintf(out, n, "--");
  else snprintf(out, n, "%u.%02u", (unsigned)(mv / 1000), (unsigned)((mv % 1000) / 10));
}

// Zeile 1: Verbindung links, Status rechts.
// Zeile 2: Laufnummer links, Akku rechts (S = Start-Gate, Z = Ziel-Gate).
// Zeilen 3-5: letzte 3 Zeiten.
void drawScreen(const char* conn, const char* state, uint16_t sMv = 0, uint16_t zMv = 0) {
  char b[28], vs[8], vz[8];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, conn);
  display.drawStr(128 - display.getStrWidth(state), 10, state);
  if (runNo) snprintf(b, sizeof(b), "Lauf %u", runNo); else snprintf(b, sizeof(b), "Lauf -");
  display.drawStr(0, 22, b);
  fmtV(vs, sizeof(vs), sMv);
  fmtV(vz, sizeof(vz), zMv);
  snprintf(b, sizeof(b), "S%s Z%sV", vs, vz);
  display.drawStr(128 - display.getStrWidth(b), 22, b);
  for (int i = 0; i < 3; i++) {
    display.setFont(i == 0 ? u8g2_font_7x13B_tr : u8g2_font_7x13_tr);   // neueste fett
    if (resMs[i] > 0) {
      snprintf(b, sizeof(b), "#%u %lu.%03lu s", (unsigned)(resNo - i),
               (unsigned long)(resMs[i] / 1000), (unsigned long)(resMs[i] % 1000));
    } else {
      snprintf(b, sizeof(b), "#- --.--- s");
    }
    display.drawStr(0, 37 + i * 13, b);
  }
  display.sendBuffer();
}

// ---- Blinken bei erkanntem Start (Display invertieren, nicht blockierend) ----
#define FLASH_PHASE_MS 200      // Dauer von an bzw. aus
#define FLASH_BLINKS   2        // Anzahl Blinker
uint32_t flashT0 = 0;           // 0 = kein Blinken aktiv
bool flashInv = false;

void startFlash() {
  flashT0 = millis();
  if (!flashT0) flashT0 = 1;
}

// Oft aufrufen (loop und Warteschleifen), schaltet die Invertierung zum richtigen Zeitpunkt
void updateFlash() {
  bool inv = false;
  if (flashT0) {
    uint32_t d = millis() - flashT0;
    if (d < 2UL * FLASH_BLINKS * FLASH_PHASE_MS) inv = ((d / FLASH_PHASE_MS) % 2 == 0);
    else flashT0 = 0;
  }
  if (inv != flashInv) {
    flashInv = inv;
    display.sendF("c", inv ? 0xA7 : 0xA6);   // SSD1306: 0xA7 = invertiert, 0xA6 = normal
  }
}

// =====================================================================
// ROLLE START
// =====================================================================
volatile bool trig = false;
volatile int64_t trigUs = 0;
volatile int64_t lastTrigUs = -10000000;

bool running = false;   // Ziel meldet: Lauf gestartet, noch nicht im Ziel
bool noAck = false;     // letzter Start wurde nicht bestätigt
uint32_t lastPing = 0, lastPong = 0;

void IRAM_ATTR onStart() {
  int64_t now = esp_timer_get_time();
  if (!trig && now - lastTrigUs > 5000000) {   // 5 s Sperre (Entprellen + Re-Arm)
    trigUs = now;
    lastTrigUs = now;
    trig = true;
  }
}

void takeResults(const Pkt& r) {
  runNo = r.runNo;
  otherBattMv = r.battMv;
  resNo = r.resNo;
  memcpy(resMs, r.resMs, sizeof(resMs));
}

void drawStart() {
  bool link = lastPong && (millis() - lastPong < 10000);
  drawScreen(link ? "Ziel OK" : "Ziel AUS!",
             millis() < clashUntil ? "ROLLE 2x!" : (noAck ? "KEIN ACK" : (running ? "GESTARTET" : "BEREIT")),
             battMv, otherBattMv);
}

// Wartet auf ein Antwortpaket vom Ziel
bool waitFor(uint8_t type, uint16_t runId, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (radioFlag) {
      radioFlag = false;
      Pkt r;
      if (radio.readData((uint8_t*)&r, sizeof(r)) == RADIOLIB_ERR_NONE &&
          r.magic == MAGIC && r.type == type && (type == T_PONG || r.runId == runId)) {
        takeResults(r);
        if (type == T_PONG) { running = (r.copy & 1); noAck = false; }
        return true;
      }
      radio.startReceive();
    }
    updateFlash();
    delay(1);
  }
  return false;
}

void blinkFast(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { digitalWrite(LED_PIN, (millis() / 80) % 2); updateFlash(); delay(10); }
}

void doStart() {
  startFlash();                                   // Display blinkt, sobald der Start erkannt ist
  uint16_t run = (uint16_t)esp_random();
  bool acked = false;
  for (uint8_t i = 0; i < 8 && !acked; i++) {     // erneut senden, bis das Ziel bestätigt
    sendPkt(T_START, run, i, (uint32_t)(esp_timer_get_time() - trigUs));
    radio.startReceive();
    acked = waitFor(T_ACK, run, 300);
  }
  Serial.printf("Start Lauf-ID %u: %s\n", run, acked ? "bestaetigt" : "KEINE Bestaetigung");
  running = acked;
  noAck = !acked;
  drawStart();
  if (acked) {
    digitalWrite(LED_PIN, HIGH);
    uint32_t t1 = millis();
    while (millis() - t1 < 1500) { updateFlash(); delay(5); }
  } else {
    blinkFast(3000);
  }
  trig = false;
}

void setupStart() {
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  initDisplay();
  drawScreen("Starte ...", "");
  if (!initRadio()) { Serial.println("LoRa Fehler"); while (true) delay(1000); }
  battMv = readBattMv();
  // RISING: Auslösung bei steigender Flanke (z. B. NC-Kontakt oder Lichtschranke)
  attachInterrupt(digitalPinToInterrupt(START_PIN), onStart, RISING);
  radio.startReceive();
  Serial.println("Start-Gate bereit");
}

void loopStart() {
  if (radioFlag) {                                // Ergebnis vom Ziel (T_RESULT)
    radioFlag = false;
    Pkt r;
    if (radio.readData((uint8_t*)&r, sizeof(r)) == RADIOLIB_ERR_NONE && r.magic == MAGIC) {
      if (r.type == T_RESULT) {
        takeResults(r);
        running = false;
        noAck = false;
        lastDraw = 0;                             // sofort neu zeichnen
      } else if (r.type == T_PING || r.type == T_START) {
        clashUntil = millis() + 10000;            // ein zweiter Start sendet: Rolle doppelt
      }
    }
    radio.startReceive();
  }

  if (trig) { doStart(); radio.startReceive(); }

  if (millis() - lastBatt > 30000) { lastBatt = millis(); battMv = readBattMv(); }

  if (millis() - lastPing > 3000) {               // Heartbeat alle 3 s
    lastPing = millis();
    sendPkt(T_PING, 0, 0, 0);
    radio.startReceive();
    if (waitFor(T_PONG, 0, 300)) lastPong = millis();
    radio.startReceive();
  }

  bool link = lastPong && (millis() - lastPong < 10000);
  // LED an = Ziel erreichbar, blinkt = keine Verbindung (dann nicht starten!)
  digitalWrite(LED_PIN, link ? HIGH : (millis() / 300) % 2);

  if (millis() - lastDraw > 500) { lastDraw = millis(); drawStart(); }
  updateFlash();
  delay(5);
}

// =====================================================================
// ROLLE ZIEL
// =====================================================================
WebServer server(80);

volatile bool finishFlag = false;
volatile int64_t finishUs = 0;
volatile int64_t lastFinishIsr = 0;

bool armed = false;
int64_t startUs = 0;
uint16_t lastRun = 0;
bool haveRun = false;

uint32_t lastSeenMs = 0;
bool seenOnce = false;
float lastRssi = 0, lastSnr = 0;
uint16_t startBatt = 0;

// ---- Uhrzeit (wird vom Handy über die Weboberfläche gesetzt) ----
bool timeSet = false;
int64_t epochOffsetUs = 0;     // Unix-Zeit in µs = esp_timer_get_time() + epochOffsetUs
int tzMin = 0;                 // Zeitzone des Handys in Minuten gegenüber UTC

// Unix-Sekunden (UTC) zu einem esp_timer-Zeitpunkt, 0 wenn die Uhr nicht gesetzt ist
uint32_t toUnixS(int64_t timerUs) {
  return timeSet ? (uint32_t)((timerUs + epochOffsetUs) / 1000000) : 0;
}

String fmtLocal(uint32_t unixS, bool dateToo = true) {
  if (!unixS) return "";
  time_t t = (time_t)unixS + tzMin * 60;
  struct tm tmv;
  gmtime_r(&t, &tmv);
  char b[24];
  strftime(b, sizeof(b), dateToo ? "%Y-%m-%d %H:%M:%S" : "%H:%M:%S", &tmv);
  return String(b);
}

// ---- Läufe (CSV im Flash: "Lauf;Sekunden;StartUnix;Start;Ziel") und Ereignisse ----
#define CSV_FILE "/runs.csv"
#define EVT_FILE "/events.csv"
#define MAX_RECENT 10
#define MAX_EVT 10
#define RUN_TIMEOUT_S 900      // scharfer Lauf ohne Zieldurchlauf wird nach dieser Zeit verworfen
struct Run { uint16_t no; float s; uint32_t startUnix; };
struct Evt { uint32_t unixS; char text[48]; };
Evt evts[MAX_EVT];
int nEvt = 0;
const char* noteText = "";     // kurze Meldung auf dem Display (5 s)
uint32_t noteUntil = 0;
Run recent[MAX_RECENT];
int nRecent = 0;
uint16_t totalRuns = 0;
float bestTime = 0;

// Auffälligkeit protokollieren: Liste für die Webseite, events.csv im Flash, kurze Meldung am Display
void logEvent(const char* what, const String& details, const char* note) {
  uint32_t u = toUnixS(esp_timer_get_time());
  if (nEvt < MAX_EVT) nEvt++;
  for (int i = nEvt - 1; i > 0; i--) evts[i] = evts[i - 1];
  evts[0].unixS = u;
  snprintf(evts[0].text, sizeof(evts[0].text), "%s%s%s", what, details.length() ? ": " : "", details.c_str());
  File f = LittleFS.open(EVT_FILE, "a");
  if (f) {
    if (f.size() == 0) f.println("Unix;Zeit;Ereignis;Details");
    f.printf("%lu;%s;%s;%s\n", (unsigned long)u, fmtLocal(u).c_str(), what, details.c_str());
    f.close();
  }
  noteText = note;
  noteUntil = millis() + 5000;
  lastDraw = 0;
  Serial.printf("Ereignis: %s %s\n", what, details.c_str());
}

void IRAM_ATTR onFinish() {
  int64_t now = esp_timer_get_time();
  if (now - lastFinishIsr > 200000) {            // 200 ms Entprellung
    lastFinishIsr = now;
    finishUs = now;
    finishFlag = true;
  }
}

void pushRecent(uint16_t no, float s, uint32_t startUnix) {
  if (nRecent < MAX_RECENT) nRecent++;
  for (int i = nRecent - 1; i > 0; i--) recent[i] = recent[i - 1];
  recent[0] = { no, s, startUnix };
}

// Ergebnisse für Funk und Display aus den gespeicherten Läufen ableiten
void updateResults() {
  resNo = totalRuns;
  for (int i = 0; i < 3; i++) resMs[i] = (i < nRecent) ? (uint32_t)(recent[i].s * 1000.0f + 0.5f) : 0;
  runNo = totalRuns + 1;
}

void loadRuns() {
  File f = LittleFS.open(CSV_FILE, "r");
  if (f) {
    while (f.available()) {
      String l = f.readStringUntil('\n');
      int i1 = l.indexOf(';');
      if (i1 < 1) continue;
      int i2 = l.indexOf(';', i1 + 1);
      int i3 = (i2 > 0) ? l.indexOf(';', i2 + 1) : -1;
      uint16_t no = l.substring(0, i1).toInt();
      float s = (i2 > 0 ? l.substring(i1 + 1, i2) : l.substring(i1 + 1)).toFloat();
      uint32_t su = 0;
      if (i2 > 0) su = (uint32_t)strtoul((i3 > 0 ? l.substring(i2 + 1, i3) : l.substring(i2 + 1)).c_str(), nullptr, 10);
      if (no == 0) continue;                     // Kopfzeile überspringen
      totalRuns = no;
      if (bestTime == 0 || s < bestTime) bestTime = s;
      pushRecent(no, s, su);
    }
    f.close();
  }
  updateResults();
}

void appendRun(float s, uint32_t startUnix, uint32_t endUnix) {
  totalRuns++;
  File f = LittleFS.open(CSV_FILE, "a");
  if (f) {
    if (f.size() == 0) f.println("Lauf;Sekunden;StartUnix;Start;Ziel");
    f.printf("%u;%.3f;%lu;%s;%s\n", totalRuns, s, (unsigned long)startUnix,
             fmtLocal(startUnix).c_str(), fmtLocal(endUnix).c_str());
    f.close();
  }
  if (bestTime == 0 || s < bestTime) bestTime = s;
  pushRecent(totalRuns, s, startUnix);
  updateResults();
}

void clearRuns() {
  LittleFS.remove(CSV_FILE);
  LittleFS.remove(EVT_FILE);
  nRecent = 0; totalRuns = 0; bestTime = 0; armed = false; nEvt = 0;
  updateResults();
  lastDraw = 0;
}

void drawZiel() {
  bool link = seenOnce && (millis() - lastSeenMs < 10000);
  drawScreen(link ? "Start OK" : "Start AUS!",
             millis() < clashUntil ? "ROLLE 2x!" : (millis() < noteUntil ? noteText : (armed ? "GESTARTET" : "BEREIT")),
             otherBattMv, battMv);
}

void handleRx() {
  int64_t rx = rxUs;                             // Empfangszeit sofort sichern
  Pkt p;
  if (radio.readData((uint8_t*)&p, sizeof(p)) != RADIOLIB_ERR_NONE || p.magic != MAGIC) return;
  if (p.type == T_PONG || p.type == T_ACK || p.type == T_RESULT) {   // das senden nur Ziele
    clashUntil = millis() + 10000;               // ein zweites Ziel in Reichweite: Rolle doppelt
    return;
  }
  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();
  lastSeenMs = millis();
  seenOnce = true;
  startBatt = p.battMv;
  otherBattMv = p.battMv;

  if (p.type == T_PING) {
    sendPkt(T_PONG, 0, armed ? 1 : 0, 0);        // Bit0 = Ziel ist scharf
  } else if (p.type == T_START) {
    if (!haveRun || p.runId != lastRun) {        // nur die erste Kopie zählt
      if (armed) logEvent("Start ersetzt unbeendeten Lauf", "Lauf " + String(runNo), "ABGEBROCH.");
      lastRun = p.runId;
      haveRun = true;
      int64_t airUs = (int64_t)radio.getTimeOnAir(sizeof(p));
      // Startzeitpunkt = Empfangsende - Airtime - Verzögerung seit Auslösung - Kalibrierung
      startUs = rx - airUs - (int64_t)p.offsetUs - TX_CAL_US;
      armed = true;
      lastDraw = 0;
      startFlash();                              // Display blinkt bei erkanntem Start
      Serial.printf("Start empfangen, Lauf-ID %u (Kopie %u)\n", p.runId, p.copy);
    }
    sendPkt(T_ACK, p.runId, 0, 0);               // jede Kopie bestätigen
  }
}

// ---- Weboberfläche ----
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="de"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Zeitnahme</title>
<style>
:root{color-scheme:light dark}
body{font-family:system-ui,sans-serif;margin:0 auto;padding:16px;max-width:480px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.pill{font-size:13px;padding:4px 10px;border-radius:8px;background:#8883}
.ok{background:#2a7d3a33}.bad{background:#c0392b33}
.big{font-size:3.2em;font-weight:500;text-align:center;margin:.2em 0}
.c{text-align:center;color:#888}
table{width:100%;border-collapse:collapse;margin:8px 0}
td{padding:6px 0;border-bottom:1px solid #8883}td+td{text-align:right}
button,a.b{flex:1;padding:10px;font-size:15px;text-align:center;border:1px solid #8886;
border-radius:8px;background:none;color:inherit;text-decoration:none}
</style></head><body>
<h2 style="font-size:18px;font-weight:500">Zeitnahme</h2>
<div class="row"><span id="link" class="pill">...</span><span id="sig" class="pill"></span><span id="bat" class="pill"></span></div>
<div class="c" id="state">...</div>
<div class="big" id="last">-</div>
<div class="c" id="best"></div>
<table id="runs"></table>
<h3 style="font-size:15px;font-weight:500">Auffälligkeiten</h3>
<table id="evts"></table>
<div class="row"><a class="b" href="/export.csv" download>Läufe (CSV)</a>
<a class="b" href="/events.csv" download>Ereignisse (CSV)</a></div>
<div class="row"><button onclick="clr()">Alles löschen</button></div>
<script>
const f=x=>x.toFixed(3).replace('.',',')+' s';
let tsent=0;
function sync(d){
 if(!d.timeSet||Date.now()-tsent>60000){
  tsent=Date.now();
  fetch('/settime?ms='+Date.now()+'&tz='+(-new Date().getTimezoneOffset()));
 }
}
async function upd(){
 try{
  const d=await (await fetch('/api')).json();
  sync(d);
  const l=document.getElementById('link');
  if(d.age<0){l.textContent='Start noch nicht gesehen';l.className='pill bad';}
  else if(d.link){l.textContent='Start verbunden';l.className='pill ok';}
  else{l.textContent='Start offline seit '+d.age+' s';l.className='pill bad';}
  sig.textContent=d.age<0?'':'RSSI '+Math.round(d.rssi)+' dBm · SNR '+d.snr.toFixed(0)+' dB';
  bat.textContent=[d.batt>1999?'Start '+(d.batt/1000).toFixed(2)+' V':'',d.zbatt>1999?'Ziel '+(d.zbatt/1000).toFixed(2)+' V':''].filter(x=>x).join(' · ');
  state.textContent=d.armed?'Läuft ...':(d.total?'Lauf '+d.total+' beendet':'Bereit');
  last.textContent=d.runs.length?f(d.runs[0].s):'-';
  best.textContent=d.best>0?'Bestzeit '+f(d.best):'';
  runs.innerHTML=d.runs.map(r=>'<tr><td>#'+r.n+'</td><td>'+r.t+'</td><td>'+f(r.s)+'</td></tr>').join('');
  evts.innerHTML=d.events.length?d.events.map(e=>'<tr><td>'+e.t+'</td><td>'+e.e+'</td></tr>').join(''):'<tr><td>keine</td></tr>';
 }catch(e){const l=document.getElementById('link');l.textContent='Keine Verbindung zum Ziel-Gate';l.className='pill bad';}
}
async function clr(){if(confirm('Alle Läufe und Ereignisse wirklich löschen?')){await fetch('/clear');upd();}}
upd();setInterval(upd,1500);
</script></body></html>)HTML";

void handleApi() {
  bool link = seenOnce && (millis() - lastSeenMs < 10000);
  int age = seenOnce ? (int)((millis() - lastSeenMs) / 1000) : -1;
  String j = "{\"armed\":" + String(armed ? "true" : "false");
  j += ",\"link\":" + String(link ? "true" : "false");
  j += ",\"age\":" + String(age);
  j += ",\"rssi\":" + String(lastRssi, 0);
  j += ",\"snr\":" + String(lastSnr, 1);
  j += ",\"batt\":" + String(startBatt);
  j += ",\"zbatt\":" + String(battMv);
  j += ",\"timeSet\":" + String(timeSet ? "true" : "false");
  j += ",\"total\":" + String(totalRuns);
  j += ",\"best\":" + String(bestTime, 3);
  j += ",\"runs\":[";
  for (int i = 0; i < nRecent; i++) {
    if (i) j += ",";
    j += "{\"n\":" + String(recent[i].no) + ",\"s\":" + String(recent[i].s, 3) +
         ",\"t\":\"" + fmtLocal(recent[i].startUnix, false) + "\"}";
  }
  j += "],\"events\":[";
  for (int i = 0; i < nEvt; i++) {
    if (i) j += ",";
    j += "{\"t\":\"" + fmtLocal(evts[i].unixS, false) + "\",\"e\":\"" + String(evts[i].text) + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void setupZiel() {
  pinMode(ZIEL_PIN, INPUT_PULLUP);
  initDisplay();
  drawScreen("Starte ...", "");
  if (!LittleFS.begin(true)) Serial.println("LittleFS Fehler");
  loadRuns();
  logEvent("Neustart", "", "NEUSTART");
  battMv = readBattMv();
  if (!initRadio()) { Serial.println("LoRa Fehler"); while (true) delay(1000); }
  radio.startReceive();
  attachInterrupt(digitalPinToInterrupt(ZIEL_PIN), onFinish, RISING);

  WiFi.softAP("Zeitnahme", "12345678");          // Passwort ändern
  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api", handleApi);
  server.on("/export.csv", []() {
    File f = LittleFS.open(CSV_FILE, "r");
    if (!f) { server.send(404, "text/plain", "Keine Läufe"); return; }
    server.sendHeader("Content-Disposition", "attachment; filename=zeiten.csv");
    server.streamFile(f, "text/csv");
    f.close();
  });
  server.on("/events.csv", []() {
    File f = LittleFS.open(EVT_FILE, "r");
    if (!f) { server.send(404, "text/plain", "Keine Ereignisse"); return; }
    server.sendHeader("Content-Disposition", "attachment; filename=ereignisse.csv");
    server.streamFile(f, "text/csv");
    f.close();
  });
  server.on("/settime", []() {                   // Handy schickt Unix-ms und Zeitzone
    int64_t ms = strtoll(server.arg("ms").c_str(), nullptr, 10);
    int tz = server.arg("tz").toInt();
    if (ms > 1700000000000LL) {                  // plausibler Wert (nach 2023)
      int64_t newOff = ms * 1000 - esp_timer_get_time();
      if (!timeSet || llabs(newOff - epochOffsetUs) > 2000000) {   // nur bei > 2 s Abweichung nachstellen
        epochOffsetUs = newOff;
        timeSet = true;
      }
      tzMin = tz;
    }
    server.send(200, "text/plain", "ok");
  });
  server.on("/clear", []() { clearRuns(); server.send(200, "text/plain", "ok"); });
  server.begin();
  Serial.println("Ziel-Gate bereit: WLAN 'Zeitnahme', http://192.168.4.1");
}

void loopZiel() {
  server.handleClient();
  updateFlash();
  if (rxFlag) {
    rxFlag = false;
    handleRx();
    rxFlag = false;                               // TX-Done-Interrupt der Antwort verwerfen
    radio.startReceive();
  }
  if (finishFlag) {
    finishFlag = false;
    int64_t fu = finishUs;
    if (armed) {
      int64_t dt = fu - startUs;
      if (dt > 500000) {                          // min. 0,5 s, sonst Fehlauslösung
        float s = dt / 1000000.0f;
        armed = false;
        appendRun(s, toUnixS(startUs), toUnixS(fu));   // aktualisiert auch resNo/resMs/runNo
        sendPkt(T_RESULT, 0, 0, 0);               // sofort an den Start melden
        rxFlag = false;
        radio.startReceive();
        lastDraw = 0;
        Serial.printf("Lauf %u: %.3f s\n", totalRuns, s);
      } else {
        logEvent("Zielimpuls zu kurz", String(dt / 1000000.0f, 3) + " s", "ZU KURZ");   // Lauf bleibt scharf
      }
    } else {
      logEvent("Zielimpuls ohne Start", "", "OHNE START");
    }
  }

  // Scharfer Lauf ohne Zieldurchlauf: nach RUN_TIMEOUT_S verwerfen
  if (armed && (esp_timer_get_time() - startUs) > (int64_t)RUN_TIMEOUT_S * 1000000LL) {
    armed = false;
    logEvent("Zeitueberschreitung", "Lauf " + String(runNo) + " verworfen", "TIMEOUT");
  }
  if (millis() - lastBatt > 30000) { lastBatt = millis(); battMv = readBattMv(); }
  if (millis() - lastDraw > 500) { lastDraw = millis(); drawZiel(); }
}

// =====================================================================
// Rollenwahl, Taste (PRG, GPIO 0) und Deep Sleep
// =====================================================================
void showBig(const char* top, const char* big) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, top);
  display.setFont(u8g2_font_logisoso24_tr);
  display.drawStr((128 - display.getStrWidth(big)) / 2, 50, big);
  display.sendBuffer();
}

void goToSleep() {
  showBig("Ausschalten ...", "AUS");
  if (!isStart) logEvent("Ausgeschaltet", "", "");
  delay(800);
  radio.sleep();
  if (!isStart) { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); }
  display.setPowerSave(1);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(VEXT_PIN, HIGH);                       // OLED-Versorgung aus
  gpio_hold_en((gpio_num_t)VEXT_PIN);
  gpio_deep_sleep_hold_en();
  while (digitalRead(BUTTON_PIN) == LOW) delay(10);   // erst loslassen, sonst weckt die Taste sofort
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
  esp_deep_sleep_start();
}

void switchRole() {
  prefs.putBool("start", !isStart);
  showBig("Neue Rolle:", isStart ? "ZIEL" : "START");
  delay(1200);
  ESP.restart();
}

// Taste halten: Anzeige zeigt die jeweilige Aktion, beim Loslassen wird sie ausgeführt
void buttonCheck() {
  static uint32_t t0 = 0;
  static bool down = false;
  static uint8_t phase = 0;                           // 0 nichts, 1 Rolle wechseln, 2 Deep Sleep
  bool now = (digitalRead(BUTTON_PIN) == LOW);
  if (now && !down) { down = true; t0 = millis(); phase = 0; }
  if (now && down) {
    uint32_t h = millis() - t0;
    uint8_t p = (h >= HOLD_SLEEP_MS) ? 2 : (h >= HOLD_ROLE_MS) ? 1 : 0;
    if (p != phase) {
      phase = p;
      if (p == 1) showBig("Loslassen = Wechsel", isStart ? "ZIEL" : "START");
      if (p == 2) showBig("Loslassen:", "AUS");
    }
    if (phase) lastDraw = millis();                   // normale Anzeige währenddessen pausieren
  }
  if (!now && down) {
    down = false;
    if (phase == 2) goToSleep();
    else if (phase == 1) switchRole();
    phase = 0;
    lastDraw = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  prefs.begin("gate", false);
  isStart = prefs.getBool("start", true);
  initDisplay();
  showBig("Rolle", isStart ? "START" : "ZIEL");
  uint32_t t = millis();
  while (digitalRead(BUTTON_PIN) == LOW && millis() - t < 3000) delay(10);   // Taste nach dem Aufwecken erst loslassen
  delay(1000);
  if (isStart) setupStart(); else setupZiel();
}

void loop() {
  buttonCheck();
  if (isStart) loopStart(); else loopZiel();
}
