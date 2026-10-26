// Zeitnahme Start/Ziel per LoRa + WLAN-Webserver am Ziel  (v4 - Multi-Fahrer)
// Board: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
// Bibliotheken: RadioLib, U8g2 (beide über den Bibliotheksverwalter)
//
// Neu in v4:
//  - Mehrere Fahrer gleichzeitig auf der Strecke (Queue-System)
//  - Start-Display zeigt die zugewiesene Laufnummer groß an
//  - Weboberfläche: Zuordnung Zieldurchfahrt ↔ Laufnummer korrigierbar
//  - Fahrer ohne Zieldurchfahrt (Ausfall) können entfernt werden
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
#include <Update.h>
#include "driver/gpio.h"

#define FIRMWARE_VERSION "4.1"

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
#define T_ACK    4   // Ziel -> Start, Start angekommen (enthält zugewiesene Laufnummer)
#define T_RESULT 5   // Ziel -> Start, Lauf beendet (plus letzte Ergebnisse)

struct __attribute__((packed)) Pkt {
  uint8_t  magic;
  uint8_t  type;
  uint16_t runId;       // START/ACK: zufällige Lauf-ID
  uint8_t  copy;        // START: Kopie 0..7; PONG: Bit0 = Fahrer auf Strecke
  uint32_t offsetUs;    // START: Zeit von Auslösung bis Sendebeginn
  uint16_t battMv;      // Akkuspannung des Senders (0 = unbekannt)
  uint16_t runNo;       // ACK: zugewiesene Laufnummer für diesen Start
  uint16_t resNo;       // Ziel: Nummer des letzten beendeten Laufs
  uint32_t resMs[3];    // Ziel: letzte 3 Zeiten in ms, neueste zuerst (0 = leer)
  uint8_t  queueLen;    // Ziel: Anzahl Fahrer auf der Strecke
};

// Feste Verzögerung zwischen transmit()-Aufruf und echtem Sendebeginn (µs).
#define TX_CAL_US 1500

uint16_t battMv = 0;
uint16_t runNo = 0;                 // Start: zuletzt zugewiesene Laufnummer
uint16_t resNo = 0;                 // Nummer des letzten beendeten Laufs
uint32_t resMs[3] = {0, 0, 0};      // letzte 3 Zeiten in ms, neueste zuerst
volatile bool radioFlag = false;    // DIO1-Interrupt
uint32_t lastDraw = 0;
uint32_t lastBatt = 0;
uint16_t otherBattMv = 0;           // Akkuspannung der Gegenstelle (aus dem Funkpaket)
uint8_t queueLen = 0;               // Anzahl Fahrer auf der Strecke (vom Ziel)

volatile bool rxFlag = false;       // Ziel: Empfang (Zeitstempel in rxUs)
volatile int64_t rxUs = 0;
uint32_t clashUntil = 0;            // Meldung "ROLLE 2x!" bis zu diesem Zeitpunkt
Preferences prefs;
bool isStart = true;                // Rolle, im Flash gespeichert

void IRAM_ATTR onRadio() { rxUs = esp_timer_get_time(); rxFlag = true; radioFlag = true; }

bool initRadio() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(LORA_FREQ, 125.0, 7, 5, 0x12, 14, 8, 1.8, false);
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(onRadio);
  return st == RADIOLIB_ERR_NONE;
}

void sendPkt(uint8_t type, uint16_t runId, uint8_t copy, uint32_t offsetUs, uint16_t assignedRunNo = 0);

// Akkuspannung (Heltec V3)
#define VBAT_CTRL_PIN    37
#define VBAT_READ_PIN    1
#define VBAT_CTRL_ACTIVE HIGH
#define VBAT_FACTOR_X100 490

uint16_t readBattMv() {
  pinMode(VBAT_CTRL_PIN, OUTPUT);
  digitalWrite(VBAT_CTRL_PIN, VBAT_CTRL_ACTIVE);
  delay(10);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogReadMilliVolts(VBAT_READ_PIN); delay(2); }
  pinMode(VBAT_CTRL_PIN, INPUT);
  return (uint16_t)((sum / 8) * VBAT_FACTOR_X100 / 100UL);
}

// ---- Display (beide Rollen) ----
void initDisplay() {
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)VEXT_PIN);
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(50);
  display.begin();
}

void fmtV(char* out, size_t n, uint16_t mv) {
  if (mv < 2000) snprintf(out, n, "--");
  else snprintf(out, n, "%u.%02u", (unsigned)(mv / 1000), (unsigned)((mv % 1000) / 10));
}

// ---- Blinken bei erkanntem Start ----
#define FLASH_PHASE_MS 200
#define FLASH_BLINKS   2
uint32_t flashT0 = 0;
bool flashInv = false;

void startFlash() {
  flashT0 = millis();
  if (!flashT0) flashT0 = 1;
}

void updateFlash() {
  bool inv = false;
  if (flashT0) {
    uint32_t d = millis() - flashT0;
    if (d < 2UL * FLASH_BLINKS * FLASH_PHASE_MS) inv = ((d / FLASH_PHASE_MS) % 2 == 0);
    else flashT0 = 0;
  }
  if (inv != flashInv) {
    flashInv = inv;
    display.sendF("c", inv ? 0xA7 : 0xA6);
  }
}

// =====================================================================
// ROLLE START
// =====================================================================
volatile bool trig = false;
volatile int64_t trigUs = 0;
volatile int64_t lastTrigUs = -10000000;

bool noAck = false;
uint32_t lastPing = 0, lastPong = 0;
uint16_t lastAssignedRunNo = 0;     // zuletzt zugewiesene Laufnummer (für große Anzeige)
uint32_t showRunNoUntil = 0;        // Laufnummer groß anzeigen bis zu diesem Zeitpunkt

void IRAM_ATTR onStart() {
  int64_t now = esp_timer_get_time();
  if (!trig && now - lastTrigUs > 2000000) {   // 2 s Sperre (Entprellen)
    trigUs = now;
    lastTrigUs = now;
    trig = true;
  }
}

void takeResults(const Pkt& r) {
  otherBattMv = r.battMv;
  resNo = r.resNo;
  memcpy(resMs, r.resMs, sizeof(resMs));
  queueLen = r.queueLen;
}

// Start-Display: zeigt die zugewiesene Laufnummer groß an
void drawStart() {
  char b[28], vs[8], vz[8];
  bool link = lastPong && (millis() - lastPong < 10000);
  
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, link ? "Ziel OK" : "Ziel AUS!");
  
  const char* state = millis() < clashUntil ? "ROLLE 2x!" : (noAck ? "KEIN ACK" : "BEREIT");
  display.drawStr(128 - display.getStrWidth(state), 10, state);
  
  // Zeile 2: Fahrer auf Strecke + Akku
  snprintf(b, sizeof(b), "Auf Strecke: %u", queueLen);
  display.drawStr(0, 22, b);
  fmtV(vs, sizeof(vs), battMv);
  fmtV(vz, sizeof(vz), otherBattMv);
  snprintf(b, sizeof(b), "S%s Z%sV", vs, vz);
  display.drawStr(128 - display.getStrWidth(b), 22, b);
  
  // Wenn gerade ein Start erfolgt ist: Laufnummer groß anzeigen
  if (millis() < showRunNoUntil && lastAssignedRunNo > 0) {
    display.setFont(u8g2_font_logisoso24_tr);
    snprintf(b, sizeof(b), "#%u", lastAssignedRunNo);
    display.drawStr((128 - display.getStrWidth(b)) / 2, 52, b);
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0, 64, "DEINE NUMMER");
  } else {
    // Letzte 3 Zeiten
    for (int i = 0; i < 3; i++) {
      display.setFont(i == 0 ? u8g2_font_7x13B_tr : u8g2_font_7x13_tr);
      if (resMs[i] > 0) {
        snprintf(b, sizeof(b), "#%u %lu.%03lu s", (unsigned)(resNo - i),
                 (unsigned long)(resMs[i] / 1000), (unsigned long)(resMs[i] % 1000));
      } else {
        snprintf(b, sizeof(b), "#- --.--- s");
      }
      display.drawStr(0, 37 + i * 13, b);
    }
  }
  display.sendBuffer();
}

bool waitFor(uint8_t type, uint16_t runId, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (radioFlag) {
      radioFlag = false;
      Pkt r;
      if (radio.readData((uint8_t*)&r, sizeof(r)) == RADIOLIB_ERR_NONE &&
          r.magic == MAGIC && r.type == type && (type == T_PONG || r.runId == runId)) {
        takeResults(r);
        if (type == T_ACK) {
          lastAssignedRunNo = r.runNo;
          runNo = r.runNo;
        }
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
  startFlash();
  uint16_t run = (uint16_t)esp_random();
  bool acked = false;
  for (uint8_t i = 0; i < 8 && !acked; i++) {
    sendPkt(T_START, run, i, (uint32_t)(esp_timer_get_time() - trigUs));
    radio.startReceive();
    acked = waitFor(T_ACK, run, 300);
  }
  Serial.printf("Start Lauf-ID %u: %s, Laufnummer %u\n", run, acked ? "bestaetigt" : "KEINE Bestaetigung", lastAssignedRunNo);
  noAck = !acked;
  if (acked) {
    showRunNoUntil = millis() + 5000;   // Laufnummer 5 Sekunden groß anzeigen
    digitalWrite(LED_PIN, HIGH);
    uint32_t t1 = millis();
    while (millis() - t1 < 1500) { updateFlash(); drawStart(); delay(50); }
  } else {
    blinkFast(3000);
  }
  trig = false;
}

void setupStart() {
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  initDisplay();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 32, "Starte ...");
  display.sendBuffer();
  if (!initRadio()) { Serial.println("LoRa Fehler"); while (true) delay(1000); }
  battMv = readBattMv();
  attachInterrupt(digitalPinToInterrupt(START_PIN), onStart, RISING);
  radio.startReceive();
  Serial.println("Start-Gate bereit");
}

void loopStart() {
  if (radioFlag) {
    radioFlag = false;
    Pkt r;
    if (radio.readData((uint8_t*)&r, sizeof(r)) == RADIOLIB_ERR_NONE && r.magic == MAGIC) {
      if (r.type == T_RESULT) {
        takeResults(r);
        noAck = false;
        lastDraw = 0;
      } else if (r.type == T_PING || r.type == T_START) {
        clashUntil = millis() + 10000;
      }
    }
    radio.startReceive();
  }

  if (trig) { doStart(); radio.startReceive(); }

  if (millis() - lastBatt > 30000) { lastBatt = millis(); battMv = readBattMv(); }

  if (millis() - lastPing > 3000) {
    lastPing = millis();
    sendPkt(T_PING, 0, 0, 0);
    radio.startReceive();
    if (waitFor(T_PONG, 0, 300)) lastPong = millis();
    radio.startReceive();
  }

  bool link = lastPong && (millis() - lastPong < 10000);
  digitalWrite(LED_PIN, link ? HIGH : (millis() / 300) % 2);

  if (millis() - lastDraw > 200) { lastDraw = millis(); drawStart(); }
  updateFlash();
  delay(5);
}

// =====================================================================
// ROLLE ZIEL - Multi-Fahrer Queue
// =====================================================================
WebServer server(80);

volatile bool finishFlag = false;
volatile int64_t finishUs = 0;
volatile int64_t lastFinishIsr = 0;

// Warteschlange für gestartete Fahrer
#define MAX_QUEUE 20
struct QueuedRun {
  uint16_t runNo;       // Laufnummer
  uint16_t runId;       // Funk-ID (zur Deduplizierung)
  int64_t startUs;      // Startzeitpunkt
  uint32_t startUnix;   // Unix-Zeit des Starts
  bool valid;           // Eintrag belegt
};
QueuedRun queue[MAX_QUEUE];
int queueCount = 0;

uint32_t lastSeenMs = 0;
bool seenOnce = false;
float lastRssi = 0, lastSnr = 0;
uint16_t startBatt = 0;

// Uhrzeit
bool timeSet = false;
int64_t epochOffsetUs = 0;
int tzMin = 0;

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

// Läufe und Ereignisse
#define CSV_FILE "/runs.csv"
#define EVT_FILE "/events.csv"
#define MAX_RECENT 10
#define MAX_EVT 10
#define RUN_TIMEOUT_S 1800     // 30 min Timeout für Fahrer auf Strecke
struct Run { uint16_t no; float s; uint32_t startUnix; };
struct Evt { uint32_t unixS; char text[48]; };
Evt evts[MAX_EVT];
int nEvt = 0;
const char* noteText = "";
uint32_t noteUntil = 0;
Run recent[MAX_RECENT];
int nRecent = 0;
uint16_t totalRuns = 0;
float bestTime = 0;

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
  if (now - lastFinishIsr > 200000) {
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
      if (no == 0) continue;
      totalRuns = no;
      if (bestTime == 0 || s < bestTime) bestTime = s;
      pushRecent(no, s, su);
    }
    f.close();
  }
  updateResults();
}

void appendRun(uint16_t no, float s, uint32_t startUnix, uint32_t endUnix) {
  File f = LittleFS.open(CSV_FILE, "a");
  if (f) {
    if (f.size() == 0) f.println("Lauf;Sekunden;StartUnix;Start;Ziel");
    f.printf("%u;%.3f;%lu;%s;%s\n", no, s, (unsigned long)startUnix,
             fmtLocal(startUnix).c_str(), fmtLocal(endUnix).c_str());
    f.close();
  }
  if (bestTime == 0 || s < bestTime) bestTime = s;
  pushRecent(no, s, startUnix);
  updateResults();
}

void clearRuns() {
  LittleFS.remove(CSV_FILE);
  LittleFS.remove(EVT_FILE);
  nRecent = 0; totalRuns = 0; bestTime = 0; nEvt = 0;
  for (int i = 0; i < MAX_QUEUE; i++) queue[i].valid = false;
  queueCount = 0;
  updateResults();
  lastDraw = 0;
}

// Queue-Funktionen
int findQueueSlot() {
  for (int i = 0; i < MAX_QUEUE; i++) if (!queue[i].valid) return i;
  return -1;
}

int findQueueByRunId(uint16_t runId) {
  for (int i = 0; i < MAX_QUEUE; i++) if (queue[i].valid && queue[i].runId == runId) return i;
  return -1;
}

int findQueueByRunNo(uint16_t rNo) {
  for (int i = 0; i < MAX_QUEUE; i++) if (queue[i].valid && queue[i].runNo == rNo) return i;
  return -1;
}

int findOldestInQueue() {
  int oldest = -1;
  int64_t oldestTime = INT64_MAX;
  for (int i = 0; i < MAX_QUEUE; i++) {
    if (queue[i].valid && queue[i].startUs < oldestTime) {
      oldestTime = queue[i].startUs;
      oldest = i;
    }
  }
  return oldest;
}

void countQueue() {
  queueCount = 0;
  for (int i = 0; i < MAX_QUEUE; i++) if (queue[i].valid) queueCount++;
}

void removeFromQueue(int idx) {
  if (idx >= 0 && idx < MAX_QUEUE) {
    queue[idx].valid = false;
    countQueue();
  }
}

void drawZiel() {
  char b[28], vs[8], vz[8];
  bool link = seenOnce && (millis() - lastSeenMs < 10000);
  
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, link ? "Start OK" : "Start AUS!");
  
  const char* state = millis() < clashUntil ? "ROLLE 2x!" : 
                      (millis() < noteUntil ? noteText : 
                      (queueCount > 0 ? "AUF STRECKE" : "BEREIT"));
  display.drawStr(128 - display.getStrWidth(state), 10, state);
  
  // Zeile 2: Fahrer auf Strecke + Akku
  snprintf(b, sizeof(b), "Fahrer: %d", queueCount);
  display.drawStr(0, 22, b);
  fmtV(vs, sizeof(vs), otherBattMv);
  fmtV(vz, sizeof(vz), battMv);
  snprintf(b, sizeof(b), "S%s Z%sV", vs, vz);
  display.drawStr(128 - display.getStrWidth(b), 22, b);
  
  // Zeilen 3-5: letzte 3 Zeiten
  for (int i = 0; i < 3; i++) {
    display.setFont(i == 0 ? u8g2_font_7x13B_tr : u8g2_font_7x13_tr);
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

void sendPkt(uint8_t type, uint16_t runId, uint8_t copy, uint32_t offsetUs, uint16_t assignedRunNo) {
  Pkt p;
  memset(&p, 0, sizeof(p));
  p.magic = MAGIC;
  p.type = type;
  p.runId = runId;
  p.copy = copy;
  p.offsetUs = offsetUs;
  p.battMv = battMv;
  p.runNo = assignedRunNo;
  p.resNo = resNo;
  memcpy(p.resMs, resMs, sizeof(resMs));
  p.queueLen = queueCount;
  radio.transmit((uint8_t*)&p, sizeof(p));
  radioFlag = false;
}

void handleRx() {
  int64_t rx = rxUs;
  Pkt p;
  if (radio.readData((uint8_t*)&p, sizeof(p)) != RADIOLIB_ERR_NONE || p.magic != MAGIC) return;
  if (p.type == T_PONG || p.type == T_ACK || p.type == T_RESULT) {
    clashUntil = millis() + 10000;
    return;
  }
  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();
  lastSeenMs = millis();
  seenOnce = true;
  startBatt = p.battMv;
  otherBattMv = p.battMv;

  if (p.type == T_PING) {
    sendPkt(T_PONG, 0, queueCount > 0 ? 1 : 0, 0, runNo);
  } else if (p.type == T_START) {
    if (findQueueByRunId(p.runId) < 0) {
      int slot = findQueueSlot();
      if (slot >= 0) {
        totalRuns++;
        queue[slot].runNo = totalRuns;
        queue[slot].runId = p.runId;
        int64_t airUs = (int64_t)radio.getTimeOnAir(sizeof(p));
        queue[slot].startUs = rx - airUs - (int64_t)p.offsetUs - TX_CAL_US;
        queue[slot].startUnix = toUnixS(queue[slot].startUs);
        queue[slot].valid = true;
        countQueue();
        runNo = totalRuns;
        lastDraw = 0;
        startFlash();
        Serial.printf("Start #%u empfangen (Queue: %d)\n", totalRuns, queueCount);
        sendPkt(T_ACK, p.runId, 0, 0, totalRuns);
      } else {
        logEvent("Queue voll", "Start abgelehnt", "QUEUE VOLL");
        sendPkt(T_ACK, p.runId, 0, 0, 0);
      }
    } else {
      int idx = findQueueByRunId(p.runId);
      sendPkt(T_ACK, p.runId, 0, 0, queue[idx].runNo);
    }
  }
}

// ---- Weboberfläche ----
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="de"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Zeitnahme</title>
<style>
:root{color-scheme:light dark}
body{font-family:system-ui,sans-serif;margin:0 auto;padding:16px;max-width:520px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.pill{font-size:13px;padding:4px 10px;border-radius:8px;background:#8883}
.ok{background:#2a7d3a33}.bad{background:#c0392b33}.warn{background:#f39c1233}
.big{font-size:3.2em;font-weight:500;text-align:center;margin:.2em 0}
.c{text-align:center;color:#888}
table{width:100%;border-collapse:collapse;margin:8px 0}
td,th{padding:6px 4px;border-bottom:1px solid #8883;text-align:left}
th{font-weight:500;font-size:13px}
td+td,th+th{text-align:right}
button,a.b,select{padding:8px 12px;font-size:14px;border:1px solid #8886;
border-radius:8px;background:none;color:inherit;text-decoration:none}
button:active{background:#8882}
.sm{padding:4px 8px;font-size:12px}
h3{font-size:15px;font-weight:500;margin:16px 0 8px}
.qitem{display:flex;align-items:center;gap:8px;padding:8px;border:1px solid #8883;border-radius:8px;margin:4px 0}
.qno{font-size:1.5em;font-weight:600;min-width:50px}
</style></head><body>
<h2 style="font-size:18px;font-weight:500">Zeitnahme</h2>
<div class="row"><span id="link" class="pill">...</span><span id="sig" class="pill"></span><span id="bat" class="pill"></span></div>
<div class="c" id="state">...</div>
<div class="big" id="last">-</div>
<div class="c" id="best"></div>

<h3>Auf der Strecke</h3>
<div id="queue"></div>

<h3>Letzte Ergebnisse</h3>
<table id="runs"><tr><td>Keine</td></tr></table>

<h3>Korrektur</h3>
<div class="row">
<select id="finishSel" style="flex:2"><option value="">Zieldurchfahrt wählen...</option></select>
<select id="runnerSel" style="flex:2"><option value="">Lauf# zuweisen...</option></select>
<button onclick="reassign()" class="sm">Zuweisen</button>
</div>

<h3>Auffälligkeiten</h3>
<table id="evts"><tr><td>keine</td></tr></table>

<div class="row"><a class="b" href="/export.csv" download>CSV</a>
<a class="b" href="/events.csv" download>Ereignisse</a>
<button onclick="clr()">Löschen</button></div>
<div class="row"><a class="b" href="/ota">Firmware Update</a></div>
<div class="c" style="margin-top:8px;font-size:12px">Version )HTML" FIRMWARE_VERSION R"HTML(</div>

<script>
const f=x=>x.toFixed(3).replace('.',',')+' s';
let tsent=0,gd={};
function sync(d){
 if(!d.timeSet||Date.now()-tsent>60000){
  tsent=Date.now();
  fetch('/settime?ms='+Date.now()+'&tz='+(-new Date().getTimezoneOffset()));
 }
}
async function upd(){
 try{
  const d=await (await fetch('/api')).json();
  gd=d;
  sync(d);
  const l=document.getElementById('link');
  if(d.age<0){l.textContent='Start nicht gesehen';l.className='pill bad';}
  else if(d.link){l.textContent='Start verbunden';l.className='pill ok';}
  else{l.textContent='Start offline '+d.age+'s';l.className='pill bad';}
  sig.textContent=d.age<0?'':'RSSI '+Math.round(d.rssi)+' · SNR '+d.snr.toFixed(0);
  bat.textContent=[d.batt>1999?'S '+(d.batt/1000).toFixed(2)+'V':'',d.zbatt>1999?'Z '+(d.zbatt/1000).toFixed(2)+'V':''].filter(x=>x).join(' ');
  state.textContent=d.queue.length?d.queue.length+' Fahrer auf Strecke':'Bereit';
  last.textContent=d.runs.length?f(d.runs[0].s):'-';
  best.textContent=d.best>0?'Bestzeit '+f(d.best):'';
  
  // Queue (Fahrer auf Strecke)
  queue.innerHTML=d.queue.length?d.queue.map(q=>
    '<div class="qitem"><span class="qno">#'+q.n+'</span><span>Start: '+q.t+'</span>'+
    '<button class="sm" onclick="dnf('+q.n+')">DNF</button></div>'
  ).join(''):'<div class="c" style="padding:16px">Keine Fahrer auf der Strecke</div>';
  
  // Ergebnisse
  runs.innerHTML=d.runs.length?d.runs.map(r=>'<tr><td>#'+r.n+'</td><td>'+r.t+'</td><td>'+f(r.s)+'</td></tr>').join(''):'<tr><td>Keine</td></tr>';
  
  // Korrektur-Dropdowns
  finishSel.innerHTML='<option value="">Zieldurchfahrt...</option>'+d.runs.map(r=>'<option value="'+r.n+'">#'+r.n+' ('+f(r.s)+')</option>').join('');
  runnerSel.innerHTML='<option value="">Lauf# zuweisen...</option>'+d.queue.map(q=>'<option value="'+q.n+'">#'+q.n+'</option>').join('');
  
  evts.innerHTML=d.events.length?d.events.map(e=>'<tr><td>'+e.t+'</td><td>'+e.e+'</td></tr>').join(''):'<tr><td>keine</td></tr>';
 }catch(e){document.getElementById('link').textContent='Keine Verbindung';document.getElementById('link').className='pill bad';}
}
async function dnf(n){if(confirm('Lauf #'+n+' als Ausfall (DNF) markieren?')){await fetch('/dnf?n='+n);upd();}}
async function reassign(){
 const fn=finishSel.value,rn=runnerSel.value;
 if(!fn||!rn){alert('Bitte beide Felder auswählen');return;}
 if(confirm('Zieldurchfahrt #'+fn+' dem Lauf #'+rn+' zuweisen?')){
  await fetch('/reassign?finish='+fn+'&runner='+rn);upd();
 }
}
async function clr(){if(confirm('Alle Daten löschen?')){await fetch('/clear');upd();}}
upd();setInterval(upd,1500);
</script></body></html>)HTML";

void handleApi() {
  bool link = seenOnce && (millis() - lastSeenMs < 10000);
  int age = seenOnce ? (int)((millis() - lastSeenMs) / 1000) : -1;
  String j = "{\"link\":" + String(link ? "true" : "false");
  j += ",\"age\":" + String(age);
  j += ",\"rssi\":" + String(lastRssi, 0);
  j += ",\"snr\":" + String(lastSnr, 1);
  j += ",\"batt\":" + String(startBatt);
  j += ",\"zbatt\":" + String(battMv);
  j += ",\"timeSet\":" + String(timeSet ? "true" : "false");
  j += ",\"total\":" + String(totalRuns);
  j += ",\"best\":" + String(bestTime, 3);
  
  // Queue (Fahrer auf Strecke)
  j += ",\"queue\":[";
  bool first = true;
  for (int i = 0; i < MAX_QUEUE; i++) {
    if (queue[i].valid) {
      if (!first) j += ",";
      first = false;
      j += "{\"n\":" + String(queue[i].runNo) + ",\"t\":\"" + fmtLocal(queue[i].startUnix, false) + "\"}";
    }
  }
  
  j += "],\"runs\":[";
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

void handleDnf() {
  int n = server.arg("n").toInt();
  int idx = findQueueByRunNo(n);
  if (idx >= 0) {
    removeFromQueue(idx);
    logEvent("DNF", "Lauf #" + String(n), "DNF");
  }
  server.send(200, "text/plain", "ok");
}

void handleReassign() {
  int finishNo = server.arg("finish").toInt();
  int runnerNo = server.arg("runner").toInt();
  
  int qIdx = findQueueByRunNo(runnerNo);
  if (qIdx < 0) {
    server.send(400, "text/plain", "Lauf nicht in Queue");
    return;
  }
  
  // Finde die Zielzeit in recent und ändere die Laufnummer
  for (int i = 0; i < nRecent; i++) {
    if (recent[i].no == finishNo) {
      float oldTime = recent[i].s;
      uint32_t oldStartUnix = recent[i].startUnix;
      
      // Berechne neue Zeit basierend auf dem tatsächlichen Start
      int64_t finishTime = queue[qIdx].startUs + (int64_t)(oldTime * 1000000.0f);
      float newTime = (finishTime - queue[qIdx].startUs) / 1000000.0f;
      
      recent[i].no = runnerNo;
      recent[i].startUnix = queue[qIdx].startUnix;
      
      removeFromQueue(qIdx);
      logEvent("Korrektur", "#" + String(finishNo) + " -> #" + String(runnerNo), "KORRIGIERT");
      updateResults();
      break;
    }
  }
  server.send(200, "text/plain", "ok");
}

// ---- OTA Firmware Update ----
const char OTA_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="de"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Firmware Update</title>
<style>
body{font-family:system-ui,sans-serif;margin:0 auto;padding:16px;max-width:400px}
h2{font-size:18px;font-weight:500}
.info{background:#8882;padding:12px;border-radius:8px;margin:12px 0}
input[type=file]{margin:12px 0}
button{width:100%;padding:12px;font-size:16px;border:none;border-radius:8px;background:#2a7d3a;color:#fff;cursor:pointer}
button:disabled{background:#888}
#prog{width:100%;height:20px;margin:12px 0;display:none}
#msg{margin:12px 0;padding:12px;border-radius:8px}
.ok{background:#2a7d3a33}.err{background:#c0392b33}
</style></head><body>
<h2>Firmware Update</h2>
<div class="info">
<strong>Aktuelle Version:</strong> )HTML" FIRMWARE_VERSION R"HTML(<br>
<strong>Rolle:</strong> ZIEL-Gate
</div>
<form id="form">
<input type="file" id="file" accept=".bin">
<progress id="prog" value="0" max="100"></progress>
<div id="msg"></div>
<button type="submit" id="btn">Update starten</button>
</form>
<script>
const form=document.getElementById('form'),file=document.getElementById('file'),
      prog=document.getElementById('prog'),msg=document.getElementById('msg'),btn=document.getElementById('btn');
form.onsubmit=async e=>{
  e.preventDefault();
  if(!file.files.length){msg.textContent='Bitte .bin Datei wählen';msg.className='err';return;}
  btn.disabled=true;prog.style.display='block';msg.textContent='';
  const data=new FormData();data.append('update',file.files[0]);
  const xhr=new XMLHttpRequest();
  xhr.open('POST','/update');
  xhr.upload.onprogress=e=>{if(e.lengthComputable)prog.value=Math.round(e.loaded/e.total*100);};
  xhr.onload=()=>{
    if(xhr.status===200){msg.textContent='Update erfolgreich! Neustart...';msg.className='ok';setTimeout(()=>location.reload(),5000);}
    else{msg.textContent='Fehler: '+xhr.responseText;msg.className='err';btn.disabled=false;}
  };
  xhr.onerror=()=>{msg.textContent='Upload fehlgeschlagen';msg.className='err';btn.disabled=false;};
  xhr.send(data);
};
</script></body></html>)HTML";

void handleOtaPage() {
  server.send_P(200, "text/html", OTA_HTML);
}

void handleOtaUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA Update: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA Update erfolgreich: %u Bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

void handleOtaResult() {
  if (Update.hasError()) {
    server.send(500, "text/plain", "Update fehlgeschlagen");
  } else {
    server.send(200, "text/plain", "OK");
    delay(500);
    ESP.restart();
  }
}

void setupZiel() {
  pinMode(ZIEL_PIN, INPUT_PULLUP);
  initDisplay();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 32, "Starte ...");
  display.sendBuffer();
  
  if (!LittleFS.begin(true)) Serial.println("LittleFS Fehler");
  loadRuns();
  for (int i = 0; i < MAX_QUEUE; i++) queue[i].valid = false;
  queueCount = 0;
  
  logEvent("Neustart", "", "NEUSTART");
  battMv = readBattMv();
  if (!initRadio()) { Serial.println("LoRa Fehler"); while (true) delay(1000); }
  radio.startReceive();
  attachInterrupt(digitalPinToInterrupt(ZIEL_PIN), onFinish, RISING);

  WiFi.softAP("Zeitnahme", "12345678");
  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api", handleApi);
  server.on("/dnf", handleDnf);
  server.on("/reassign", handleReassign);
  server.on("/ota", handleOtaPage);
  server.on("/update", HTTP_POST, handleOtaResult, handleOtaUpload);
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
  server.on("/settime", []() {
    int64_t ms = strtoll(server.arg("ms").c_str(), nullptr, 10);
    int tz = server.arg("tz").toInt();
    if (ms > 1700000000000LL) {
      int64_t newOff = ms * 1000 - esp_timer_get_time();
      if (!timeSet || llabs(newOff - epochOffsetUs) > 2000000) {
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
    rxFlag = false;
    radio.startReceive();
  }
  if (finishFlag) {
    finishFlag = false;
    int64_t fu = finishUs;
    
    int oldest = findOldestInQueue();
    if (oldest >= 0) {
      int64_t dt = fu - queue[oldest].startUs;
      if (dt > 500000) {
        float s = dt / 1000000.0f;
        uint16_t finishedRunNo = queue[oldest].runNo;
        uint32_t startUnix = queue[oldest].startUnix;
        removeFromQueue(oldest);
        appendRun(finishedRunNo, s, startUnix, toUnixS(fu));
        sendPkt(T_RESULT, 0, 0, 0, finishedRunNo);
        rxFlag = false;
        radio.startReceive();
        startFlash();                            // Display blinkt bei Zieleinlauf
        lastDraw = 0;
        Serial.printf("Ziel #%u: %.3f s (Queue: %d)\n", finishedRunNo, s, queueCount);
      } else {
        logEvent("Zielimpuls zu kurz", String(dt / 1000000.0f, 3) + " s", "ZU KURZ");
      }
    } else {
      logEvent("Zielimpuls ohne Start", "", "OHNE START");
    }
  }

  // Timeout für Fahrer auf Strecke
  int64_t now = esp_timer_get_time();
  for (int i = 0; i < MAX_QUEUE; i++) {
    if (queue[i].valid && (now - queue[i].startUs) > (int64_t)RUN_TIMEOUT_S * 1000000LL) {
      logEvent("Timeout", "Lauf #" + String(queue[i].runNo), "TIMEOUT");
      removeFromQueue(i);
    }
  }
  
  if (millis() - lastBatt > 30000) { lastBatt = millis(); battMv = readBattMv(); }
  if (millis() - lastDraw > 500) { lastDraw = millis(); drawZiel(); }
}

// =====================================================================
// Rollenwahl, Taste und Deep Sleep
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
  digitalWrite(VEXT_PIN, HIGH);
  gpio_hold_en((gpio_num_t)VEXT_PIN);
  gpio_deep_sleep_hold_en();
  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
  esp_deep_sleep_start();
}

void switchRole() {
  prefs.putBool("start", !isStart);
  showBig("Neue Rolle:", isStart ? "ZIEL" : "START");
  delay(1200);
  ESP.restart();
}

void buttonCheck() {
  static uint32_t t0 = 0;
  static bool down = false;
  static uint8_t phase = 0;
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
    if (phase) lastDraw = millis();
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
  while (digitalRead(BUTTON_PIN) == LOW && millis() - t < 3000) delay(10);
  delay(1000);
  if (isStart) setupStart(); else setupZiel();
}

void loop() {
  buttonCheck();
  if (isStart) loopStart(); else loopZiel();
}
