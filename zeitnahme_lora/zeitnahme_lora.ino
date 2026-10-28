// Zeitnahme Start/Ziel per LoRa + WLAN-Webserver am Ziel  (v4.3 - Multi-Fahrer)
// Board: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
// Bibliotheken: RadioLib, U8g2 (beide über den Bibliotheksverwalter)
//
// Neu in v4.3 (Zuordnung Start <-> Ziel komplett überarbeitet):
//  - Starts und Zielimpulse haben getrennte Nummern: S#1, S#2 ... und Z#1, Z#2 ...
//    Ein Zielimpuls ohne Start zählt Z# hoch, die Startnummern bleiben unberührt.
//  - Ein Lauf ist ein Paar aus Start (S#) und Ziel (Z#). Die Laufnummer ist die Startnummer.
//  - Verknüpfen: Start #2 mit Ziel Z3 verknüpfen. Die bisherigen Partner (Z2 und Start #3)
//    werden frei und können danach beliebig neu zugewiesen werden.
//  - Alle Ereignisse stehen in einem Journal im Flash (nur anhängen, nichts wird überschrieben).
//    Zuordnungen und Korrekturen überstehen einen Neustart. Das CSV wird daraus erzeugt.
//  - Korrektur-Auswahl auf der Webseite behält ihre Auswahl beim Aktualisieren.
//  - Start-Display zeigt die letzten 3 Zielimpulse, Zielimpulse ohne Start als "Zn kein Start".
//  - Info-Seite bei kurzem PRG-Tastendruck (Akku %, RSSI/SNR, Verbindungsstatus)
//
// Auf BEIDE Boards dieselbe Datei flashen. Die Rolle (START oder ZIEL) steht im Flash.
// PRG-Taste: kurz drücken = Info-Seite (10s),
//            2 s halten und loslassen = Rolle wechseln (Neustart),
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
#include <stdarg.h>
#include "driver/gpio.h"

#define FIRMWARE_VERSION "4.3"

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
#define START_PIN 5    // Schalter (INPUT_PULLUP)
#define ZIEL_PIN  5    // Schalter (INPUT_PULLUP)
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
#define T_PONG   2   // Ziel -> Start (copy Bit0 = Fahrer auf Strecke, plus letzte Ergebnisse)
#define T_START  3   // Start -> Ziel, Lauf gestartet
#define T_ACK    4   // Ziel -> Start, Start angekommen (runNo = zugewiesene Startnummer)
#define T_RESULT 5   // Ziel -> Start, Zielimpuls (plus letzte Ergebnisse)

struct __attribute__((packed)) Pkt {
  uint8_t  magic;
  uint8_t  type;
  uint16_t runId;       // START/ACK: zufällige Lauf-ID
  uint8_t  copy;        // START: Kopie 0..7; PONG: Bit0 = Fahrer auf Strecke
  uint32_t offsetUs;    // START: Zeit von Auslösung bis Sendebeginn
  uint16_t battMv;      // Akkuspannung des Senders (0 = unbekannt)
  uint16_t runNo;       // ACK: zugewiesene Startnummer
  uint16_t resNos[3];   // Ziel: letzte 3 Zielimpulse: Startnummer, oder 0x8000|Z# ohne Start
  uint32_t resMs[3];    // Ziel: deren Zeiten in ms, neueste zuerst (0 = leer/ohne Start)
  uint8_t  queueLen;    // Ziel: Anzahl Fahrer auf der Strecke
};

// Feste Verzögerung zwischen transmit()-Aufruf und echtem Sendebeginn (µs).
#define TX_CAL_US 1500

uint16_t battMv = 0;
uint16_t runNo = 0;                 // Start: zuletzt zugewiesene Startnummer
uint16_t resNos[3] = {0, 0, 0};     // letzte 3 Zielimpulse (siehe Pkt)
uint32_t resMs[3] = {0, 0, 0};      // deren Zeiten in ms
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

// Info-Seite: Akku, Empfang etc. (wird bei kurzem PRG-Klick angezeigt)
uint32_t infoPageUntil = 0;
#define INFO_PAGE_MS 10000
float globalRssi = 0, globalSnr = 0;

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

// Akkuspannung in Prozent umrechnen (3.0V = 0%, 4.0V = 100%, linear)
uint8_t battPercent(uint16_t mv) {
  if (mv < 3000) return 0;
  if (mv > 4000) return 100;
  return (uint8_t)((mv - 3000) * 100 / 1000);
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

// Info-Seite: Akku, Empfang etc. (wird bei einfachem Klick angezeigt)
void drawInfoPage(uint16_t sMv, uint16_t zMv, bool link) {
  char b[32];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  
  display.drawStr(0, 10, "System-Info");
  
  uint8_t sP = battPercent(sMv);
  if (sMv >= 2000) snprintf(b, sizeof(b), "Start:  %3u%% (%u.%02uV)", sP, sMv/1000, (sMv%1000)/10);
  else snprintf(b, sizeof(b), "Start:  --");
  display.drawStr(0, 24, b);
  
  uint8_t zP = battPercent(zMv);
  if (zMv >= 2000) snprintf(b, sizeof(b), "Ziel:   %3u%% (%u.%02uV)", zP, zMv/1000, (zMv%1000)/10);
  else snprintf(b, sizeof(b), "Ziel:   --");
  display.drawStr(0, 36, b);
  
  snprintf(b, sizeof(b), "Funk:   %s", link ? "OK" : "KEINE");
  display.drawStr(0, 48, b);
  
  if (link && globalRssi != 0) {
    snprintf(b, sizeof(b), "RSSI:%ddBm SNR:%.0fdB", (int)globalRssi, globalSnr);
    display.drawStr(0, 60, b);
  }
  
  display.sendBuffer();
}

// Eine Ergebniszeile für das Display: "#3 74.382 s" oder "Z4 kein Start"
void fmtResLine(char* b, size_t n, int i) {
  if (resNos[i] & 0x8000) {
    snprintf(b, n, "Z%u kein Start", (unsigned)(resNos[i] & 0x7FFF));
  } else if (resMs[i] > 0) {
    snprintf(b, n, "#%u %lu.%03lu s", (unsigned)resNos[i],
             (unsigned long)(resMs[i] / 1000), (unsigned long)(resMs[i] % 1000));
  } else {
    snprintf(b, n, "#- --.--- s");
  }
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
uint16_t lastAssignedRunNo = 0;     // zuletzt zugewiesene Startnummer (für große Anzeige)
uint32_t showRunNoUntil = 0;        // Startnummer groß anzeigen bis zu diesem Zeitpunkt

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
  memcpy(resNos, r.resNos, sizeof(resNos));
  memcpy(resMs, r.resMs, sizeof(resMs));
  queueLen = r.queueLen;
  globalRssi = radio.getRSSI();
  globalSnr = radio.getSNR();
}

// Start-Display: zeigt die zugewiesene Startnummer groß an
void drawStart() {
  char b[28], vs[8], vz[8];
  bool link = lastPong && (millis() - lastPong < 10000);

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, link ? "Ziel OK" : "Ziel AUS!");

  const char* state = millis() < clashUntil ? "ROLLE 2x!" : (noAck ? "KEIN ACK" : "BEREIT");
  display.drawStr(128 - display.getStrWidth(state), 10, state);

  // Zeile 2: Fahrer auf Strecke + Akku in Prozent
  snprintf(b, sizeof(b), "Auf Strecke: %u", queueLen);
  display.drawStr(0, 22, b);
  snprintf(b, sizeof(b), "S%u%% Z%u%%", battPercent(battMv), battPercent(otherBattMv));
  display.drawStr(128 - display.getStrWidth(b), 22, b);

  if (millis() < showRunNoUntil && lastAssignedRunNo > 0) {
    display.setFont(u8g2_font_logisoso24_tr);
    snprintf(b, sizeof(b), "#%u", lastAssignedRunNo);
    display.drawStr((128 - display.getStrWidth(b)) / 2, 52, b);
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0, 64, "DEINE NUMMER");
  } else {
    for (int i = 0; i < 3; i++) {
      display.setFont(i == 0 ? u8g2_font_7x13B_tr : u8g2_font_7x13_tr);
      fmtResLine(b, sizeof(b), i);
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
  Serial.printf("Start Lauf-ID %u: %s, Startnummer %u\n", run, acked ? "bestaetigt" : "KEINE Bestaetigung", lastAssignedRunNo);
  noAck = !acked;
  if (acked) {
    showRunNoUntil = millis() + 5000;   // Startnummer 5 Sekunden groß anzeigen
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

  if (millis() - lastDraw > 200) {
    lastDraw = millis();
    if (millis() < infoPageUntil) {
      drawInfoPage(battMv, otherBattMv, link);
    } else {
      drawStart();
    }
  }
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

uint32_t lastSeenMs = 0;
bool seenOnce = false;
float lastRssi = 0, lastSnr = 0;
uint16_t startBatt = 0;

// Uhrzeit (wird vom Handy über die Weboberfläche gesetzt)
bool timeSet = false;
int64_t epochOffsetUs = 0;     // Unix-Zeit in µs = esp_timer_get_time() + epochOffsetUs
int tzMin = 0;                 // Zeitzone des Handys in Minuten gegenüber UTC

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

// Dateien
#define JOURNAL_FILE "/journal.csv"
#define EVT_FILE "/events.csv"
#define OLD_CSV_FILE "/runs.csv"      // aus v4.2, wird beim Löschen mit entfernt
#define MAX_EVT 10
#define RUN_TIMEOUT_S 1800            // Fahrer ohne Zieldurchfahrt gelten nach 30 min als Ausfall

struct Evt { uint32_t unixS; char text[48]; };
Evt evts[MAX_EVT];
int nEvt = 0;
const char* noteText = "";
uint32_t noteUntil = 0;

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

// ---------------------------------------------------------------------
// MODELL: Starts (S#) und Zielimpulse (Z#) mit freier Verknüpfung
// ---------------------------------------------------------------------
void journal(const char* fmt, ...);   // schreibt eine Zeile ins Journal (Definition weiter unten)

#define MAX_EV 300
struct StartEv  { uint16_t no; uint16_t runId; uint32_t boot; int64_t us; uint16_t finishNo; bool dnf; };
struct FinishEv { uint16_t no; uint32_t boot; int64_t us; uint16_t startNo; bool ign; };
StartEv  starts[MAX_EV];     // Ring: Platz (no-1) % MAX_EV, gültig wenn .no == no
FinishEv fins[MAX_EV];
uint16_t totalStarts = 0, totalFinishes = 0;
uint32_t bootNo = 0;         // zählt die Neustarts des Ziels
int queueCount = 0;          // Fahrer auf der Strecke (offene Starts)
float bestTime = 0;

// Pro Neustart die Zuordnung Boot-Zeit -> Uhrzeit (Offset), damit Zeiten auch später noch eine Uhrzeit haben
struct BootInfo { uint32_t boot; int64_t offUs; };
BootInfo bootInfo[8];
int nBootInfo = 0;

void setBootOffset(uint32_t boot, int64_t offUs) {
  for (int i = 0; i < nBootInfo; i++) if (bootInfo[i].boot == boot) { bootInfo[i].offUs = offUs; return; }
  if (nBootInfo == 8) { for (int i = 1; i < 8; i++) bootInfo[i - 1] = bootInfo[i]; nBootInfo = 7; }
  bootInfo[nBootInfo++] = { boot, offUs };
}

bool unixUsOf(uint32_t boot, int64_t us, int64_t& out) {
  for (int i = 0; i < nBootInfo; i++) {
    if (bootInfo[i].boot == boot) { out = us + bootInfo[i].offUs; return out > 0; }
  }
  return false;
}

uint32_t unixSOf(uint32_t boot, int64_t us) {   // 0 = unbekannt
  int64_t o;
  return unixUsOf(boot, us, o) ? (uint32_t)(o / 1000000) : 0;
}

StartEv* getStart(uint16_t no) {
  if (!no) return nullptr;
  StartEv* s = &starts[(no - 1) % MAX_EV];
  return s->no == no ? s : nullptr;
}

FinishEv* getFin(uint16_t no) {
  if (!no) return nullptr;
  FinishEv* f = &fins[(no - 1) % MAX_EV];
  return f->no == no ? f : nullptr;
}

void addStart(uint16_t no, uint16_t runId, uint32_t boot, int64_t us) {
  starts[(no - 1) % MAX_EV] = { no, runId, boot, us, 0, false };
  if (no > totalStarts) totalStarts = no;
}

void addFin(uint16_t no, uint32_t boot, int64_t us) {
  fins[(no - 1) % MAX_EV] = { no, boot, us, 0, false };
  if (no > totalFinishes) totalFinishes = no;
}

// Start sNo und Ziel zNo verknüpfen. Die bisherigen Partner beider Seiten werden frei.
bool linkPair(uint16_t sNo, uint16_t zNo) {
  StartEv* s = getStart(sNo);
  FinishEv* z = getFin(zNo);
  if (!s || !z) return false;
  if (s->finishNo) { FinishEv* o = getFin(s->finishNo); if (o) o->startNo = 0; }
  if (z->startNo)  { StartEv* o = getStart(z->startNo); if (o) o->finishNo = 0; }
  s->finishNo = zNo;
  z->startNo = sNo;
  s->dnf = false;
  z->ign = false;
  return true;
}

void unlinkStart(uint16_t sNo) {
  StartEv* s = getStart(sNo);
  if (!s) return;
  if (s->finishNo) { FinishEv* f = getFin(s->finishNo); if (f) f->startNo = 0; }
  s->finishNo = 0;
}

// Laufzeit eines Paares. Im selben Neustart auf µs genau, sonst über die Uhrzeit (ungenauer).
bool pairSec(const StartEv& s, const FinishEv& f, float& sec) {
  if (s.boot == f.boot) { sec = (float)((double)(f.us - s.us) / 1e6); return true; }
  int64_t a, b;
  if (unixUsOf(s.boot, s.us, a) && unixUsOf(f.boot, f.us, b)) { sec = (float)((double)(b - a) / 1e6); return true; }
  return false;
}

// Ein Start ist "auf der Strecke", wenn er noch kein Ziel hat, kein DNF ist und aus diesem Neustart stammt
bool isOpen(const StartEv& s) { return s.no && !s.finishNo && !s.dnf && s.boot == bootNo; }

void countQueue() {
  queueCount = 0;
  for (int i = 0; i < MAX_EV; i++) if (isOpen(starts[i])) queueCount++;
}

StartEv* oldestOpenStart() {
  StartEv* best = nullptr;
  for (int i = 0; i < MAX_EV; i++) {
    if (!isOpen(starts[i])) continue;
    if (!best || starts[i].us < best->us) best = &starts[i];
  }
  return best;
}

StartEv* findStartByRunId(uint16_t runId) {
  for (int n = totalStarts, c = 0; n >= 1 && c < 30; n--, c++) {
    StartEv* s = getStart((uint16_t)n);
    if (s && s->boot == bootNo && s->runId == runId) return s;
  }
  return nullptr;
}

void recomputeBest() {
  bestTime = 0;
  for (int i = 0; i < MAX_EV; i++) {
    const StartEv& s = starts[i];
    if (!s.no || !s.finishNo) continue;
    FinishEv* f = getFin(s.finishNo);
    float sec;
    if (f && !f->ign && pairSec(s, *f, sec) && sec > 0 && (bestTime == 0 || sec < bestTime)) bestTime = sec;
  }
}

// Letzte 3 Zielimpulse für Funk und Display (neueste zuerst, verworfene übersprungen)
void updateResults() {
  int k = 0;
  for (int z = totalFinishes; z >= 1 && k < 3 && (int)totalFinishes - z < MAX_EV; z--) {
    FinishEv* f = getFin(z);
    if (!f || f->ign) continue;
    resNos[k] = 0x8000 | (uint16_t)z;
    resMs[k] = 0;
    StartEv* s = f->startNo ? getStart(f->startNo) : nullptr;
    float sec;
    if (s && pairSec(*s, *f, sec) && sec > 0) {
      resNos[k] = s->no;
      resMs[k] = (uint32_t)(sec * 1000.0f + 0.5f);
    }
    k++;
  }
  for (; k < 3; k++) { resNos[k] = 0; resMs[k] = 0; }
  runNo = totalStarts + 1;     // nächste Startnummer
  recomputeBest();
  countQueue();
}

// ---- Aktionen (schreiben zusätzlich ins Journal) ----
uint16_t actStart(uint16_t runId, int64_t us) {
  uint16_t no = totalStarts + 1;
  addStart(no, runId, bootNo, us);
  journal("S;%u;%u;%lld", (unsigned)no, (unsigned)bootNo, (long long)us);
  countQueue();
  return no;
}

// Rückgabe: 0 = zu kurz (ignoriert), 1 = Lauf zugeordnet (FIFO), 2 = Zielimpuls ohne Start
int actFinish(int64_t fu, uint16_t& zNo, uint16_t& sNo) {
  StartEv* o = oldestOpenStart();
  if (o && fu - o->us <= 500000) return 0;      // frischer Start < 0,5 s: Fehlauslösung
  zNo = totalFinishes + 1;
  addFin(zNo, bootNo, fu);
  journal("F;%u;%u;%lld", (unsigned)zNo, (unsigned)bootNo, (long long)fu);
  sNo = 0;
  if (o) {
    sNo = o->no;
    linkPair(sNo, zNo);
    journal("P;%u;%u", (unsigned)sNo, (unsigned)zNo);
  }
  updateResults();
  return o ? 1 : 2;
}

bool canLink(uint16_t sNo, uint16_t zNo, float& sec, const char*& err) {
  StartEv* s = getStart(sNo);
  FinishEv* z = getFin(zNo);
  if (!s) { err = "Start nicht gefunden"; return false; }
  if (!z) { err = "Ziel nicht gefunden"; return false; }
  if (!pairSec(*s, *z, sec)) { err = "Zeit nicht berechenbar (anderer Neustart ohne Uhrzeit)"; return false; }
  if (sec <= 0.5f) { err = "Ziel liegt vor dem Start (oder unter 0,5 s)"; return false; }
  return true;
}

bool actLink(uint16_t sNo, uint16_t zNo) {
  if (!linkPair(sNo, zNo)) return false;
  journal("P;%u;%u", (unsigned)sNo, (unsigned)zNo);
  updateResults();
  return true;
}

void actUnlink(uint16_t sNo) {
  unlinkStart(sNo);
  journal("U;%u", (unsigned)sNo);
  updateResults();
}

bool actDnf(uint16_t sNo) {
  StartEv* s = getStart(sNo);
  if (!s || s->finishNo) return false;
  s->dnf = true;
  journal("D;%u", (unsigned)sNo);
  updateResults();
  return true;
}

bool actIgnore(uint16_t zNo) {
  FinishEv* f = getFin(zNo);
  if (!f || f->startNo) return false;
  f->ign = true;
  journal("X;%u", (unsigned)zNo);
  updateResults();
  return true;
}

void actBootOffset(uint32_t boot, int64_t offUs) {
  setBootOffset(boot, offUs);
  journal("T;%u;%lld", (unsigned)boot, (long long)offUs);
}

// Eine Journalzeile wieder einspielen (ohne erneut zu schreiben)
void applyJournalLine(const char* l) {
  if (!l[0] || l[1] != ';') return;
  const char* p = l + 2;
  unsigned a = 0, b = 0;
  long long c = 0;
  switch (l[0]) {
    case 'S': if (sscanf(p, "%u;%u;%lld", &a, &b, &c) == 3) addStart((uint16_t)a, 0, b, c); break;
    case 'F': if (sscanf(p, "%u;%u;%lld", &a, &b, &c) == 3) addFin((uint16_t)a, b, c); break;
    case 'P': if (sscanf(p, "%u;%u", &a, &b) == 2) linkPair((uint16_t)a, (uint16_t)b); break;
    case 'U': if (sscanf(p, "%u", &a) == 1) unlinkStart((uint16_t)a); break;
    case 'D': if (sscanf(p, "%u", &a) == 1) { StartEv* s = getStart((uint16_t)a); if (s && !s->finishNo) s->dnf = true; } break;
    case 'X': if (sscanf(p, "%u", &a) == 1) { FinishEv* f = getFin((uint16_t)a); if (f && !f->startNo) f->ign = true; } break;
    case 'T': if (sscanf(p, "%u;%lld", &b, &c) == 2) setBootOffset(b, c); break;
  }
}

void journal(const char* fmt, ...) {
  char b[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  File f = LittleFS.open(JOURNAL_FILE, "a");
  if (f) { f.println(b); f.close(); }
}

void loadJournal() {
  File f = LittleFS.open(JOURNAL_FILE, "r");
  if (f) {
    while (f.available()) {
      String l = f.readStringUntil('\n');
      l.trim();
      applyJournalLine(l.c_str());
    }
    f.close();
  }
  updateResults();
}

void clearRuns() {
  LittleFS.remove(JOURNAL_FILE);
  LittleFS.remove(EVT_FILE);
  LittleFS.remove(OLD_CSV_FILE);
  memset(starts, 0, sizeof(starts));
  memset(fins, 0, sizeof(fins));
  totalStarts = 0; totalFinishes = 0; nEvt = 0; bestTime = 0; nBootInfo = 0;
  if (timeSet) actBootOffset(bootNo, epochOffsetUs);
  updateResults();
  lastDraw = 0;
}

// Zeitangabe für die Anzeige: Uhrzeit, sonst Minuten:Sekunden seit dem Einschalten
String tLabel(uint32_t boot, int64_t us) {
  uint32_t u = unixSOf(boot, us);
  if (u) return fmtLocal(u, false);
  if (boot == bootNo) {
    uint32_t s = (uint32_t)(us / 1000000);
    char b[16];
    snprintf(b, sizeof(b), "+%u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
    return String(b);
  }
  return "?";
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

  // Zeile 2: Fahrer auf Strecke + Akku in Prozent
  snprintf(b, sizeof(b), "Fahrer: %d", queueCount);
  display.drawStr(0, 22, b);
  snprintf(b, sizeof(b), "S%u%% Z%u%%", battPercent(otherBattMv), battPercent(battMv));
  display.drawStr(128 - display.getStrWidth(b), 22, b);

  for (int i = 0; i < 3; i++) {
    display.setFont(i == 0 ? u8g2_font_7x13B_tr : u8g2_font_7x13_tr);
    fmtResLine(b, sizeof(b), i);
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
  memcpy(p.resNos, resNos, sizeof(resNos));
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
    clashUntil = millis() + 10000;     // das senden nur Ziele: zweites Ziel in Reichweite
    return;
  }
  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();
  globalRssi = lastRssi;
  globalSnr = lastSnr;
  lastSeenMs = millis();
  seenOnce = true;
  startBatt = p.battMv;
  otherBattMv = p.battMv;

  if (p.type == T_PING) {
    sendPkt(T_PONG, 0, queueCount > 0 ? 1 : 0, 0, runNo);
  } else if (p.type == T_START) {
    uint16_t no;
    StartEv* ex = findStartByRunId(p.runId);
    if (ex) {
      no = ex->no;                     // weitere Kopie desselben Starts
    } else {
      int64_t airUs = (int64_t)radio.getTimeOnAir(sizeof(p));
      int64_t su = rx - airUs - (int64_t)p.offsetUs - TX_CAL_US;
      no = actStart(p.runId, su);
      updateResults();
      lastDraw = 0;
      startFlash();
      Serial.printf("Start S#%u empfangen (auf Strecke: %d)\n", no, queueCount);
    }
    sendPkt(T_ACK, p.runId, 0, 0, no);
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
.ok{background:#2a7d3a33}.bad{background:#c0392b33}
.big{font-size:3.2em;font-weight:500;text-align:center;margin:.2em 0}
.c{text-align:center;color:#888}
table{width:100%;border-collapse:collapse;margin:8px 0}
td{padding:6px 4px;border-bottom:1px solid #8883;text-align:left}
td+td{text-align:right}
button,a.b,select{padding:8px 12px;font-size:14px;border:1px solid #8886;
border-radius:8px;background:none;color:inherit;text-decoration:none}
button:active{background:#8882}
.sm{padding:4px 8px;font-size:12px}
h3{font-size:15px;font-weight:500;margin:16px 0 8px}
.qitem{display:flex;align-items:center;gap:8px;padding:8px;border:1px solid #8883;border-radius:8px;margin:4px 0}
.qno{font-size:1.4em;font-weight:600;min-width:50px}
.qitem span+span{flex:1}
select{flex:1;min-width:0}
</style></head><body>
<h2 style="font-size:18px;font-weight:500">Zeitnahme</h2>
<div class="row"><span id="link" class="pill">...</span><span id="sig" class="pill"></span><span id="bat" class="pill"></span></div>
<div class="c" id="state">...</div>
<div class="big" id="last">-</div>
<div class="c" id="best"></div>

<h3>Auf der Strecke</h3>
<div id="queue"></div>

<h3>Ergebnisse</h3>
<table id="runs"><tr><td>Keine</td></tr></table>

<h3>Zielimpulse ohne Start</h3>
<div id="free"></div>

<h3>Korrektur: Start und Ziel verknüpfen</h3>
<div class="row">
<select id="sSel"><option value="">Start wählen ...</option></select>
<select id="zSel"><option value="">Ziel wählen ...</option></select>
</div>
<div class="row"><button onclick="doLink()">Verknüpfen</button></div>
<div class="c" style="font-size:12px">Bisherige Partner von Start und Ziel werden frei und lassen sich neu zuweisen.</div>

<h3>Auffälligkeiten</h3>
<table id="evts"><tr><td>keine</td></tr></table>

<div class="row"><a class="b" href="/export.csv" download>CSV</a>
<a class="b" href="/events.csv" download>Ereignisse</a>
<a class="b" href="/journal.csv" download>Rohdaten</a>
<button onclick="clr()">Alles löschen</button></div>
<div class="row"><a class="b" href="/ota">Firmware Update</a></div>
<div class="c" style="margin-top:8px;font-size:12px">Version )HTML" FIRMWARE_VERSION R"HTML(</div>

<script>
const f=x=>x.toFixed(3).replace('.',',')+' s';
let tsent=0,lastSig='';
function sync(d){
 if(!d.timeSet||Date.now()-tsent>60000){
  tsent=Date.now();
  fetch('/settime?ms='+Date.now()+'&tz='+(-new Date().getTimezoneOffset()));
 }
}
async function call(u){
 try{const r=await fetch(u);const t=await r.text();if(!r.ok)alert(t);}
 catch(e){alert('Keine Verbindung');}
 upd();
}
async function upd(){
 try{
  const d=await (await fetch('/api')).json();
  sync(d);
  const l=document.getElementById('link');
  if(d.age<0){l.textContent='Start nicht gesehen';l.className='pill bad';}
  else if(d.link){l.textContent='Start verbunden';l.className='pill ok';}
  else{l.textContent='Start offline '+d.age+' s';l.className='pill bad';}
  document.getElementById('sig').textContent=d.age<0?'':'RSSI '+Math.round(d.rssi)+' · SNR '+d.snr.toFixed(0);
  document.getElementById('bat').textContent=[d.batt>1999?'S '+(d.batt/1000).toFixed(2)+' V':'',d.zbatt>1999?'Z '+(d.zbatt/1000).toFixed(2)+' V':''].filter(x=>x).join(' · ');
  state.textContent=d.queue.length?d.queue.length+' Fahrer auf Strecke':'Bereit';
  last.textContent=d.runs.length&&d.runs[0].s!=null?f(d.runs[0].s):'-';
  best.textContent=d.best>0?'Bestzeit '+f(d.best):'';

  queue.innerHTML=d.queue.length?d.queue.map(q=>
   '<div class="qitem"><span class="qno">#'+q.n+'</span><span>Start '+q.t+'</span>'+
   '<button class="sm" onclick="call(\'/dnf?n='+q.n+'\')">DNF</button></div>').join('')
   :'<div class="c" style="padding:12px">Keine Fahrer auf der Strecke</div>';

  runs.innerHTML=d.runs.length?d.runs.map(r=>
   '<tr><td>#'+r.n+'</td><td>S '+r.st+' · Z'+r.z+' '+r.et+'</td><td>'+(r.s==null?'?':f(r.s))+'</td>'+
   '<td><button class="sm" onclick="call(\'/unlink?s='+r.n+'\')">lösen</button></td></tr>').join('')
   :'<tr><td>Keine</td></tr>';

  const fr=d.fins.filter(x=>x.n==0&&!x.i);
  free.innerHTML=fr.length?fr.map(x=>
   '<div class="qitem"><span class="qno">Z'+x.z+'</span><span>'+x.t+'</span>'+
   '<button class="sm" onclick="call(\'/ignore?z='+x.z+'\')">verwerfen</button></div>').join('')
   :'<div class="c" style="padding:8px">keine</div>';

  // Auswahllisten nur neu aufbauen, wenn sich die Daten ändern (Auswahl bleibt sonst erhalten)
  const ssig=JSON.stringify([d.starts,d.fins]);
  if(ssig!==lastSig){
   lastSig=ssig;
   const sv=sSel.value,zv=zSel.value;
   sSel.innerHTML='<option value="">Start wählen ...</option>'+d.starts.map(s=>
    '<option value="'+s.n+'">#'+s.n+' · '+s.t+' · '+(s.z?'mit Z'+s.z:(s.d?'DNF':'frei'))+'</option>').join('');
   zSel.innerHTML='<option value="">Ziel wählen ...</option>'+d.fins.map(z=>
    '<option value="'+z.z+'">Z'+z.z+' · '+z.t+' · '+(z.n?'mit #'+z.n:(z.i?'verworfen':'frei'))+'</option>').join('');
   sSel.value=sv;zSel.value=zv;
  }

  evts.innerHTML=d.events.length?d.events.map(e=>'<tr><td>'+e.t+'</td><td>'+e.e+'</td></tr>').join(''):'<tr><td>keine</td></tr>';
 }catch(e){const l=document.getElementById('link');l.textContent='Keine Verbindung';l.className='pill bad';}
}
async function doLink(){
 const s=sSel.value,z=zSel.value;
 if(!s||!z){alert('Bitte Start und Ziel wählen');return;}
 if(confirm('Start #'+s+' mit Ziel Z'+z+' verknüpfen?')){
  await call('/link?s='+s+'&z='+z);
  sSel.value='';zSel.value='';
 }
}
async function clr(){if(confirm('Alle Daten löschen?')){await call('/clear');}}
upd();setInterval(upd,1500);
</script></body></html>)HTML";

void handleApi() {
  bool link = seenOnce && (millis() - lastSeenMs < 10000);
  int age = seenOnce ? (int)((millis() - lastSeenMs) / 1000) : -1;
  String j;
  j.reserve(9000);
  j = "{\"link\":" + String(link ? "true" : "false");
  j += ",\"age\":" + String(age);
  j += ",\"rssi\":" + String(lastRssi, 0);
  j += ",\"snr\":" + String(lastSnr, 1);
  j += ",\"batt\":" + String(startBatt);
  j += ",\"zbatt\":" + String(battMv);
  j += ",\"timeSet\":" + String(timeSet ? "true" : "false");
  j += ",\"best\":" + String(bestTime, 3);

  // Fahrer auf der Strecke (offene Starts)
  j += ",\"queue\":[";
  bool first = true;
  int from = (int)totalStarts > MAX_EV ? (int)totalStarts - MAX_EV + 1 : 1;
  for (int n = from; n <= (int)totalStarts; n++) {
    StartEv* s = getStart((uint16_t)n);
    if (!s || !isOpen(*s)) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"n\":" + String(n) + ",\"t\":\"" + tLabel(s->boot, s->us) + "\"}";
  }

  // Ergebnisse: die letzten 10 Paare (nach Zielnummer, neueste zuerst)
  j += "],\"runs\":[";
  first = true;
  int cnt = 0;
  for (int z = totalFinishes; z >= 1 && cnt < 10 && (int)totalFinishes - z < MAX_EV; z--) {
    FinishEv* f = getFin((uint16_t)z);
    if (!f || f->ign || !f->startNo) continue;
    StartEv* s = getStart(f->startNo);
    if (!s) continue;
    float sec;
    bool ok = pairSec(*s, *f, sec);
    if (!first) j += ",";
    first = false;
    j += "{\"n\":" + String(s->no) + ",\"z\":" + String(f->no) +
         ",\"s\":" + (ok ? String(sec, 3) : String("null")) +
         ",\"st\":\"" + tLabel(s->boot, s->us) + "\",\"et\":\"" + tLabel(f->boot, f->us) + "\"}";
    cnt++;
  }

  // Listen für die Korrektur: die letzten 25 Starts und 25 Zielimpulse
  j += "],\"starts\":[";
  first = true;
  cnt = 0;
  for (int n = totalStarts; n >= 1 && cnt < 25 && (int)totalStarts - n < MAX_EV; n--) {
    StartEv* s = getStart((uint16_t)n);
    if (!s) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"n\":" + String(n) + ",\"t\":\"" + tLabel(s->boot, s->us) + "\",\"z\":" + String(s->finishNo) +
         ",\"d\":" + String(s->dnf ? 1 : 0) + "}";
    cnt++;
  }
  j += "],\"fins\":[";
  first = true;
  cnt = 0;
  for (int z = totalFinishes; z >= 1 && cnt < 25 && (int)totalFinishes - z < MAX_EV; z--) {
    FinishEv* f = getFin((uint16_t)z);
    if (!f) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"z\":" + String(z) + ",\"t\":\"" + tLabel(f->boot, f->us) + "\",\"n\":" + String(f->startNo) +
         ",\"i\":" + String(f->ign ? 1 : 0) + "}";
    cnt++;
  }

  j += "],\"events\":[";
  for (int i = 0; i < nEvt; i++) {
    if (i) j += ",";
    j += "{\"t\":\"" + fmtLocal(evts[i].unixS, false) + "\",\"e\":\"" + String(evts[i].text) + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleLink() {
  uint16_t s = (uint16_t)server.arg("s").toInt();
  uint16_t z = (uint16_t)server.arg("z").toInt();
  float sec = 0;
  const char* err = nullptr;
  if (!canLink(s, z, sec, err)) { server.send(400, "text/plain", err); return; }
  actLink(s, z);
  lastDraw = 0;
  logEvent("Verknuepft", "S#" + String(s) + " + Z" + String(z) + " = " + String(sec, 3) + " s", "VERKNUEPFT");
  server.send(200, "text/plain", "ok");
}

void handleUnlink() {
  uint16_t s = (uint16_t)server.arg("s").toInt();
  StartEv* st = getStart(s);
  if (!st || !st->finishNo) { server.send(400, "text/plain", "Start hat kein Ziel"); return; }
  uint16_t z = st->finishNo;
  actUnlink(s);
  lastDraw = 0;
  logEvent("Geloest", "S#" + String(s) + " / Z" + String(z), "GELOEST");
  server.send(200, "text/plain", "ok");
}

void handleDnf() {
  uint16_t n = (uint16_t)server.arg("n").toInt();
  if (!actDnf(n)) { server.send(400, "text/plain", "Start hat bereits ein Ziel oder existiert nicht"); return; }
  lastDraw = 0;
  logEvent("DNF", "Lauf #" + String(n), "DNF");
  server.send(200, "text/plain", "ok");
}

void handleIgnore() {
  uint16_t z = (uint16_t)server.arg("z").toInt();
  if (!actIgnore(z)) { server.send(400, "text/plain", "Ziel ist verknuepft oder existiert nicht"); return; }
  lastDraw = 0;
  logEvent("Verworfen", "Z" + String(z), "VERWORFEN");
  server.send(200, "text/plain", "ok");
}

// CSV aus dem aktuellen Stand erzeugen (Starts aufsteigend, danach Zielimpulse ohne Start)
void handleExport() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=zeiten.csv");
  server.send(200, "text/csv", "");
  server.sendContent("Lauf;Sekunden;Start;Ziel;Zielnr;Status\n");
  char b[160];
  int from = (int)totalStarts > MAX_EV ? (int)totalStarts - MAX_EV + 1 : 1;
  for (int n = from; n <= (int)totalStarts; n++) {
    StartEv* s = getStart((uint16_t)n);
    if (!s) continue;
    FinishEv* f = s->finishNo ? getFin(s->finishNo) : nullptr;
    float sec = 0;
    bool ok = f && pairSec(*s, *f, sec);
    const char* status = f ? (ok ? "OK" : "Zeit unbekannt") : (s->dnf ? "DNF" : "offen");
    snprintf(b, sizeof(b), "%u;%s;%s;%s;%s;%s\n", n,
             ok ? String(sec, 3).c_str() : "",
             fmtLocal(unixSOf(s->boot, s->us)).c_str(),
             f ? fmtLocal(unixSOf(f->boot, f->us)).c_str() : "",
             f ? String(f->no).c_str() : "", status);
    server.sendContent(b);
  }
  int fromF = (int)totalFinishes > MAX_EV ? (int)totalFinishes - MAX_EV + 1 : 1;
  for (int z = fromF; z <= (int)totalFinishes; z++) {
    FinishEv* f = getFin((uint16_t)z);
    if (!f || (f->startNo && getStart(f->startNo))) continue;
    snprintf(b, sizeof(b), ";;;%s;%u;%s\n", fmtLocal(unixSOf(f->boot, f->us)).c_str(), z,
             f->ign ? "Ziel verworfen" : "Ziel ohne Start");
    server.sendContent(b);
  }
  server.sendContent("");
}

void streamFile(const char* path, const char* name) {
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(404, "text/plain", "Keine Daten"); return; }
  server.sendHeader("Content-Disposition", String("attachment; filename=") + name);
  server.streamFile(f, "text/csv");
  f.close();
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
  bootNo = prefs.getUInt("boot", 0) + 1;       // zählt die Neustarts, trennt Zeiten verschiedener Sitzungen
  prefs.putUInt("boot", bootNo);
  loadJournal();

  logEvent("Neustart", "", "NEUSTART");
  battMv = readBattMv();
  if (!initRadio()) { Serial.println("LoRa Fehler"); while (true) delay(1000); }
  radio.startReceive();
  attachInterrupt(digitalPinToInterrupt(ZIEL_PIN), onFinish, RISING);

  WiFi.softAP("Zeitnahme", "12345678");
  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api", handleApi);
  server.on("/link", handleLink);
  server.on("/unlink", handleUnlink);
  server.on("/dnf", handleDnf);
  server.on("/ignore", handleIgnore);
  server.on("/ota", handleOtaPage);
  server.on("/update", HTTP_POST, handleOtaResult, handleOtaUpload);
  server.on("/export.csv", handleExport);
  server.on("/events.csv", []() { streamFile(EVT_FILE, "ereignisse.csv"); });
  server.on("/journal.csv", []() { streamFile(JOURNAL_FILE, "rohdaten.csv"); });
  server.on("/settime", []() {
    int64_t ms = strtoll(server.arg("ms").c_str(), nullptr, 10);
    int tz = server.arg("tz").toInt();
    if (ms > 1700000000000LL) {
      int64_t newOff = ms * 1000 - esp_timer_get_time();
      if (!timeSet || llabs(newOff - epochOffsetUs) > 2000000) {
        epochOffsetUs = newOff;
        timeSet = true;
        actBootOffset(bootNo, newOff);          // auch spätere Auswertung kennt die Uhrzeit dieser Sitzung
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
    uint16_t z = 0, s = 0;
    int r = actFinish(fu, z, s);
    if (r == 1) {
      sendPkt(T_RESULT, 0, 0, 0, s);
      rxFlag = false;
      radio.startReceive();
      startFlash();
      lastDraw = 0;
      Serial.printf("Ziel Z%u -> Lauf #%u (auf Strecke: %d)\n", z, s, queueCount);
    } else if (r == 2) {
      sendPkt(T_RESULT, 0, 0, 0, 0);
      rxFlag = false;
      radio.startReceive();
      startFlash();
      logEvent("Ziel ohne Start", "Z" + String(z) + " " + tLabel(bootNo, fu), "OHNE START");
    } else {
      logEvent("Zielimpuls zu kurz", "", "ZU KURZ");
    }
  }

  // Fahrer ohne Zieldurchfahrt nach RUN_TIMEOUT_S als Ausfall markieren
  static uint32_t lastTimeoutCheck = 0;
  if (millis() - lastTimeoutCheck > 1000) {
    lastTimeoutCheck = millis();
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < MAX_EV; i++) {
      if (isOpen(starts[i]) && now - starts[i].us > (int64_t)RUN_TIMEOUT_S * 1000000LL) {
        uint16_t n = starts[i].no;
        actDnf(n);
        logEvent("Timeout", "Lauf #" + String(n), "TIMEOUT");
      }
    }
  }

  if (millis() - lastBatt > 30000) { lastBatt = millis(); battMv = readBattMv(); }
  if (millis() - lastDraw > 500) {
    lastDraw = millis();
    if (millis() < infoPageUntil) {
      bool link = seenOnce && (millis() - lastSeenMs < 10000);
      drawInfoPage(otherBattMv, battMv, link);
    } else {
      drawZiel();
    }
  }
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
    else if (phase == 0) {
      // Kurzer Tastendruck (< 2s): Info-Seite anzeigen
      infoPageUntil = millis() + INFO_PAGE_MS;
    }
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
