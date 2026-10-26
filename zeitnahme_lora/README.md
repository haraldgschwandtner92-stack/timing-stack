# Zeitnahme Start/Ziel per LoRa (v3)

Drahtloses Zeitnahmesystem mit zwei Heltec WiFi LoRa 32 V3 Boards für Start- und Ziel-Gate.

## Hardware

- **Board:** Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
- **Start-Gate:** Mikroschalter an GPIO 4 (gegen GND)
- **Ziel-Gate:** Lichtschranke/Schalter an GPIO 5 (gegen GND)

## Funktionen

- Funkübertragung via LoRa (869.525 MHz, EU 10% Duty-Cycle Band)
- OLED-Display zeigt: Verbindungsstatus, Laufnummer, letzte 3 Zeiten
- Akkuspannungsanzeige beider Gates
- WLAN-Webserver am Ziel-Gate (SSID: "Zeitnahme", IP: 192.168.4.1)
- CSV-Export der Läufe und Ereignisse
- Zeitsynchronisation über Smartphone-Browser
- Deep-Sleep-Modus zum Stromsparen

## Installation

1. **Arduino IDE** mit ESP32-Board-Support einrichten
2. **Bibliotheken** installieren (Bibliotheksverwalter):
   - RadioLib
   - U8g2
3. **Board auswählen:** Heltec WiFi LoRa 32 V3
4. **Sketch flashen** auf beide Boards

## Bedienung

### Rollenwahl
Beide Boards starten als START. Auf dem Ziel-Board:
- PRG-Taste **2 Sekunden** halten und loslassen → Rolle wechselt zu ZIEL

### Deep Sleep
- PRG-Taste **5 Sekunden** halten und loslassen → Ausschalten
- Aufwecken: PRG kurz drücken oder RST

### Weboberfläche (Ziel-Gate)
1. Mit WLAN "Zeitnahme" verbinden (Passwort: 12345678)
2. Browser öffnen: http://192.168.4.1

## Kalibrierung

Die Konstante `TX_CAL_US` (Standard: 1500 µs) kompensiert die Verzögerung zwischen Senden und Empfangen. Zum Kalibrieren:
1. Start und Ziel nebeneinander aufstellen
2. Beide gleichzeitig auslösen
3. Zeit sollte nahe 0 sein; ggf. `TX_CAL_US` anpassen

## Pin-Belegung

| Funktion | GPIO |
|----------|------|
| LoRa SCK | 9 |
| LoRa MISO | 11 |
| LoRa MOSI | 10 |
| LoRa NSS | 8 |
| LoRa DIO1 | 14 |
| LoRa RST | 12 |
| LoRa BUSY | 13 |
| LED | 35 |
| VEXT (OLED) | 36 |
| Start-Sensor | 4 |
| Ziel-Sensor | 5 |
| PRG-Taste | 0 |
| VBAT Read | 1 |
| VBAT Ctrl | 37 |

## Akkumessung

Bei Heltec V3.1: `VBAT_CTRL_ACTIVE` = LOW  
Bei Heltec V3.2: `VBAT_CTRL_ACTIVE` = HIGH (falls Anzeige Unsinn zeigt)

Feinabgleich: `VBAT_FACTOR_X100` mit Multimeter anpassen (Standard: 490).

## Lizenz

Open Source - frei verwendbar.
