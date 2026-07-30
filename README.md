# Plant Tracker

Two-ESP32 plant monitoring system:

- **`plant_tracker.ino.ino`** — outdoor sensor node. Wakes every 10 minutes,
  reads soil moisture, temperature/humidity (DHT22), pressure (BMP180) and
  light (BH1750), sends one ESP-NOW packet to the hub, then deep-sleeps
  (~2–4 s awake per cycle, battery friendly).
- **`plant_hub/plant_hub.ino`** — indoor hub / base station on a second ESP32
  (no sensors attached). Receives the packets, keeps 48 h of history in RAM,
  and serves a phone-friendly dashboard with live values, history charts and
  a data table.

## Flashing

1. Flash `plant_hub/plant_hub.ino` to the second ESP32. Optionally set
   `HOME_WIFI_SSID` / `HOME_WIFI_PASS` at the top first, and change `AP_PASS`.
2. Flash `plant_tracker.ino.ino` to the sensor ESP32 (existing wiring unchanged).

No MAC addresses need to be copied between boards: the hub forces its own MAC
to a fixed value (`02:50:4C:41:4E:54`) that the sensor sketch already targets,
and the sensor auto-discovers the WiFi channel (and re-discovers it if your
router changes channels), caching it in RTC memory between sleeps.

## Viewing the dashboard

- **Standalone (default):** join the WiFi network `PlantHub` (password
  `plantpots` — change it), then open <http://192.168.4.1>.
- **Home WiFi:** with `HOME_WIFI_SSID` set, open <http://planthub.local> or
  the IP printed on the hub's serial monitor. The `PlantHub` AP stays up as a
  fallback either way.

The dashboard shows a Live/Offline badge (offline = no packet for 25 min),
per-metric history charts with hover/keyboard tooltips, 6h/24h/48h ranges, a
data table, and hub diagnostics (channel, packet count, radio signal strength,
raw soil reading for recalibration) in the footer.

## Tuning

- `SLEEP_MINUTES` in the sensor sketch — reporting interval (10 min default).
- `DRY_VALUE` / `WET_VALUE` — soil calibration; the current raw ADC value is
  shown in the dashboard footer to help recalibrate.
- `USE_OLED 0` — drop the OLED for extra battery life; the display only shows
  a ~2.5 s status splash per wake now that the dashboard replaces it.
- `HISTORY_LEN` in the hub sketch — samples kept in RAM (288 × 10 min = 48 h).
- `WIFI_COUNTRY` (both sketches) — the default world-safe setting only allows
  WiFi channels 1–11. If your home router uses channel 12/13 (common outside
  North America), set your ISO country code (e.g. `"GB"`) in **both** sketches
  so the sensor is allowed to transmit on the hub's channel.

## Hardware notes

- The soil sensor is on GPIO14 (**ADC2**), which cannot be read while WiFi is
  on — the sensor sketch reads all sensors *before* starting the radio, so
  keep that order if you edit it. Moving soil to an ADC1 pin (GPIO32–39)
  removes the constraint.
- Sensor init failures no longer hang the node (which would drain a battery);
  failed sensors are sent as "no data" and shown as `—` on the dashboard.
- Requires arduino-esp32 core 2.x or 3.x (callback signatures are handled for
  both).
