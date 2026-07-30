/*
 * Plant Tracker — outdoor sensor node
 *
 * Wakes every SLEEP_MINUTES, reads all sensors, fires one ESP-NOW packet
 * at the hub (plant_hub sketch on a second ESP32), then deep-sleeps.
 * Awake time is ~2-4 s per cycle, so this can run on battery.
 *
 * Pairing is zero-config: the hub forces its STA MAC to HUB_MAC below,
 * so this node always knows where to send. The WiFi channel is discovered
 * by scanning until the hub ACKs, then cached in RTC memory so later
 * wakes send immediately. The scan covers channels 1-11 by default; set
 * WIFI_COUNTRY (in BOTH sketches) to reach routers on channels 12-13.
 *
 * NOTE: GPIO14 (soil sensor) is an ADC2 pin. ADC2 does not work while
 * WiFi is active, so all sensor reads happen BEFORE the radio starts.
 * Do not reorder. (Long-term, moving soil to an ADC1 pin, GPIO32-39,
 * removes this constraint.)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>

// Set to 0 to drop the OLED entirely (saves power; the hub's web page
// replaces it as the main way to read the sensors).
#define USE_OLED 1

#if USE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#if !defined(ESP_ARDUINO_VERSION_VAL)
#error "This sketch requires arduino-esp32 core 2.x or newer"
#endif
#define CORE_AT_LEAST_3_3 (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0))

// ---------------- configuration ----------------
const uint32_t SLEEP_MINUTES   = 10;
const uint32_t DISPLAY_HOLD_MS = 2500;  // how long the OLED stays on after a send

// Must match HUB_MAC in plant_hub.ino (the hub sets its own MAC to this).
static const uint8_t HUB_MAC[6] = {0x02, 0x50, 0x4C, 0x41, 0x4E, 0x54};

// WiFi regulatory domain. "01" (world-safe) only allows transmitting on
// channels 1-11 on arduino-esp32 core 3.x. If your router uses channel
// 12/13 (common outside North America), set your ISO country code here,
// e.g. "GB", "DE", "AU" — in BOTH sketches.
#define WIFI_COUNTRY "01"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SOIL_PIN 14
#define DHTPIN 4
#define DHTTYPE DHT22

const int DRY_VALUE = 2050;  // calibrate: sensor in dry air
const int WET_VALUE = 150;   // calibrate: sensor in water

// ---------------- shared packet ----------------
// !! Keep this struct byte-identical with the copy in plant_hub/plant_hub.ino
#define PKT_MAGIC   0x504C4E54UL  // "PLNT"
#define PKT_VERSION 1

#define FLAG_DHT_OK   0x01
#define FLAG_BMP_OK   0x02
#define FLAG_LIGHT_OK 0x04

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t bootCount;
  float    temperatureC;  // NAN when invalid
  float    humidityPct;   // NAN when invalid
  float    pressureHpa;   // NAN when invalid
  float    lux;           // NAN when invalid
  uint16_t soilRaw;
  uint8_t  soilPct;
  uint8_t  flags;
} PlantPacket;

// ---------------- state ----------------
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint8_t  lastChannel = 0;  // 0 = unknown, forces a scan

Adafruit_BMP085 bmp;
BH1750 lightMeter;
DHT dht(DHTPIN, DHTTYPE);
#if USE_OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOk = false;
#endif

volatile int sendResult = -1;  // -1 pending, 0 failed, 1 delivered

// ---------------- helpers ----------------
#if CORE_AT_LEAST_3_3
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  sendResult = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
}

int oversample(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return sum / samples;
}

bool trySend(const PlantPacket &pkt) {
  sendResult = -1;
  if (esp_now_send(HUB_MAC, (const uint8_t *)&pkt, sizeof(pkt)) != ESP_OK) return false;
  uint32_t t0 = millis();
  while (sendResult == -1 && millis() - t0 < 150) delay(1);
  return sendResult == 1;
}

bool sendOnChannel(uint8_t ch, const PlantPacket &pkt, uint8_t attempts) {
  if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  for (uint8_t i = 0; i < attempts; i++) {
    if (trySend(pkt)) return true;
  }
  return false;
}

void goToSleep() {
  uint64_t sleepUs = (uint64_t)SLEEP_MINUTES * 60ULL * 1000000ULL;
  uint64_t awakeUs = (uint64_t)millis() * 1000ULL;
  if (awakeUs < sleepUs / 2) sleepUs -= awakeUs;  // keep a ~10-min cadence
  Serial.printf("Sleeping for %llu s\n", sleepUs / 1000000ULL);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(sleepUs);
  esp_deep_sleep_start();
}

// ---------------- main ----------------
void setup() {
  Serial.begin(115200);
  bootCount++;
  bool timerWake = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
  Serial.printf("\nBoot #%u (woke from %s)\n", bootCount, timerWake ? "timer" : "power-on/reset");

  Wire.begin(21, 22);

  // DHT22 needs ~2 s after power-up before it answers; start it first and
  // do everything else inside that window.
  dht.begin();

  bool bmpOk = bmp.begin();
  if (!bmpOk) Serial.println("BMP180 init failed");

  // One-shot mode: the BH1750 powers itself down after the measurement.
  bool lightOk = lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  if (!lightOk) Serial.println("BH1750 init failed");

#if USE_OLED
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println("OLED init failed");
#endif

  // --- sensor reads (must finish before WiFi starts: GPIO14 is ADC2) ---
  uint16_t soilRaw = oversample(SOIL_PIN, 32);
  uint8_t soilPct = constrain(map(soilRaw, DRY_VALUE, WET_VALUE, 0, 100), 0, 100);

  float pressure = NAN;
  if (bmpOk) pressure = bmp.readPressure() / 100.0f;

  float lux = NAN;
  if (lightOk) {
    delay(180);  // high-res one-shot measurement time
    float lv = lightMeter.readLightLevel();
    if (lv >= 0) lux = lv;
  }

  // The DHT22 stays powered through deep sleep, so only a cold boot needs
  // its ~2 s power-up window; timer wakes can read immediately.
  if (!timerWake) {
    while (millis() < 2100) delay(10);
  }
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) {
    delay(2200);  // DHT22 minimum sampling interval
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
  }
  bool dhtOk = !isnan(temperature) && !isnan(humidity);
  if (!dhtOk) Serial.println("DHT22 read failed");

  PlantPacket pkt = {};
  pkt.magic = PKT_MAGIC;
  pkt.version = PKT_VERSION;
  pkt.bootCount = bootCount;
  pkt.temperatureC = dhtOk ? temperature : NAN;
  pkt.humidityPct = dhtOk ? humidity : NAN;
  pkt.pressureHpa = pressure;
  pkt.lux = lux;
  pkt.soilRaw = soilRaw;
  pkt.soilPct = soilPct;
  pkt.flags = (dhtOk ? FLAG_DHT_OK : 0) | (bmpOk ? FLAG_BMP_OK : 0) | (lightOk ? FLAG_LIGHT_OK : 0);

  Serial.printf("T=%.1fC H=%.1f%% P=%.1fhPa L=%.1flx Soil=%u%% (raw %u)\n",
                pkt.temperatureC, pkt.humidityPct, pkt.pressureHpa, pkt.lux, soilPct, soilRaw);

  // --- radio: send to the hub ---
  uint8_t usedChannel = 0;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  if (strcmp(WIFI_COUNTRY, "01") != 0) esp_wifi_set_country_code(WIFI_COUNTRY, false);

  // Highest channel we may transmit on: world-safe mode stops at 11; a
  // country code can extend the scan to 13 so it can follow an EU router.
  uint8_t maxCh = 11;
  wifi_country_t cinfo;
  if (esp_wifi_get_country(&cinfo) == ESP_OK) {
    maxCh = constrain(cinfo.schan + cinfo.nchan - 1, 11, 13);
  }

  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, HUB_MAC, 6);
    peer.channel = 0;  // follow whatever channel the radio is on
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // Try the cached channel first, then scan.
    if (lastChannel >= 1 && lastChannel <= maxCh && sendOnChannel(lastChannel, pkt, 3)) {
      usedChannel = lastChannel;
    } else {
      for (uint8_t ch = 1; ch <= maxCh && usedChannel == 0; ch++) {
        if (ch == lastChannel) continue;
        if (sendOnChannel(ch, pkt, 2)) usedChannel = ch;
      }
    }
    lastChannel = usedChannel;  // 0 = not found, rescan next wake
  } else {
    Serial.println("esp_now_init failed");
  }

  if (usedChannel) Serial.printf("Delivered to hub on channel %u\n", usedChannel);
  else Serial.println("Hub not reachable — is plant_hub powered on and in range?");

#if USE_OLED
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.printf("Boot #%u  ", bootCount);
    if (usedChannel) display.printf("sent ch%u\n", usedChannel);
    else display.println("SEND FAIL");
    if (dhtOk) {
      display.printf("Tmp: %.1f C\n", temperature);
      display.printf("Hum: %.1f %%\n", humidity);
    } else {
      display.println("DHT22 error");
    }
    if (bmpOk) display.printf("Pres: %.1f hPa\n", pressure);
    if (lightOk && !isnan(lux)) display.printf("Light: %.1f lx\n", lux);
    display.printf("Soil: %u %%\n", soilPct);
    display.display();
    delay(DISPLAY_HOLD_MS);
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
#endif

  goToSleep();
}

void loop() {
  // never reached — setup() ends in deep sleep
}
