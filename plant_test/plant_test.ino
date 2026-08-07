/*
 * Plant Tracker — bench test sketch
 *
 * For testing wiring and sensors only: no deep sleep, no ESP-NOW, no WiFi.
 * Reads every sensor once a second and shows the values on the OLED and
 * Serial. A failed sensor is reported on screen instead of hanging, so you
 * can see at a glance which one is misbehaving.
 *
 * Also shows the raw soil ADC value — water/dry the probe and use the
 * extremes to calibrate DRY_VALUE / WET_VALUE in the real sensor sketch.
 */

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SOIL_PIN 14
#define DHTPIN 4
#define DHTTYPE DHT22

// Keep in sync with the values in plant_tracker.ino.ino
const int DRY_VALUE = 2050;  // calibrate: sensor in dry air
const int WET_VALUE = 150;   // calibrate: sensor in water

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP085 bmp;
BH1750 lightMeter;
DHT dht(DHTPIN, DHTTYPE);

bool oledOk = false;
bool bmpOk = false;
bool lightOk = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Serial.println("Wire OK");

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.println(oledOk ? "OLED OK" : "OLED failed");

  bmpOk = bmp.begin();
  Serial.println(bmpOk ? "BMP OK" : "BMP180 failed");

  lightOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(lightOk ? "BH1750 OK" : "BH1750 failed");

  dht.begin();
  Serial.println("DHT22 started");
}

int oversample(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return sum / samples;
}

void loop() {
  int soilRaw = oversample(SOIL_PIN, 32);
  int moisture = constrain(map(soilRaw, DRY_VALUE, WET_VALUE, 0, 100), 0, 100);

  float pressure = bmpOk ? bmp.readPressure() / 100.0F : NAN;
  float lux = lightOk ? lightMeter.readLightLevel() : NAN;
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  bool dhtOk = !isnan(temperature) && !isnan(humidity);

  Serial.printf("T=%.1fC H=%.1f%% P=%.1fhPa L=%.1flx Soil=%d%% (raw %d)\n",
                temperature, humidity, pressure, lux, moisture, soilRaw);

  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    if (dhtOk) {
      display.printf("Tmp: %.1f C\n", temperature);
      display.printf("Hum: %.1f %%\n", humidity);
    } else {
      display.println("DHT22 error");
    }

    if (bmpOk) display.printf("Pres: %.1f hPa\n", pressure);
    else display.println("BMP180 error");

    if (lightOk && lux >= 0) display.printf("Light: %.1f lx\n", lux);
    else display.println("BH1750 error");

    display.printf("Soil: %d %%\n", moisture);
    display.printf("Raw: %d\n", soilRaw);
    display.display();
  }

  delay(1000);
}
