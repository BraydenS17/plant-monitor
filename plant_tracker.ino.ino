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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP085 bmp;
BH1750 lightMeter;
DHT dht(DHTPIN, DHTTYPE);

const int DRY_VALUE = 2050;  // calibrate: sensor in dry air
const int WET_VALUE = 150;   // calibrate: sensor in water

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Serial.println("Wire OK");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    while (true);
  }
  Serial.println("OLED OK");

  if (!bmp.begin()) {
    Serial.println("BMP180 failed");
    while (true);
  }
  Serial.println("BMP OK");

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 failed");
    while (true);
  }
  Serial.println("BH1750 OK");

  dht.begin();
  Serial.println("DHT22 OK");

  Serial.println("All sensors initialized!");
}

int oversample(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return sum / samples;
}

int readSoil() {
  int raw = oversample(SOIL_PIN, 32);
  Serial.println(raw);

  int moisture = map(raw, DRY_VALUE, WET_VALUE, 0, 100);
  return constrain(moisture, 0, 100);
}

void loop() {
  float pressure = bmp.readPressure() / 100.0F;
  float lux = lightMeter.readLightLevel();
  int moisture = readSoil();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  bool dhtOk = !isnan(temperature) && !isnan(humidity);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if (dhtOk) {
    display.print("Tmp: ");
    display.print(temperature, 1);
    display.println(" C");

    display.print("Hum: ");
    display.print(humidity, 1);
    display.println("%");
  } else {
    display.println("DHT22 error");
    Serial.println("DHT22 read failed");
  }

  display.print("Pres: ");
  display.print(pressure, 1);
  display.println(" hPa");

  display.print("Light: ");
  display.print(lux, 1);
  display.println(" lx");

  display.print("Soil: ");
  display.print(moisture);
  display.println("%");

  display.display();
  delay(50);
}