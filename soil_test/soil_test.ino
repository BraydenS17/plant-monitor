/*
 * Plant Tracker — soil moisture sensor test
 *
 * Tests ONLY the capacitive soil sensor on GPIO14 — nothing else needs to
 * be wired. Prints the raw ADC value, the mapped percentage, and a running
 * session min/max four times a second.
 *
 * To calibrate: hold the probe in dry air for a few seconds, then submerge
 * it in water. The MAX seen is your DRY_VALUE and the MIN seen is your
 * WET_VALUE — copy them into plant_tracker.ino.ino and plant_test.ino.
 */

#define SOIL_PIN 14

// Current calibration (used for the % column only)
const int DRY_VALUE = 2050;  // sensor in dry air
const int WET_VALUE = 150;   // sensor in water

int sessionMin = 4095;
int sessionMax = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Capacitive soil sensor test on GPIO14");
  Serial.println("Dry air -> highest raw value, water -> lowest.");
  Serial.println();
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
  int raw = oversample(SOIL_PIN, 32);
  int moisture = constrain(map(raw, DRY_VALUE, WET_VALUE, 0, 100), 0, 100);

  if (raw < sessionMin) sessionMin = raw;
  if (raw > sessionMax) sessionMax = raw;

  // Simple bar so trends are visible while you handle the probe
  char bar[26];
  int fill = map(moisture, 0, 100, 0, 25);
  for (int i = 0; i < 25; i++) bar[i] = i < fill ? '#' : '-';
  bar[25] = '\0';

  Serial.printf("raw %4d  |%s| %3d%%   session min %4d (WET_VALUE)  max %4d (DRY_VALUE)\n",
                raw, bar, moisture, sessionMin, sessionMax);

  delay(250);
}
