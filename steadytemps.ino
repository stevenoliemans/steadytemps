/*
 * steadytemps - GMT130-V1.0 IPS 240x240 ST7789 SPI display
 *               + 2x Adafruit MAX31855 thermocouple amplifiers
 * Board: Arduino Nano 33 IoT (SAMD21)
 *
 * Display wiring (display -> Nano 33 IoT) - HARDWARE SPI:
 *   GND -> GND
 *   VCC -> 3.3V
 *   SCK -> D13   (hardware SPI SCK)
 *   SDA -> D11   (hardware SPI MOSI)
 *   RES -> D8
 *   DC  -> D9
 *   BLK -> 3.3V
 *   (no CS pin -> pass -1 for CS)
 *
 * MAX31855 wiring (both boards) - SOFTWARE SPI (kept off the display bus,
 * because the display has no CS and is always selected on hardware SPI):
 *   Both boards share:
 *     VIN -> 3.3V
 *     GND -> GND
 *     CLK -> D2   (shared software-SPI clock)
 *     DO  -> D3   (shared software-SPI data out / MISO)
 *   Per-board chip select:
 *     Board 1 CS -> D4
 *     Board 2 CS -> D5
 *
 * Libraries (install all via Library Manager):
 *   - Adafruit GFX Library
 *   - Adafruit ST7735 and ST7789 Library
 *   - Adafruit BusIO (dependency)
 *   - Adafruit MAX31855 library
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX31855.h>
#include <SPI.h>

// ---- Display (hardware SPI) ----
#define TFT_CS   -1   // no CS pin on this module
#define TFT_DC    9
#define TFT_RST   8

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ---- Thermocouples (software SPI) ----
#define MAX_CLK   2   // shared clock
#define MAX_DO    3   // shared data out (SO)
#define MAX_CS1   4
#define MAX_CS2   5

Adafruit_MAX31855 thermocouple1(MAX_CLK, MAX_CS1, MAX_DO);
Adafruit_MAX31855 thermocouple2(MAX_CLK, MAX_CS2, MAX_DO);

// Update cadence.
const unsigned long UPDATE_MS = 1000;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);

  // --- Display init ---
  tft.init(240, 240, SPI_MODE3);
  tft.invertDisplay(true);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 10);
  tft.println("steadytemps");

  // Static labels.
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 70);
  tft.println("TC1");
  tft.setCursor(10, 140);
  tft.println("TC2");

  // --- Thermocouple init ---
  if (!thermocouple1.begin()) {
    Serial.println("MAX31855 #1 init failed!");
  }
  if (!thermocouple2.begin()) {
    Serial.println("MAX31855 #2 init failed!");
  }

  // Small settle delay for the MAX31855 reference junction.
  delay(500);
}

// Draws a temperature value in a fixed area, clearing the previous value.
// y is the top of the value text. Returns nothing.
void drawTemp(int16_t y, float celsius, uint8_t fault) {
  // Clear the value region.
  tft.fillRect(10, y, 220, 30, ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setCursor(10, y);

  if (fault) {
    tft.setTextColor(ST77XX_RED);
    tft.print("FAULT");
    return;
  }

  if (isnan(celsius)) {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO TC");
    return;
  }

  tft.setTextColor(ST77XX_GREEN);
  tft.print(celsius, 1);
  tft.print(" C");
}

void readAndReport(Adafruit_MAX31855 &tc, int16_t valueY, const char *label) {
  uint8_t fault = tc.readError();
  float c = tc.readCelsius();

  drawTemp(valueY, c, fault);

  Serial.print(label);
  Serial.print(": ");
  if (fault) {
    Serial.print("FAULT 0x");
    Serial.println(fault, HEX);
  } else {
    Serial.print(c);
    Serial.println(" C");
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_MS) {
    return;
  }
  lastUpdate = now;

  readAndReport(thermocouple1, 100, "TC1");
  readAndReport(thermocouple2, 170, "TC2");
}
