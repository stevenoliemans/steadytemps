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
 *
 * Target-temperature potentiometer (10k):
 *   One outer leg -> 3.3V
 *   Other outer leg -> GND
 *   Wiper (center) -> A0
 *
 * Valve servo:
 *   Signal -> D6
 *   V+     -> external 5V supply (NOT the Nano's 3.3V)
 *   GND    -> common ground (servo supply GND + Nano GND tied together)
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX31855.h>
#include <Servo.h>
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

// ---- Target setpoint (10k potentiometer on A0) ----
#define POT_PIN        A0
const float TARGET_MIN_C = 0.0;     // pot fully CCW
const float TARGET_MAX_C = 300.0;   // pot fully CW
const int   ADC_MAX      = 1023;    // Nano 33 IoT default 10-bit ADC

// ---- Valve servo ----
// Travel is clamped to a safe band so the valve never slams its end stops.
// 0% valve command -> SERVO_MIN_DEG, 100% -> SERVO_MAX_DEG.
#define SERVO_PIN        6
const int SERVO_MIN_DEG = 54;    // 30% of 180 deg travel
const int SERVO_MAX_DEG = 126;   // 70% of 180 deg travel
Servo valveServo;

// Commands the valve to 0..100% within the safe band.
void setValvePercent(float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  int deg = SERVO_MIN_DEG + (int)((SERVO_MAX_DEG - SERVO_MIN_DEG) * pct / 100.0f + 0.5f);
  valveServo.write(deg);
}

// ---- Display layout (value column + row tops) ----
#define VAL_X         80
#define ROW_TARGET_Y  40
#define ROW_VALVE_Y   85
#define ROW_TC1_Y     130
#define ROW_TC2_Y     175

// Update cadence.
const unsigned long UPDATE_MS = 1000;
unsigned long lastUpdate = 0;

// Reads the potentiometer (averaged to suppress ADC jitter) and maps it
// to a target temperature in Celsius.
float readTargetC() {
  const int N = 32;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(POT_PIN);
  }
  float raw = sum / (float)N;
  return TARGET_MIN_C + (TARGET_MAX_C - TARGET_MIN_C) * (raw / (float)ADC_MAX);
}

void setup() {
  Serial.begin(115200);

  // --- Display init ---
  tft.init(240, 240, SPI_MODE3);
  tft.invertDisplay(true);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.println("steadytemps");

  // Static labels (size 2). Offset +7px so they bottom-align with the
  // size-3 values drawn at each row top.
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, ROW_TARGET_Y + 7);
  tft.println("SET");
  tft.setCursor(10, ROW_VALVE_Y + 7);
  tft.println("VLV");
  tft.setCursor(10, ROW_TC1_Y + 7);
  tft.println("TC1");
  tft.setCursor(10, ROW_TC2_Y + 7);
  tft.println("TC2");

  // --- Thermocouple init ---
  if (!thermocouple1.begin()) {
    Serial.println("MAX31855 #1 init failed!");
  }
  if (!thermocouple2.begin()) {
    Serial.println("MAX31855 #2 init failed!");
  }

  // --- Servo init ---
  valveServo.attach(SERVO_PIN);
  setValvePercent(0);   // park at the safe minimum (30% travel)

  // Small settle delay for the MAX31855 reference junction.
  delay(500);
}

// Draws a temperature value in a fixed area, clearing the previous value.
// y is the top of the value text. Returns nothing.
void drawTemp(int16_t y, float celsius, uint8_t fault) {
  // Clear the value region.
  tft.fillRect(VAL_X, y, 240 - VAL_X, 28, ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setCursor(VAL_X, y);

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

// Draws the target setpoint (whole degrees). Uses opaque text (fg over bg)
// with a fixed-width string so each redraw overwrites the previous value in
// a single pass - no clear-then-draw flash.
void drawTarget(int celsius) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%3d C", celsius);

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(VAL_X, ROW_TARGET_Y);
  tft.print(buf);
}

// Draws the valve position as a percentage. Opaque, fixed-width, flicker-free.
void drawValve(int pct) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%3d %%", pct);

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.setCursor(VAL_X, ROW_VALVE_Y);
  tft.print(buf);
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
  // Smooth the (already block-averaged) reading with an exponential moving
  // average to knock down residual ADC noise, then apply hysteresis so the
  // displayed whole-degree value only changes on clear, deliberate movement.
  static float filtered = NAN;
  float raw = readTargetC();
  if (isnan(filtered)) {
    filtered = raw;
  }
  filtered += 0.15f * (raw - filtered);

  static int lastShownTarget = -9999;
  if (lastShownTarget == -9999 || fabs(filtered - lastShownTarget) >= 1.0) {
    int shown = (int)(filtered + 0.5);
    if (shown != lastShownTarget) {
      drawTarget(shown);

      // For now, drive the valve directly from the target temperature:
      // map the full target range (TARGET_MIN_C..TARGET_MAX_C) to 0..100%.
      // This placeholder is where the PID output will go later.
      float pct = (shown - TARGET_MIN_C) / (TARGET_MAX_C - TARGET_MIN_C) * 100.0f;
      setValvePercent(pct);
      drawValve((int)(constrain(pct, 0.0f, 100.0f) + 0.5f));

      Serial.print("SET: ");
      Serial.print(shown);
      Serial.print(" C  VLV: ");
      Serial.print((int)(pct + 0.5f));
      Serial.println(" %");
      lastShownTarget = shown;
    }
  }

  // Thermocouples update on the slower cadence.
  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_MS) {
    lastUpdate = now;
    readAndReport(thermocouple1, ROW_TC1_Y, "TC1");
    readAndReport(thermocouple2, ROW_TC2_Y, "TC2");
  }
}
