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
 *   - Servo
 *   - PID by Brett Beauregard (PID_v1)
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
#include <PID_v1.h>
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
const int SERVO_MIN_DEG = 3; // Min servo travel
const int SERVO_MAX_DEG = 126; // Max servo travel

// Pulse range used by attach(). The Servo default (544-2400 us) pushes low
// angles down to ~600-730 us, which many servos reject and go limp. Pinning
// to a standard 1000-2000 us range keeps every commanded angle inside the
// servo's reliable holding band. Widen toward 600/2400 if you need more
// travel and your servo accepts it.
const int SERVO_MIN_US = 950;
const int SERVO_MAX_US = 2000;

Servo valveServo;

// Commands the valve to 0..100% within the safe band.
void setValvePercent(float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  int deg = SERVO_MIN_DEG + (int)((SERVO_MAX_DEG - SERVO_MIN_DEG) * pct / 100.0f + 0.5f);
  valveServo.write(deg);
}

// ---- PID temperature control ----
// Ported from the proven sketch. Process variable is TC1, setpoint is the
// pot target, output is the servo position. DIRECT acting (more output =
// more heat).
// conservative: Kp=1, Ki=0.05, Kd=0.25
// aggressive:   Kp=4, Ki=0.2,  Kd=1
double Kp = 4, Ki = 0.2, Kd = 1;

// Below this target (setpoint) temperature the valve is forced closed.
const double MIN_CONTROL_TEMP_C = 60;

double target_temperature = 0;    // setpoint (from pot)
double actual_temperature1 = 0;   // process variable (TC1)
double servo_position = 0;        // PID output / valve command
PID myPID(&actual_temperature1, &servo_position, &target_temperature, Kp, Ki, Kd, DIRECT);

// Original controller used a 10 s cycle; lowered for debugging so the valve
// reacts quickly. Raise back toward 10000 for normal operation.
const unsigned long PID_PERIOD_MS = 1000;
unsigned long lastPid = 0;

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

  // --- Servo init FIRST: always boot with the valve closed (= minimum) ---
  valveServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setValvePercent(0);   // 0% command -> SERVO_MIN_DEG (closed = the minimum)

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

  // --- PID init ---
  myPID.SetMode(AUTOMATIC);

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

// Renders the VLV cell. Only repaints when the text or color changes
// (flicker-free) and left-pads to a fixed width so old glyphs are cleared.
void drawValveCell(const char *text, uint16_t color) {
  static char last[12] = "";
  static uint16_t lastColor = 0;
  if (strcmp(text, last) == 0 && color == lastColor) {
    return;
  }
  strncpy(last, text, sizeof(last) - 1);
  last[sizeof(last) - 1] = '\0';
  lastColor = color;

  char buf[12];
  snprintf(buf, sizeof(buf), "%-6s", text);   // fixed 6-char field

  tft.setTextSize(3);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(VAL_X, ROW_VALVE_Y);
  tft.print(buf);
}

// PID-driven position, shown as a percentage (magenta).
void drawValve(int pct) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d %%", pct);
  drawValveCell(buf, ST77XX_MAGENTA);
}

// Hardcoded feedforward override state, shown instead of the percentage
// (orange) so it's clear the value isn't coming from the PID loop.
void drawValveMode(const char *label) {
  drawValveCell(label, ST77XX_ORANGE);
}

// Reads a thermocouple, updates its display row, logs it, and returns the
// temperature in Celsius (NAN on fault / no probe).
float readAndReport(Adafruit_MAX31855 &tc, int16_t valueY, const char *label) {
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

  return fault ? NAN : c;
}

// Ports the proven valve-control logic: a gap-based feedforward override when
// far below setpoint and still cold, otherwise the PID. The result is a
// 0..180 deg command (servo_position), which the original wrote straight to
// the servo. Here we convert it to a valve % and apply it through the safe
// 30-70% clamp instead of bypassing it.
void updatePidValve() {
  // On a bad sensor reading, fail safe: drive the valve closed (this also
  // keeps the servo actively driven, so it never goes limp at startup while
  // the MAX31855 cold-junction settles / reports an initial fault).
  if (isnan(actual_temperature1)) {
    setValvePercent(0);
    drawValveMode("TC ERR");
    Serial.println("PID skipped: TC1 fault -> valve closed");
    return;
  }

  // Target below the control threshold: keep the valve hardcoded closed,
  // overriding the PID and feedforward logic.
  if (target_temperature < MIN_CONTROL_TEMP_C) {
    setValvePercent(0);
    drawValveMode("Closed");
    Serial.print("Target below ");
    Serial.print((int)MIN_CONTROL_TEMP_C);
    Serial.println("C -> valve closed");
    return;
  }

  // Detect which mode set servo_position: a hardcoded feedforward override
  // or the PID loop itself.
  bool overrideMode = true;
  const char *modeLabel = "";

  double gap = abs(target_temperature - actual_temperature1);
  if (target_temperature > actual_temperature1 && actual_temperature1 <= 100 && gap >= 50) {
    servo_position = 60;
    modeLabel = "FF 60";
  } else if (target_temperature > actual_temperature1 && actual_temperature1 <= 100 && gap >= 20) {
    servo_position = 30;
    modeLabel = "FF 30";
  } else {
    overrideMode = false;
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.Compute();
    servo_position = map(servo_position, 0, 255, 15, 180);
  }

  // Valve is always physically driven; only the display differs by mode.
  float valve_pct = servo_position / 180.0f * 100.0f;
  setValvePercent(valve_pct);

  int pct_int = (int)(constrain(valve_pct, 0.0f, 100.0f) + 0.5f);
  if (overrideMode) {
    drawValveMode(modeLabel);     // show the hardcoded state, not the %
  } else {
    drawValve(pct_int);           // value came from the PID loop
  }

  Serial.print(overrideMode ? "FF" : "PID");
  Serial.print(" -> Sp(deg): ");
  Serial.print(servo_position);
  Serial.print("  VLV: ");
  Serial.print(pct_int);
  Serial.println(" %");
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

  // The smoothed pot value is the PID setpoint.
  target_temperature = filtered;

  static int lastShownTarget = -9999;
  if (lastShownTarget == -9999 || fabs(filtered - lastShownTarget) >= 1.0) {
    int shown = (int)(filtered + 0.5);
    if (shown != lastShownTarget) {
      drawTarget(shown);
      Serial.print("SET: ");
      Serial.print(shown);
      Serial.println(" C");
      lastShownTarget = shown;
    }
  }

  unsigned long now = millis();

  // Thermocouples update on the display cadence; TC1 feeds the PID.
  if (now - lastUpdate >= UPDATE_MS) {
    lastUpdate = now;
    actual_temperature1 = readAndReport(thermocouple1, ROW_TC1_Y, "TC1");
    readAndReport(thermocouple2, ROW_TC2_Y, "TC2");
  }

  // PID drives the valve on its own (slower) control cycle.
  if (now - lastPid >= PID_PERIOD_MS) {
    lastPid = now;
    updatePidValve();
  }
}
