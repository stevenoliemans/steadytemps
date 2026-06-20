#include <PID_v1.h>
#include <Servo.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// Servo
#define PIN_SERVO     5

// MAX31855 thermocouples (shared SPI bus)
#define PIN_CS1       4
#define PIN_CS2       3

// ST7789 display
#define TFT_DC        9
#define TFT_RST       8
#define TFT_BLK       7   // backlight, connect to 3.3V or control via pin

// Potentiometer
#define PIN_POT       A0

// PID parameters
// conservative: Kp=1, Ki=0.05, Kd=0.25
// aggressive:   Kp=4, Ki=0.2,  Kd=1
double Kp = 4, Ki = 0.2, Kd = 1;

// Timing
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 10000; // 10 seconds

// Variables
double target_temperature, actual_temperature1, actual_temperature2, servo_position;

// Objects
Servo valve_servo;
Adafruit_MAX31855 thermometer1(PIN_CS1);
Adafruit_MAX31855 thermometer2(PIN_CS2);
Adafruit_ST7789 tft = Adafruit_ST7789(-1, TFT_DC, TFT_RST); // no CS pin

// PID object
PID myPID(&actual_temperature1, &servo_position, &target_temperature, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // Backlight on
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  // Display init
  tft.init(240, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // Thermocouples
  thermometer1.begin();
  thermometer2.begin();

  // Servo
  valve_servo.attach(PIN_SERVO);

  // PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  startMillis = millis();

  // Splash screen
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 100);
  tft.println("  Initialising...");
  delay(1000);
}

void updateDisplay() {
  tft.fillScreen(ST77XX_BLACK);

  // Target temperature
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 10);
  tft.print("Target:  ");
  tft.print((int)target_temperature);
  tft.println(" C");

  // Thermocouple 1
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10, 45);
  tft.print("Temp 1:  ");
  tft.print((int)actual_temperature1);
  tft.println(" C");

  // Thermocouple 2
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 80);
  tft.print("Temp 2:  ");
  tft.print((int)actual_temperature2);
  tft.println(" C");

  // Servo position
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 115);
  tft.print("Servo:   ");
  tft.print((int)servo_position);
  tft.println(" deg");

  // Gap indicator
  double gap = abs(target_temperature - actual_temperature1);
  tft.setCursor(10, 150);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Gap:     ");
  tft.print((int)gap);
  tft.println(" C");

  // PID mode indicator
  tft.setCursor(10, 185);
  if (target_temperature > actual_temperature1 && gap >= 50) {
    tft.setTextColor(ST77XX_RED);
    tft.println("Mode: BOOST HIGH");
  } else if (target_temperature > actual_temperature1 && gap >= 20) {
    tft.setTextColor(ST77XX_ORANGE);
    tft.println("Mode: BOOST LOW");
  } else {
    tft.setTextColor(ST77XX_GREEN);
    tft.println("Mode: PID");
  }
}

void loop() {
  currentMillis = millis();

  // Read potentiometer and map to target temperature range
  target_temperature = map(analogRead(PIN_POT), 0, 1023, 100, 400);

  if (currentMillis - startMillis >= period) {
    startMillis = currentMillis;

    // Read thermocouples
    actual_temperature1 = thermometer1.readCelsius();
    actual_temperature2 = thermometer2.readCelsius();

    // Check for thermocouple errors
    if (isnan(actual_temperature1)) {
      actual_temperature1 = 0;
      Serial.println("Error reading thermocouple 1");
    }
    if (isnan(actual_temperature2)) {
      actual_temperature2 = 0;
      Serial.println("Error reading thermocouple 2");
    }

    // PID / gap logic
    double gap = abs(target_temperature - actual_temperature1);
    if (target_temperature > actual_temperature1 && actual_temperature1 <= 100 && gap >= 50) {
      servo_position = 60;
    } else if (target_temperature > actual_temperature1 && actual_temperature1 <= 100 && gap >= 20) {
      servo_position = 30;
    } else {
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.Compute();
      servo_position = map(servo_position, 0, 255, 30, 150); // safe limits
    }

    // Move servo
    valve_servo.write(servo_position);

    // Update display
    updateDisplay();

    // Serial debug
    Serial.print("T1: ");
    Serial.print(actual_temperature1);
    Serial.print(" T2: ");
    Serial.print(actual_temperature2);
    Serial.print(" Target: ");
    Serial.print(target_temperature);
    Serial.print(" Servo: ");
    Serial.println(servo_position);
  }

  delay(1000);
}
