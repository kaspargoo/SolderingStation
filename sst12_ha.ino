#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <max6675.h>
//#include <PWM.h>
#include <PID_v1.h>

// UI display modes
#define UI_MAIN 0
#define UI_SET 1
#define UI_ABOUT 2

// UI cursor
#define SEL_IRON 0
#define SEL_HAG 1
#define SEL_HAG_FAN 2

/*#define UI_MM_IRON 0
#define UI_MM_HAG 1
#define UI_MM_SETTINGS 2
#define UI_IRON 3
#define UI_HAG 4
#define UI_SETTINGS 5
#define UI_ABOUT 6
*/
// encoder pins
#define ENC_A  3 // INT0
#define ENC_B  12
#define ENC_C  2 // INT1

// iron pins
#define IRON_CTRL 9
#define IRON_TC_OUT A7
#define IRON_SHAKE_SENSOR 6
#define IRON_THERMISTOR A3
#define AIR_GUN_HEATER_CTRL 10
#define AIR_GUN_TC_OUT A6
#define AIR_GUN_FAN_CTRL 11
#define AIR_GUN_REED 8

// max6675 thermocouple, temporary for calibration
//#define thermoDO 4
//#define thermoCS 5
//#define thermoCLK 7

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

unsigned int UIMode, UIBackTimer, UICursor;

int prevEncoderPos;
volatile int encoderPos;
volatile int encoderMin, encoderMax, encoderStep;
volatile bool encoderLooped = true;
volatile unsigned int btnPress = 0;

// Control variables
int ironTempCtrl, airGunTempCtrl, airGunFanCtrl;

// Sensor variables
int ironTemp, airGunTemp, ironHandleTemp;
byte ironSakeSensor;

// PID variables for iron and air gun
double iron_sp, air_gun_sp, iron_in, air_gun_in, iron_out, air_gun_out;

//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID iron_PID(&iron_in, &iron_out, &iron_sp, 2, 5, 1, DIRECT);
PID air_gun_PID(&air_gun_in, &air_gun_out, &air_gun_sp, 2, 5, 1, DIRECT);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
//#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
// Lower PWM frequency to 30 Hz
  TCCR1B = TCCR1B & 0b11111000 | 0x05;

// Init PWM and ctrl pins
// PWM lib. Init all timers except for 0, to save time keeping functions
//  InitTimersSafe();
// PWM lib. Set IRON_CTRL and AIR_GUN_HEATER_CTRL pins frequency to 3 Hz
//  SetPinFrequencySafe(IRON_CTRL, 3);
//  SetPinFrequencySafe(AIR_GUN_HEATER_CTRL, 3);
//  SetPinFrequencySafe(AIR_GUN_FAN_CTRL, 490);

  pinMode(IRON_CTRL, OUTPUT);
  pinMode(AIR_GUN_HEATER_CTRL, OUTPUT);
  pinMode(AIR_GUN_FAN_CTRL, OUTPUT);

  analogWrite(IRON_CTRL, 0);
  analogWrite(AIR_GUN_HEATER_CTRL, 0);
  analogWrite(AIR_GUN_FAN_CTRL, 0);

// Init encoder pins and assign functions to interruptions
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_C, INPUT);
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  digitalWrite(ENC_C, HIGH);

// Encoder pin on interrupt 0 - pin 2
  attachInterrupt(0, doEncoderBtn, CHANGE);
// Encoder pin on interrupt 1 - pin 3
  attachInterrupt(1, doEncoderTune, CHANGE);

// Init OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  
// Controls init
  ironTempCtrl = 0; airGunTempCtrl = 0, airGunFanCtrl = 0;

// PIDs init
  iron_in = 0; iron_sp = 0; iron_PID.SetMode(AUTOMATIC);

// for max6675
//  delay(1000);

//  initEncoder(0, 0, 250, 10, false);
  UICursor = 0;
  setUIMode(UI_ABOUT, 1000);

}

void loop() {
  
// Read sensor data
  readSensors();

// Draw UI
  drawUI();

// Calculate control values
  calcControls();

// Update control state
  setControls();

}

void drawUI() {
  String cc;

  if (UIBackTimer > 0 && UIBackTimer <= millis()) {
    setUIMode(UI_MAIN, 0);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);

  switch (UIMode) {
    case UI_SET:
      display.setTextSize(4);
      display.setCursor(20,16);
      display.println(encoderPos);
      display.setTextSize(1);
      display.println(String(UIBackTimer) + " " + String(millis()));
      display.print(String(prevEncoderPos) + " " + String(encoderPos));

      if (prevEncoderPos != encoderPos) {
        switch (UICursor) {
          case SEL_IRON:
            ironTempCtrl = encoderPos;
            break;
          case SEL_HAG:
            airGunTempCtrl = encoderPos;
            break;
          case SEL_HAG_FAN:
            airGunFanCtrl = encoderPos;
        }
        prevEncoderPos = encoderPos;
        UIBackTimer = millis() + 3000;
      }

      if (btnPress == 1) {
        setUIMode(UI_MAIN, 0);
        btnPress = 0;
      }
      break;
    case UI_ABOUT:
      display.setTextSize(2);
      display.setCursor(0,0);
      display.println(" SST12_HA ");
      display.println("SW: 0.3");
      display.println("HW: REV A");
      display.setTextSize(1);
      display.println(String(UIBackTimer) + " " + String(millis()));
      display.println("kaspargoo@gmail.com");
      if (btnPress == 1) {
        setUIMode(UI_MAIN, 0);
        btnPress = 0;
      }
      break;
    default: // UI_MAIN
      UICursor = encoderPos;
      display.setTextSize(2);
      display.setCursor(0,0);
      if (UICursor == SEL_IRON) cc = ">"; else cc = " ";
      display.print(cc + "IT: ");
      display.println(ironTemp);
      if (UICursor == SEL_HAG) cc = ">"; else cc = " ";
      display.print(cc + "HT: ");
      display.println(airGunTemp);
      if (UICursor == SEL_HAG_FAN) cc = ">"; else cc = " ";
      display.print(cc + "HF: ");
      display.println(airGunFanCtrl);
      display.setTextSize(1);
      display.print("EP: ");
      display.println(String(UIBackTimer) + " " + String(millis()));
      display.print(String(UICursor) + " " + String(btnPress));

      if (btnPress == 1) {
        setUIMode(UI_SET, 3000);
        btnPress = 0;
      }
  }

  display.display();

//  display.setTextColor(BLACK, WHITE); // 'inverted' text
//  display.drawPixel(10, 10, WHITE);

}

void setUIMode(unsigned int mode, unsigned int backTimer) {

  switch (mode) {
    case UI_SET:
      UIMode = UI_SET;
      switch (UICursor) {
        case SEL_IRON:
          initEncoder(ironTempCtrl, 0, 750, 10, false);
          break;
        case SEL_HAG:
          initEncoder(airGunTempCtrl, 0, 750, 10, false);
          break;
        case SEL_HAG_FAN:
          initEncoder(airGunFanCtrl, 0, 100, 10, false);
      }
      break;
    case UI_ABOUT:
      UIMode = UI_ABOUT;
      break;
    default: //UI_MAIN
      UIMode = UI_MAIN;
      initEncoder(UICursor, 0, 2, 1, true);
  }

  if (backTimer > 0)
    UIBackTimer = millis() + backTimer;
  else
    UIBackTimer = 0;

}

void readSensors() {
  // Display Temperature in C
//  double tempReading2 = thermocouple.readCelsius();
//  Serial.println(tempReading2);
//  float tempVolts = tempReading * 3.3 / 1024.0;
//  float tempC = (tempVolts - 0.5) * 100.0;

  int tempReading = analogRead(AIR_GUN_TC_OUT);
  airGunTemp = tempReading;
  
  tempReading = analogRead(IRON_THERMISTOR);
  ironHandleTemp = tempReading;

  analogWrite(IRON_CTRL, 0);
  delay(10);
  tempReading = analogRead(IRON_TC_OUT);
  ironTemp = tempReading;
  analogWrite(IRON_CTRL, encoderPos);
}

void calcControls() {

  if (ironTempCtrl > 0) {
    iron_sp = ironTempCtrl;
    iron_in = ironTemp;
    iron_PID.Compute();
  }
  else iron_out = 0;

  if (airGunTempCtrl > 0) {
    air_gun_sp = airGunTempCtrl;
    air_gun_in = airGunTemp;
    air_gun_PID.Compute();
  }
  else air_gun_out = 0;
  
}

void setControls() {
  analogWrite(IRON_CTRL, 255 * iron_out); //);
//  pwmWrite(IRON_CTRL, encoderPos);
  analogWrite(AIR_GUN_HEATER_CTRL, 0.2 * 255 * air_gun_out);
//  pwmWrite(AIR_GUN_HEATER_CTRL, encoderPos);
  analogWrite(AIR_GUN_FAN_CTRL, 255 * ironTempCtrl / 100);
//  pwmWrite(AIR_GUN_FAN_CTRL, encoderPos);
}

void initEncoder(unsigned int cur, unsigned int min, unsigned int max, unsigned int step, bool looped) {
  encoderPos = cur; encoderMin = min; encoderMax = max; encoderStep = step; encoderLooped = looped;
}

void doEncoderBtn() {

  if (digitalRead(ENC_C) == HIGH) {
    btnPress = 1;
  }
//  Serial.println("Btn changed");
}

void doEncoderTune() {

int diff = 0;

  if (digitalRead(ENC_A) == HIGH) {   // found a low-to-high on channel A
/*    if (digitalRead(ENC_B) == LOW)    // check channel B to see which way encoder is turning
      encoderPos = encoderPos - 1;    // CCW
    else
      encoderPos = encoderPos + 1;    // CW
*/  }
  else {                              // found a high-to-low on channel A
    if (digitalRead(ENC_B) == LOW)    // check channel B to see which way encoder is turning  
      diff = encoderStep;    // CW
    else
      diff = -encoderStep;   // CCW
    }
 

  if (encoderPos + diff > encoderMax) {
    if (encoderLooped)
      encoderPos = encoderMin;
    else
      encoderPos = encoderMax;
  } else if (encoderPos + diff < encoderMin) {
    if (encoderLooped)
      encoderPos = encoderMax;
    else
      encoderPos = encoderMin;
  } else {
    encoderPos = encoderPos + diff;
  }

//  Serial.println (encoderPos, DEC);
}
