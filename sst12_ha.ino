#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <max6675.h>
//#include <PWM.h>
#include <PID_v1.h>

// encoder pins
#define ENC_A  3 // INT0
#define ENC_B  12
#define ENC_C  2 // INT1

// iron pins
#define IRON_CTRL 9
#define IRON_TC_OUT A7
#define IRON_SHAKE_SENSOR 6
#define AIR_GUN_HEATER_CTRL 10
#define AIR_GUN_TC_OUT A6
#define AIR_GUN_FAN_CTRL 11
#define AIR_GUN_REED 8

// max6675 thermocouple, temporary for calibration
//#define thermoDO 4
//#define thermoCS 5
//#define thermoCLK 7

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

volatile int encoderPos = 0;
volatile int encoderMin, encoderMax, encoderStep;
volatile bool encoderLooped = true;
volatile unsigned int btnPress = 0;

// Sensor variables
int ironTemp, airGunTemp;

// PID variables for iron and air gun
double iron_sp, air_gun_sp, iron_in, air_gun_in, iron_out, air_gun_out;

//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID iron_PID(&iron_in, &iron_out, &iron_sp, 2, 5, 1, DIRECT);
//PID air_gun_PID(&air_gun_in, &air_gun_out, &air_gun_sp, 2, 5, 1, DIRECT);

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

  initEncoder(0, 0, 256, 10, false);

// Encoder pin on interrupt 0 - pin 2
  attachInterrupt(0, doEncoderBtn, CHANGE);
// Encoder pin on interrupt 1 - pin 3
  attachInterrupt(1, doEncoderTune, CHANGE);

// Init OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  
//  display.display();
//  delay(2000);

  // Clear the buffer.
  display.clearDisplay();

  // draw a single pixel
  display.drawPixel(10, 10, WHITE);
  // Show the display buffer on the hardware.
  // NOTE: You _must_ call display after making any drawing commands
  // to make them visible on the display hardware!
  display.display();

// ironPID init
  iron_in = 0;
  iron_sp = 0;
  iron_PID.SetMode(AUTOMATIC);

// for max6675
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Display Temperature in C
//  double tempReading2 = thermocouple.readCelsius();
//  Serial.println(tempReading2);
//  float tempVolts = tempReading * 3.3 / 1024.0;
//  float tempC = (tempVolts - 0.5) * 100.0;
  //         ----------------
//  lcd.print(tempReading);
//  lcd.print(tempC);
  
  // Display Light on second row
//  int lightReading = analogRead(lightPin);
//  lcd.setCursor(0, 1);
  //         ----------------
//  lcd.print("ADC2:         ");  
//  lcd.setCursor(6, 1);
//  lcd.print(encoderPos);//lightReading);
//  lcd.print(tempReading);

// Get temperature
  readTemp();

// iron PID regulation
  iron_sp = encoderPos;
  iron_in = ironTemp;
  iron_PID.Compute();
  analogWrite(IRON_CTRL, encoderPos); //iron_out);
//  pwmWrite(IRON_CTRL, encoderPos);
  analogWrite(AIR_GUN_HEATER_CTRL, encoderPos);
//  pwmWrite(AIR_GUN_HEATER_CTRL, encoderPos);
  analogWrite(AIR_GUN_FAN_CTRL, encoderPos);
//  pwmWrite(AIR_GUN_FAN_CTRL, encoderPos);

  display.clearDisplay();
  // text display tests
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("IRON: ");
  display.println(ironTemp);
  display.print("HAG: ");
  display.println(airGunTemp);
  display.print("ENC: ");
  display.println(encoderPos);
//  display.print("OUT: ");
//  display.println(iron_out);
//  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.display();
  btnPress = 0;

//  delay(200);
}

void readTemp() {
  int tempReading = analogRead(AIR_GUN_TC_OUT);
  airGunTemp = tempReading;

  
  analogWrite(IRON_CTRL, 0);
  delay(10);
  tempReading = analogRead(IRON_TC_OUT);
  ironTemp = tempReading;
  analogWrite(IRON_CTRL, encoderPos);
}

void initEncoder(unsigned int cur, unsigned int min, unsigned int max, unsigned int step, bool looped) {
  encoderPos = cur; encoderMin = min; encoderMax = max; encoderStep = step; encoderLooped = looped;
}

void doEncoderBtn() {
  btnPress = 1;

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
