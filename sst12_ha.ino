#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1.h>

// encoder pins
#define ENC_A  3
#define ENC_B  12
#define ENC_C  2

// iron pins
#define IRON_CTRL 9
#define IRON_TC_OUT A7
#define IRON_SHAKE_SENSOR 6 // Why not 7?
#define AIR_GUN_HEATER_CTRL 11 // Not PWM pin
#define AIR_GUN_TC_OUT A6
#define AIR_GUN_FAN_CTRL 10
#define AIR_GUN_REED 8

// max6675 thermocouple temporary
#define thermoDO 4
#define thermoCS 5
#define thermoCLK 7

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

volatile unsigned int encoderPos = 0;
volatile unsigned int btnPress = 0;

// PID variables for iron and air gun
double iron_sp, air_gun_sp, iron_in, air_gun_in, iron_out, air_gun_out;

PID iron_PID(&iron_in, &iron_out, &iron_sp, 2, 5, 1, DIRECT);
//PID air_gun_PID(&air_gun_in, &air_gun_out, &air_gun_sp, 2, 5, 1, DIRECT);

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
// Init PWM and ctrl pins
  pinMode(IRON_CTRL, OUTPUT);
  pinMode(AIR_GUN_FAN_CTRL, OUTPUT);
  TCCR1B = TCCR1B & 0b11111000 | 0x05;
  analogWrite(IRON_CTRL, 0);
  analogWrite(AIR_GUN_FAN_CTRL, 0);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_C, INPUT);
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  digitalWrite(ENC_C, HIGH);

  attachInterrupt(0, doEncoderBtn, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder_Expanded, CHANGE);     // encoder pin on interrupt 1 - pin 3

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
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
  int tempReading = analogRead(IRON_TC_OUT);
  //double tempReading = thermocouple.readCelsius();
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

// iron PID regulation
  iron_sp = encoderPos;
  iron_in = tempReading;
  iron_PID.Compute();
  analogWrite(IRON_CTRL, iron_out);//encoderPos % 256);

  display.clearDisplay();
  // text display tests
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("TMP: ");
  display.println(tempReading);
  display.print("ENC: ");
  display.println(encoderPos);
  display.print("OUT: ");
  display.println(iron_out);
//  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.display();
  btnPress = 0;

  delay(200);
}

void doEncoderBtn() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   */
  btnPress = 1;
//  Serial.println("Btn changed");
}

void doEncoder_Expanded(){
  if (digitalRead(ENC_A) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(ENC_B) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoderPos = encoderPos - 1;         // CCW
    } 
    else {
      encoderPos = encoderPos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(ENC_B) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoderPos = encoderPos + 1;          // CW
    } 
    else {
      encoderPos = encoderPos - 1;          // CCW
    }

  }
//  Serial.println (encoderPos, DEC);          // debug - remember to comment out
                                              // before final program run
  // you don't want serial slowing down your program if not needed
}
