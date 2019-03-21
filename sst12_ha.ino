#include <LiquidCrystal.h>

int tempPin = 0;
int lightPin = 1;

#define ENC_A  3
#define ENC_B  4
#define ENC_C  2

#define LCD_RS   8
#define LCD_EN   9
#define LCD_D4  10
#define LCD_D5  11
#define LCD_D6  12
#define LCD_D7  13

volatile unsigned int encoderPos = 0;

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup() 
{
  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_C, INPUT);
  digitalWrite(ENC_A, HIGH);       // turn on pull-up resistor
  digitalWrite(ENC_B, HIGH);       // turn on pull-up resistor
  digitalWrite(ENC_C, HIGH);       // turn on pull-up resistor

  attachInterrupt(0, doEncoderBtn, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder_Expanded, CHANGE);     // encoder pin on interrupt 1 - pin 3
}

void loop()
{
  // Display Temperature in C
  int tempReading = analogRead(tempPin);
  float tempVolts = tempReading * 3.3 / 1024.0;
  float tempC = (tempVolts - 0.5) * 100.0;
  //         ----------------
  lcd.setCursor(0, 0);
  lcd.print("ADC1:         ");
  lcd.setCursor(6, 0);
  lcd.print(tempReading);
//  lcd.print(tempC);
  
  // Display Light on second row
  int lightReading = analogRead(lightPin);
  lcd.setCursor(0, 1);
  //         ----------------
  lcd.print("ADC2:         ");  
  lcd.setCursor(6, 1);
  lcd.print(encoderPos);//lightReading);
//  lcd.print(tempReading);

  delay(100);
}

void doEncoder() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
//  Serial.println(encoderPos, DEC);
}

void doEncoderBtn() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   */
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
  Serial.println (encoderPos, DEC);          // debug - remember to comment out
                                              // before final program run
  // you don't want serial slowing down your program if not needed
}
