#define USE_TIMER_1     false
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include <TimerInterrupt.h>
#include <ISR_Timer.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <ArduinoTapTempo.h>
#include <LiquidCrystal.h>

#define LCD_RS       8
#define LCD_RW       9
#define LCD_EN      10
#define LCD_D4       4
#define LCD_D5       5
#define LCD_D6       6
#define LCD_D7       7
#define LCD_CHARS   16
#define LCD_LINES    2

LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define LED 2
#define MAX_BPM 480
#define MIN_BPM 40

#define MULT1 11
#define MULT2 12


const int BUTTON_PIN = 3;
ArduinoTapTempo tapTempo;

ClickEncoder *encoder;
int16_t last, value, bpmtemp, encoderpos, interval;
float period;
uint16_t counter = 0;
unsigned long lastUpdate = 0;
uint8_t multiplier = 1; 

int multplusState = 0;
int multminState = 0;
int lastmultplusState = 0;
int lastmultminState = 0;

void timerIsr() {
  encoder->service();
}

//--PULSE INTERRUPT--//

void PulseHandler()
{

  counter+=2;
  if (counter >= period)
  {
    counter = 0;
    digitalWrite(LED, !digitalRead(LED));
  }
}



void displayAccelerationStatus() {
  lcd.setCursor(0, 0);
  lcd.print("BEATS PER MINUTE: ");
}


void setup()
{

  // IO INIT
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  pinMode(MULT1, INPUT_PULLUP);
  pinMode(MULT2, INPUT_PULLUP);

  // ENCODER INIT
  encoder = new ClickEncoder(A1, A0, A2, 4);

  // LCD INIT
  lcd.begin(LCD_CHARS, LCD_LINES);
  lcd.clear();
  displayAccelerationStatus();

  // VARIABLES
  value = 120;                    // The BPM value
  bpmtemp = 0;                    // Temporary BPM
  encoderpos = 0;                 // Encoder position
  last = -1;                      // Last BPM (unused, potentially usable to only update LCD and period once the BPM actually changed)
  interval = 1;                   // Period of BPM timer in ms (2ms or double speed because a pulse consists of an upgoing and downgoing slewage)
  period = 30000 / value * 2;   // Convert BPM to desired output period

  // TIMER INIT
  Timer1.initialize(1000);            // For rotary acceleration
  Timer1.attachInterrupt(timerIsr); 
  ITimer2.init();                     // For output pulses
  if (ITimer2.attachInterruptInterval(interval, PulseHandler));

  tapTempo.setMinBPM((float)MIN_BPM);
  tapTempo.setMaxBPM((float)MAX_BPM);

  //tapTempo.disableSkippedTapDetection();
  //tapTempo.setTotalTapValues(4);
  tapTempo.setBeatsUntilChainReset(10);
  
  Serial.begin(9600);
}

void loop() {
  boolean buttonDown = digitalRead(BUTTON_PIN) == LOW; // if pressed, buttondown = true
  tapTempo.update(buttonDown); // update if buttondown = true

  
  if ((uint16_t)tapTempo.getBPM() != bpmtemp)
  {
    encoderpos = 0;
    bpmtemp = (uint16_t)tapTempo.getBPM();
  }
  encoderpos += encoder->getValue();

  // MIGHT BE ABLE TO PUT LINES 119 THROUGH 131 INTO AN IF ELSE STATEMENT
  
  value = bpmtemp + encoderpos;

  if (value >= MAX_BPM) // Restrict to max bpm
  {
    encoderpos = MAX_BPM - bpmtemp;
    value = MAX_BPM;
  }

  if (value <= MIN_BPM) // Restrict to min bpm
  {
    encoderpos = MIN_BPM - bpmtemp;
    value = MIN_BPM;
  }

  // Alter the output pulse period based on the bpm
  period = (30000 / value * 2) / multiplier;

  multplusState = digitalRead(MULT2);

 // compare the buttonState to its previous state
  if (multplusState != lastmultplusState) {
    // if the state has changed, increment the counter
    if (multplusState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
    multiplier += 1;

    if(multiplier >4)
    {
      multiplier = 4; 
    }

    lcd.setCursor(11, 1);
    lcd.print(multiplier);
    } 
  }
  // save the current state as the last state,
  //for next time through the loop
  lastmultplusState = multplusState;

  multminState = digitalRead(MULT1);

 // compare the buttonState to its previous state
  if (multminState != lastmultminState) {
    // if the state has changed, increment the counter
    if (multminState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
    multiplier -= 1;

    if(multiplier <1)
    {
      multiplier = 1;
    }
    lcd.setCursor(11, 1);
    lcd.print(multiplier);
    } 
  }
  // save the current state as the last state,
  //for next time through the loop
  lastmultminState = multminState;
  
  




    
   // Show the current BPM on the LCD
if (lastUpdate - millis() > 16 && last != value)
{
  lastUpdate = millis();
  last = value;
    Serial.println("JA");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    
    lcd.setCursor(6, 1);
    lcd.print(value);
    
    lcd.setCursor(10, 1);
    lcd.print("x");

    lcd.setCursor(11, 1);
    lcd.print(multiplier);
    
  
    if (value == MAX_BPM)
    {
      lcd.setCursor(13, 1);
      lcd.print("MAX");
    }
  
    if (value == MIN_BPM)
    {
      lcd.setCursor(0, 1);
      lcd.print("MIN");
    }
}
 




  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    Serial.print("Button: ");
#define VERBOSECASE(label) case label: Serial.println(#label); break;
    switch (b) {
        VERBOSECASE(ClickEncoder::Pressed);
        VERBOSECASE(ClickEncoder::Held)
        VERBOSECASE(ClickEncoder::Released)
        VERBOSECASE(ClickEncoder::Clicked)
      case ClickEncoder::DoubleClicked:
        Serial.println("ClickEncoder::DoubleClicked");
        encoder->setAccelerationEnabled(!encoder->getAccelerationEnabled());
        Serial.print("  Acceleration is ");
        Serial.println((encoder->getAccelerationEnabled()) ? "enabled" : "disabled");


#ifdef WITH_LCD
        displayAccelerationStatus();
#endif
        break;
    }
  }
}
