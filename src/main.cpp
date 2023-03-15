#include <Arduino.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// HEATING CARTIGE CONFIGURATION
#define HEATING 10
int HeatingPWM = 0;
bool HeatingOn = false;
#define HEATING_SWITCH 7

// MOTOR CONFIGURATION
#define MOTOR 9
#define STEPS 3200 // NEMA 17 STEPS PER REVOLUTION (1.8/STEP ON FULL)
#define M_RPM 30
#define M_SWITCH 8
// D11 PWM = 490Hz
int mSpeed = (((STEPS * M_RPM) / 60) * 255) / 29400; // STEPS PER MINUTE*255 / MAXPWM @ (490Hz)
bool motorOn = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);
int lcdRefershRate = 3600;

int TargetTemp = 210;
int LastTemp = 0;
int errorCounter = 0;
int lastMillis = 0;

// SPECIAL CHARACTERS
#define MOTOR_ON 0X0
#define MOTOR_OFF 0X01
#define HEATING_ON 0x02
#define HEATING_OFF 0x03

byte moOn[8] = {
    0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b01110};
byte moOff[8] = {
    0b11111,
    0b11011,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11011,
    0b11111};
byte heOn[8] = {
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b01110,
    0b00100,
    0b00100,
    0b00000,
};
byte heOff[8] = {
    0b11111,
    0b10001,
    0b10001,
    0b01010,
    0b01010,
    0b00100,
    0b00100,
    0b00000,
};

// THERMISTOR CONFIGURATION
#define SENSOR_PIN A0
#define REFERENCE_RESISTANCE 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
#define STM32_ANALOG_RESOLUTION 1024

Thermistor *thermistor;

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");

  lcd.init();
  lcd.backlight();
  lcd.clear();
  delay(500);
  lcd.print("    REC-PET");
  lcd.setCursor(0, 1);
  lcd.print("       v1");

  lcd.createChar(0, moOn);  // create a new custom character (index 0) MOTOR ON
  lcd.createChar(1, moOff); // create a new custom character (index 1) MOTOR OFF
  lcd.createChar(2, heOn);  // create a new custom character (index 2) HEATING ON
  lcd.createChar(3, heOff); // create a new custom character (index 2) HEATING OFF

  // HEATING CARTIGE
  pinMode(HEATING, OUTPUT);

  // THERMISTOR
  thermistor = new NTC_Thermistor(
      SENSOR_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE,
      STM32_ANALOG_RESOLUTION // <- for a thermistor calibration
  );

  // MOTOR
  pinMode(MOTOR, OUTPUT);

  // SWITCH CONFIGURATION
  pinMode(M_SWITCH, INPUT_PULLUP);
  pinMode(HEATING_SWITCH, INPUT_PULLUP);

  delay(1500);
  Serial.println("REC-PET v1");
  lastMillis = millis();
}

void ThermalRunaway()
{
  // SHUTDOWN HEATING
  digitalWrite(HEATING, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("THERMAL RUNAWAY");
  lcd.setCursor(0, 1);
  lcd.print("  PLEASE RESET");
  analogWrite(HEATING, 0);

  while (1)
  {
  }
}

void LCD_Print()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEMP: " + (String)LastTemp + " C ");
  lcd.write(HeatingOn ? HEATING_ON : HEATING_OFF);
  lcd.setCursor(0, 1);
  lcd.print("M: ");
  lcd.write(motorOn ? MOTOR_ON : MOTOR_OFF);
  lcd.setCursor(8, 1);
  lcd.print("PWM: " + (String)HeatingPWM);
}

void loop()
{
  int temperature = thermistor->readCelsius();

  int heatingSw = digitalRead(HEATING_SWITCH);
  if (heatingSw == LOW && HeatingOn != heatingSw)
  {
    HeatingOn = !HeatingOn;
  }

  if (HeatingOn)
  {
    // START HEATING

    if (LastTemp == 0)
      LastTemp = temperature;

    int CurrentDifference = LastTemp - temperature;

    // CHECKING FOR THERMAL RUNAWAY
    if (CurrentDifference < 0)
      CurrentDifference = CurrentDifference * -1;

    if ((CurrentDifference > 5) && errorCounter <= 3)
    {
      Serial.println("FLAG 1");

      if (errorCounter >= 3)
      {
        ThermalRunaway();
      }
      errorCounter++;
    }

    if (HeatingOn && temperature < LastTemp && errorCounter <= 3)
    {

      Serial.println("FLAG 2");
      if (errorCounter == 3)
        ThermalRunaway();

      errorCounter++;
    }

    if (temperature < TargetTemp)
    {
      if (HeatingPWM == 0)
        HeatingPWM = 255;
    }
    else
    {
      HeatingPWM = 0;
    }
    analogWrite(HEATING, HeatingPWM);
  }
  else
  {
    // SHUTDOWN HEATING
    if (HeatingPWM != 0)
    {
      HeatingPWM = 0;
      analogWrite(HEATING, HeatingPWM);
    }
  }

  LastTemp = temperature;

  int motorSw = digitalRead(M_SWITCH);
  if (motorSw == LOW && motorOn != motorSw)
  {
    motorOn = !motorOn;
  }

  if (motorOn)
  {
    // START MOTOR
    analogWrite(MOTOR, mSpeed);
  }
  else
  {
    // SHUTDOWN MOTOR
    analogWrite(MOTOR, 0);
  }

  if (millis() > lastMillis + lcdRefershRate)
  {
    LCD_Print();
    lastMillis = millis();
  }
}