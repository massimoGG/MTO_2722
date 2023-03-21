/*
  Arduino LSM9DS1 - Simple Accelerometer

  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <stdio.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>

#include <Arduino_LSM9DS1.h>

int PID(float kp, float ki, float kd, double curVal);
void accelerate(float acceleration);
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);
float getPitch();

#define PWM_FREQ 0x03

#define PIN_PWM_X 9
#define PIN_X_LEFT 7
#define PIN_X_RIGHT 6

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 2.14
//-8.58 // Declination (degrees) in Boulder, CO.

unsigned long prevTime  = 0;
unsigned long curTime   = 0;

void setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Serial Init */

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.begin(115200);

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");/////////////////////////////////////////////////////////////////////////////////////////////////

  /* Pin Modes */
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_X_LEFT, OUTPUT);
  pinMode(PIN_X_RIGHT, OUTPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* I2C init */
  Wire.begin();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
}

void regelDit() {
  float huidigeHoek;
  
  static float grootsteHoek;
  static int stoot;

  huidigeHoek = getPitch() + 8;
  Serial.print("Hoek:\t");
  Serial.print(huidigeHoek);
  /*
    Bereken stoot op de maximale hoek of negeer berekening
  */
  if (abs(huidigeHoek) > abs(grootsteHoek)) {
    grootsteHoek = huidigeHoek;
    /*    = PID(float kp, float ki, float kd, double toError,
                double priError, double curVal);*/
    stoot += PID(0.1, 0, 0, huidigeHoek);
    // Serial.print("\tNieuwe stoot:\t");
    // Serial.print(stoot);
  }
  Serial.print("\tStoot:\t");
  Serial.print(stoot);

  /*
    Reset als tussen [-10, 10] graden
    MAAR HIER GEVEN WE OOK DE STOOT!
  */
  if (abs(huidigeHoek) < 40) {
    grootsteHoek = 0;
    setMotorX(stoot);
  }
}

void ruisTest() {
  static int i = 0;

  static int cur;
  static unsigned long prevTime;
  
  unsigned long nu = millis();

  // Elke 10 sec sneller gaan
  if (nu - prevTime > 1E4) {
    
    // Sneller zetten
    if (cur > 255)
      cur = 255;
    else 
      cur += 20;

    prevTime = nu;
  }


  Serial.print(cur);
  Serial.print(" ");
  Serial.print(getPitch());

  setMotorX(cur);
}

void loop() {
  regelDit();

  Serial.println();
  delay(10);
}

int PID(float kp, float ki, float kd, double curVal) {
  static double priError;
  static double toError;
  int setP = 8;

  double error = setP - curVal;

  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;

  return PIDvalue;

  // int Fvalue = (int)PIDvalue;

  // Fvalue = map(Fvalue, -90, 90, -100, 100);
  // return Fvalue;
}

/**
 * Accelerates PWM 
 */
void accelerate(float acceleration) {
  curTime   = micros();

  /**
   * Hier berekenen welke versnellingsstoot we moeten hebben
   */


  prevTime  = curTime;
}

/***
 * Bestuurt de X-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorX(int16_t dc)
{
  int MAX = 200;
  if (dc < -MAX)
    dc = -MAX;
  if (dc > MAX)
    dc = MAX;
  //Serial.print("\tsetMotorX:\t");
  Serial.print(" ");
  Serial.print(dc);

  if (dc < 0)
  {
    // Anti-clockwise
    digitalWrite(PIN_X_RIGHT, LOW);
    digitalWrite(PIN_X_LEFT, HIGH);
  }
  else
  {
    // Clockwise
    digitalWrite(PIN_X_LEFT, LOW);
    digitalWrite(PIN_X_RIGHT, HIGH);
  }

  analogWrite(PIN_PWM_X, abs(dc));
  //  OCR2A = 100;
}

float getPitch()
{
  float ax, ay, az;

  if (!IMU.accelerationAvailable())
    return;
    
  IMU.readAcceleration(ax, ay, az);

  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // Convert radians to degrees
  pitch *= 180.0 / PI;
  return pitch;
}