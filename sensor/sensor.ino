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

int PID(float kp, float ki, float kd, double toError, double priError, double curVal);
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
  Serial.begin(115200);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void loop() {
  float huidigeHoek;
  
  static float grootsteHoek;
  static int stoot;

  huidigeHoek = getPitch();
  Serial.print("Hoek: ");
  Serial.print(huidigeHoek);
  /*
    Bereken stoot op de maximale hoek of negeer berekening
    TODO: Met ABS() ervan om ook bij negatieve kant te doen
  */
  if (abs(huidigeHoek) > abs(grootsteHoek)) {
    grootsteHoek = huidigeHoek;
    /*    = PID(float kp, float ki, float kd, double toError,
                double priError, double curVal);*/
    stoot = PID(1, 1, 1, 1, 1, huidigeHoek);
    Serial.print(" | Nieuwe stoot:\t");
    Serial.print(stoot);
  }
  
  /*
    Reset als tussen [-10, 10] graden
    MAAR HIER GEVEN WE OOK DE STOOT!
  */
  if (huidigeHoek > -10 && huidigeHoek < 10) {
    grootsteHoek = 0;
    setMotorX(stoot);
  }

  //Serial.print(getPitch());

  Serial.println();
  delay(10);
}

int PID(float kp, float ki, float kd, double toError, double priError, double curVal) {
  int setP = 10;
  double error = setP - curVal;

  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;

  int Fvalue = (int)PIDvalue;

  Fvalue = map(Fvalue, -90, 90, -50, 50);
  return Fvalue;
}

/**
 * Accelerates PWM 
 */
void accelerate(float acceleration) {
  curTime   = micros();
  float x, y, z;

  if (!IMU.accelerationAvailable()) {
    // IMU not available
    return;
  }
  IMU.readAcceleration(x, y, z);
  
  float hoek = getPitch();

  Serial.print("(");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(", ");
  Serial.print(hoek);
  Serial.print(") ");

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
  if (dc < -255)
    dc = -255;
  if (dc > 255)
    dc = 255;
  Serial.print("| setMotorX - ");
  Serial.print(dc);
  Serial.print("\t| ");

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