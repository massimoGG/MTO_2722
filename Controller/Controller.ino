/**
 * Includes
 SPARKFUN LSM0DS1.h!!
 */
#include <Wire.h>
#include <SPI.h>

#include <SparkFunLSM9DS1.h>

/**
 * Function Prototypes
 */
void calibrate();
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);

float getPitch();
float getRoll();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

/**
 * Constants
 */
#define PWM_FREQ 0x03

#define PIN_PWM_X 9
#define PIN_X_LEFT 7
#define PIN_X_RIGHT 6

// To be implemented
#define PIN_PWM_Y 10
#define PIN_Y_LEFT 0
#define PIN_Y_RIGHT 0

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 2.14
//-8.58 // Declination (degrees) in Boulder, CO.

// Limieten wat de pendulum nog kan tegenwerken
#define limit_lin_min 0
#define limit_lin_max 0

// IMU
unsigned long imu_Time_Now = 0;
unsigned long imu_Time_Prev = 0;

LSM9DS1 IMU;

float angleX = 0;

void setup()
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Serial Init */
  Serial.begin(115200);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Pin Modes */
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_X_LEFT, OUTPUT);
  pinMode(PIN_X_RIGHT, OUTPUT);

  /* Clear Timer1 */
  // OCR2A = 180; // (180+1) / 256 = 70.7%
  // OCR2B = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* I2C init */
  Wire.begin();

  /* IMU init */
  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMO!");
    while (1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  imu_Time_Prev = micros();

  /* Aansturing */
  /*
    PWM Signaal op clk/64 = 0x03
  */
  // TCCR1B = TCCR1B | PWM_FREQ;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  calibrate();

  Serial.println("Pendulum Stabiel - Initialized");
}

void loop()
{
  if (IMU.gyroAvailable())
    IMU.readGyro();
  if (IMU.accelAvailable())
    IMU.readAccel();
  if (IMU.magAvailable())
    IMU.readMag();

  // Temp
  float x, y, z;

  /*
  if (IMU.gyroAvailable())
  {
    IMU.readGyro();
    Serial.print(IMU.calcGyro(IMU.gx), 2);
    Serial.print(", ");
    Serial.print(IMU.calcGyro(IMU.gy), 2);
    Serial.print(", ");
    Serial.print(IMU.calcGyro(IMU.gz), 2);
    Serial.println(" deg/s");
  }*/

  imu_Time_Prev = imu_Time_Now;
  imu_Time_Now = micros();

  //printAttitude(IMU.ax, IMU.ay, IMU.az, -IMU.my, -IMU.mx, IMU.mz);
  //Serial.println();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Hier regellus/berekeningen */
  //Serial.print("Pitch - ");
  Serial.print("90 -90 ");
  Serial.print(getPitch());
  Serial.print(" ");
  Serial.print(IMU.ax);
  //Serial.print("\t| ");
  float hoek = getPitch();

  float offset = 11;
  float lim = 10;
  
  // NP Correctie
  hoek -= offset;

  // Regelaar
  int16_t hoekversnelling = map(IMU.ax, );


  if (-90 < hoek && hoek < 90) {
    if (hoek > lim) {
      setMotorX(-100);
    }
    if (hoek < -lim) {
      setMotorX(100);
    }
  } else {
    setMotorX(0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Set waardes uitzenden naar motoren */

  Serial.println();
  delay(50); //?
}

/**
 * Zet alle waardes op hun equivalent beginstand
 */
void calibrate()
{
  float x, y, z;

  if (IMU.gyroAvailable())
  {
    IMU.readGyro();
  }
}

/***
 * Bestuurt de X-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorX(int16_t dc)
{
  /*
  Serial.print("setMotorX - ");
  Serial.print(dc);
  Serial.print("\t| ");
*/
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

/***
 * Bestuurt de Y-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorY(int16_t dc)
{
}

float getPitch()
{
  //printAttitude(IMU.ax, IMU.ay, IMU.az, -IMU.my, -IMU.mx, IMU.mz);
  float ax = IMU.ax;
  float ay = IMU.ay;
  float az = IMU.az;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // Convert radians to degrees
  pitch *= 180.0 / PI;
  return pitch;
}

float getRoll()
{
  float roll = atan2(IMU.ay, IMU.az);

  // Convert radians to degrees
  roll *= 180.0 / PI;
  return roll;
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // Convert everything from radians to degrees:
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  Serial.print("Pitch, Roll:\t");
  Serial.print(pitch, 2);
  Serial.print(",\t");
    Serial.print(roll, 2);
    Serial.print("\t| ");
}
