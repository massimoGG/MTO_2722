/**
 * Includes
 */
#include <Wire.h>
#include <SPI.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

// #include <Arduino_LSM9DS1.h>

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

// PID variables
float theta_Kp;
float theta_Ki;
float theta_Kd;

float theta_Now;
float theta_Zero;
float theta_Error;

float theta_Integral;
float theta_Speed_Now;

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
    while (1)
      ;
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

  printAttitude(IMU.ax, IMU.ay, IMU.az, -IMU.my, -IMU.mx, IMU.mz);
  Serial.println();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Hier regellus/berekeningen */

  float X_P_Accel = theta_Kp * (theta_Now - theta_Zero);
  ;
  float X_I_Accel = theta_Ki * theta_Integral;
  float X_D_Accel = theta_Kd * theta_Speed_Now;

  float X_PID_Accel = X_P_Accel + X_I_Accel + X_D_Accel;

  float X_DC = 100;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Set waardes uitzenden naar motoren */

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
  Serial.print("setMotorX - ");
  Serial.println(dc);

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
  float pitch = atan2(IMU.ay, IMU.az);

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

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
}