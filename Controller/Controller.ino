/**
 * Libraries:
 * - Arduino_LSM9DS1 (Voor Accelerometer)
 */

/**
 * Includes
 */
#include <Arduino_LSM9DS1.h>
#include <ArduPID.h>

/**
 * Function Prototypes
 */
void calibrate();
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);

/**
 * Constants
 */
#define PWM_FREQ    0x03

#define PIN_PWM_X   9
#define PIN_X_LEFT  7
#define PIN_X_RIGHT 6

// To be implemented
#define PIN_PWM_Y   10
#define PIN_Y_LEFT  0
#define PIN_Y_RIGHT 0

float theta_Kp; 
float theta_Ki;
float theta_Kd;

float theta_Now;
float theta_Zero;
float theta_Error;

float theta_Integral;
float theta_Speed_Now;

unsigned long imu_Time_Now = 0;
unsigned long imu_Time_Prev= 0;

float angleX = 0;

ArduPID PID;
double setpoint = 0;
double input;
double output;
double p = 20;
double i = 0;
double d = 10;

void setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  /* Serial Init */
  Serial.begin(115200);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  /* Pin Modes */
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_X_LEFT, OUTPUT);
  pinMode(PIN_X_RIGHT, OUTPUT);

  /* Clear Timer1 */
  //OCR2A = 180; // (180+1) / 256 = 70.7%
  //OCR2B = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Accelerometer init */
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMO!");
    while(1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  imu_Time_Prev = micros();

  /* Aansturing */
  /* 
    PWM Signaal op clk/64 = 0x03 
  */
  //TCCR1B = TCCR1B | PWM_FREQ;

  /* PID regelaar */
  PID.begin(&input, &output, &setpoint, p, i, d);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  calibrate();

  Serial.println("Pendulum Stabiel - Initialized");
}

void loop() {
  // Temp 
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x,y,z);

    /* Update time difference */
    imu_Time_Prev = imu_Time_Now;
    imu_Time_Now = micros();

    /* Bepaal X-as hoek */

    theta_Error = theta_Now - theta_Zero;

    theta_Integral += theta_Error * (imu_Time_Now - imu_Time_Prev) / 1E6;

    /* X-as PID */
    input = x;
    PID.compute();
    //PID.debug(&Serial, "PID");

    /* PID OUTPUT */
    Serial.print("PID - ");
    Serial.println((uint8_t)output);
    setMotorX((uint8_t)output);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Hier regellus/berekeningen */

  float X_P_Accel = theta_Kp * (theta_Now - theta_Zero);;
  float X_I_Accel = theta_Ki * theta_Integral;
  float X_D_Accel = theta_Kd * theta_Speed_Now;
  
  float X_PID_Accel = X_P_Accel + X_I_Accel + X_D_Accel;

  float X_DC = 100;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Set waardes uitzenden naar motoren */


  delay(1); //? 
}

/**
 * Zet alle waardes op hun equivalent beginstand
 */
void calibrate() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x,y,z);

  }
}

/***
 * Bestuurt de X-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorX(int16_t dc) {
  Serial.print("setMotorX - ");
  Serial.println(dc);

  if (dc < 0) {
    // Anti-clockwise
    digitalWrite(PIN_X_RIGHT, LOW);
    digitalWrite(PIN_X_LEFT, HIGH);
  } else {
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
void setMotorY(int16_t dc) {
}