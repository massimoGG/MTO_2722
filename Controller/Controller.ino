/**
 * Libraries:
 * - Arduino_LSM9DS1 (Voor Accelerometer)
 */

/**
 * Includes
 */
#include <Arduino_LSM9DS1.h>

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
#define PIN_X_LEFT  6
#define PIN_X_RIGHT 7

// To be implemented
#define PIN_PWM_Y   10
#define PIN_X_LEFT  0
#define PIN_X_RIGHT 0




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
  OCR2A = 180; // (180+1) / 256 = 70.7%
  OCR2B = 0;

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

  /* Aansturing */
  /* 
    PWM Signaal op clk/64 = 0x03 
  */
  TCCR1B = TCCR1B | PWM_FREQ;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.println("Pendulum Stabiel - Initialized");
}

void loop() {
  // Temp 
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print("Acceleration: ");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x,y,z);

    Serial.print("Gyroscope: ");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Hier regellus/berekeningen */

  /*
  // Implementatie van ander project 

  left_P_Accel = theta_Kp * (theta_Now - theta_Zero);
  left_I_Accel = theta_Ki * theta_Integral;
  left_D_Accel = theta_Kd * theta_Speed_Now;
  left_S_Accel = theta_Ks * left_Speed_RPM / 1000.;
  left_PID_Accel = left_P_Accel + left_I_Accel + left_D_Accel + left_S_Accel;

  float friction = 0;
  if (left_Speed_RPM > 0.5) {
    friction = friction_Value;
  } else if (left_Speed_RPM < -0.5) {
    friction = -friction_Value;
  }

  float voltage = (left_PID_Accel + 0.303 * left_Speed_RPM + friction) / 9.4;     // Equation measured from Acceleration Motor Tests

  // Ipv voltage zouden we duty-cycle moeten hebben
  left_PID_Voltage = round(constrain(voltage, -voltage_Max, voltage_Max));
  */

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Set waardes uitzenden naar motoren */


  delay(1); //? 
}

/**
 * Zet alle waardes op hun equivalent beginstand
 */
void calibrate() {
  float x, y, z;

  // Lees huidige toestand uit
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x,y,z);

  }

  // Zet beginpositie gelijk aan die waardes
  


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
    digitalWrite(PIN_X_RIGHT, 0);
    digitalWrite(PIN_X_LEFT, 1);
  } else {
    // Clockwise
    digitalWrite(PIN_X_LEFT, 0);
    digitalWrite(PIN_X_RIGHT, 1);
  }

  analogWrite(PIN_PWM_X, abs(dc));
}

/***
 * Bestuurt de Y-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorY(int16_t dc) {
}