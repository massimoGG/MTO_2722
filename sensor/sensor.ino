/*
Inverted Pendulum 

- Geef stoot/versnelling tussen [-10, 10] graden
- Hoek meting bepaalt PWM (Enkel grootste hoek onthouden)
- PWM bepaalt rechtstreeks de RPM
- BV 50 PWM van de 255 => 20% RPM van de MAX RPM (~10k RPM) = 2kRPM

TODO:
- Versnelling met PWM
- ipv setMotorX rechtstreeks aan te spreken, versnel functie oproepen? Dit bepaalt dan hoeveel meer PWM erbij aangestuurd moet worden
- MAX Hoek = [-90, 90]
*/

#include <stdio.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>

// Sensor
#include <Arduino_LSM9DS1.h>

int PID(double curVal);
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);
float getPitch(float ax, float ay, float az);
float getRoll();

#define PWM_FREQ 0x03

#define PIN_PWM_X 9
#define PIN_X_LEFT 7
#define PIN_X_RIGHT 6

float setPoint = 0;
unsigned long prevTime = 0;
unsigned long curTime = 0;

/// @brief PID values
const float kp=0.1, ki=0.0, kd=0.0;

void setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.begin(115200);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  //IMU.setContinuousMode();

  /////////////////////////////////////////////////////////////////////////////////////////////////

  /* Pin Modes */
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_X_LEFT, OUTPUT);
  pinMode(PIN_X_RIGHT, OUTPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* I2C init */
  Wire.begin();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  /**
   * Calibreer rustpunt
   */
  // Bepaal hoek
  setPoint = getRoll();
  Serial.print("New Setpoint is: ");
  Serial.println(setPoint);
}

/**
 * @brief Regellus algorithme
 */
void regelDit() {
  // Huidige hoek gemeten door sensor
  float huidigeHoek;
  int output;

  // Grootste hoek gemeten tot nu toe
  static float grootsteHoek;
  // De huidige PWM die aangelegd wordt aan de motor
  static int huidigePWM;
  static int stoot;

  huidigeHoek = getRoll();
  // Als er geen hoek is, skip deze iteratie
  if (huidigeHoek == 1000)
    return;
  Serial.print(huidigeHoek);

  /**
   * Update telkens de grootste huidige hoek
   */
  if (abs(huidigeHoek) > abs(grootsteHoek)) {
    grootsteHoek = huidigeHoek;

    // Bereken aan te leggen versnelling bij grootste hoek
    output = PID(huidigeHoek);
    Serial.print(" \tPID output: ");
    Serial.print(output);

    // Vergelijk met hudige.
    // Verschil bepaalt in te stellen PWM?
    Serial.print(" \tHuidigePWM: ");
    Serial.print(huidigePWM);
    huidigePWM += output; // Tel op?
    Serial.print(" \tNieuwe PWM: ");
    Serial.print(huidigePWM);
  }

  /*
    Reset als tussen [-10, 10] graden
    MAAR HIER GEVEN WE OOK DE STOOT!
  */
  if (abs(huidigeHoek) < 10) {
    grootsteHoek = 0;
    setMotorX(huidigePWM);
  }
}

#define AANTAL_GEMIDDELD 15
float metingen[AANTAL_GEMIDDELD];

void loop() {
  unsigned int now = 0, prev = 0;

  now = millis();

  /**
   * Meting hoek - Roll
   * Links  = -90
   * Rechts = +90
   */

  /**
   * Regel systeem aan de hand van de uitgelezen waardes
   */
  // regelDit();
  setMotorX(100);

  float gemiddeldeHoek = 0;

  static int index;
  metingen[index] = getRoll();
  index++;
  if (index == AANTAL_GEMIDDELD)
    index = 0;

  // Bereken gemiddelde
  for (int i = 0; i < AANTAL_GEMIDDELD; i++) {
    gemiddeldeHoek += metingen[i];
  }

  gemiddeldeHoek /= AANTAL_GEMIDDELD;
  Serial.print(90);
  Serial.print(" ");
  Serial.print(-90);
  Serial.print(" ");
  Serial.print(gemiddeldeHoek);

  /**
   * Uitmiddelen
   */


  /**
   * 
   */
  delay(2);  // f = 1/0.002

  Serial.println();
  prev = now;
}

int PID(double curVal) {
  static double priError;
  static double toError;

  /**
   * Set-waarde om naar te streven voor de PID regelaar
   */
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

/***
 * Bestuurt de X-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorX(int16_t dc) {

  int MAX = 200;
  if (dc < -MAX)
    dc = -MAX;
  if (dc > MAX)
    dc = MAX;

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

float getPitch(float ax, float ay, float az) {
  float pitch = atan(ax / sqrt(ay * ay + az * az));
  pitch *= 180.0 / PI;
  return pitch;
}

float getRoll() {
  float ax, ay, az;

  if (!IMU.accelerationAvailable())
    return 1000;  // fout

  IMU.readAcceleration(ax, ay, az);

  float roll = atan(ay / sqrt(ax * ax + az * az));
  roll *= 180.0 / PI;
  return roll;
}