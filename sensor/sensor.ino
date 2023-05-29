/*
  Inverted Pendulum
*/

/**
 * Header includes
 */
#include <stdio.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/**
 * Functie prototypes
 */
double PID(double curVal);
double PID2(double curVal);
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);
float getGyro();
float getPitch(float ax, float ay, float az);
float getRoll();
float getAvgRoll();

// Gebruik Serial output voor informatie
#define DEBUG 0

/**
 * Arduino Uno pinnummers
 */
#define PIN_PWM_X 9
#define PIN_X_LEFT 7
#define PIN_X_RIGHT 6
#define PIN_ENC_A 2
#define PIN_ENC_B 3

// Uitmiddelen van sensor
#define AANTAL_GEMIDDELD 2
float metingen[AANTAL_GEMIDDELD];

// IMU klasse
LSM9DS1 imu;
// Encoder klasse op poort
Encoder motor(PIN_ENC_A, PIN_ENC_B);

// PID voor opslingeren
const float kp = 8, /*35*/ ki = 0.00, kd = 8;
// PID voor stabiliseren
const float kp2 = 105, ki2 = 0, kd2 = 25;

void setup()
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.begin(115200);

  Wire.begin();

  // With no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  if (imu.begin() == false)
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    // while (1);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////

  /* Pin Modes */
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_X_LEFT, OUTPUT);
  pinMode(PIN_X_RIGHT, OUTPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Motor Encoder
  motor.write(0);
}

void loop()
{
  // Huidige hoek gemeten door sensor
  float currentAngle, currentAcceleration;
  double input, output;
  const int StootHoek = 5;

  // Grootste hoek gemeten tot nu toe
  static float grootsteHoek;
  // De huidige PWM die aangelegd wordt aan de motor
  static int huidigePWM;
  static int stoot;
  static bool stootGegeven;

  currentAcceleration = getGyro();
  if (currentAcceleration == 999)
  {
// Er is een fout opgetreden bij het uitlezen van de IMU, skip deze iteratie
#if DEBUG
    Serial.println("Error reading acceleration from IMU!");
#endif
    return;
  }

  // Lees hoek uit de encoder
  int hoekFout = motor.read();

  double graden = ((double)hoekFout / 4096 * 180);
#if DEBUG
  Serial.print("\t ");
  Serial.print(graden);
#endif

  if (abs(graden) > 90)
  {

    output = PID(currentAcceleration);
#if DEBUG
    Serial.print(" \tPID1 output: ");
    Serial.print(output);
#endif

    huidigePWM = -output;
  }
  else if (abs(graden) < 25)
  {
    // Wanneer we boven zitten, gebruik andere PID regelaar

    input = -graden; //-(graden + 0.8 * currentAcceleration);

    output = PID2(input);
#if DEBUG
    Serial.print(" \tPID2 output: ");
    Serial.print(output);
#endif

    huidigePWM = output; //- 20;
  }

#if DEBUG
  Serial.print(" \tHuidigePWM: ");
  Serial.println();
#endif

  // Stuur de berekende PWM naar de motor
  setMotorX(huidigePWM);
}

double PID(double curVal)
{
  static double priError;
  static double toError;

  /**
   * Set-waarde om naar te streven voor de PID regelaar
   */
  int setP = 0;

  double error = setP - curVal;

  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;

  return PIDvalue;
}

double PID2(double curVal)
{
  static double priError2;
  static double toError2;

  /**
   * Set-waarde om naar te streven voor de PID regelaar
   */
  int setP = 0;

  double error = setP - curVal;

  double Pvalue = error * kp2;
  double Ivalue = toError2 * ki2;
  double Dvalue = (error - priError2) * kd2;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError2 = error;
  toError2 += error;

  return PIDvalue;
}

/***
 * Bestuurt de X-as motor
 * @param dc Duty cycle van de motor, negatief = anti-clockwise, positief = clockwise
 */
void setMotorX(int16_t dc)
{
  int MAX = 255;
  if (dc < -MAX)
    dc = -MAX;
  if (dc > MAX)
    dc = MAX;

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

float getPitch(float ax, float ay, float az)
{
  float pitch = atan(ax / sqrt(ay * ay + az * az));
  pitch *= 180.0 / PI;
  return pitch;
}

float getGyro()
{
  if (!imu.gyroAvailable())
    return 999;

  imu.readGyro();
  return imu.calcGyro(imu.gz) / 10;
}

float getRoll()
{
  if (!imu.accelAvailable())
    return 999; // fout

  imu.readAccel();

  float roll = atan(imu.calcAccel(imu.ay) / sqrt(imu.calcAccel(imu.ax) * imu.calcAccel(imu.ax) + imu.calcAccel(imu.az) * imu.calcAccel(imu.az)));
  roll *= 180.0 / PI;
  return roll;
}

/**
 * Berekent de gemiddelde hoek
 */
float getAvgRoll()
{
  float gemiddeldeHoek = 0;

  static int index;
  metingen[index] = getRoll();
  if (metingen[index] == 999)
    return 999;

  index++;
  if (index == AANTAL_GEMIDDELD)
    index = 0;

  // Bereken gemiddelde
  for (int i = 0; i < AANTAL_GEMIDDELD; i++)
  {
    gemiddeldeHoek += metingen[i];
  }

  gemiddeldeHoek /= AANTAL_GEMIDDELD;

#if DEBUG
  Serial.print(90);
  Serial.print(" ");
  Serial.print(-90);
  Serial.print(" ");
  Serial.print(gemiddeldeHoek);
#endif
  return gemiddeldeHoek;
}