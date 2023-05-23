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
#include <SparkFunLSM9DS1.h>

#include <Encoder.h>

#define DEBUG 0

#define PWM_FREQ 0x03
#define PIN_PWM_X 9
#define PIN_X_LEFT 7
#define PIN_X_RIGHT 6

#define DECLINATION -8.58

double PID(double curVal);
double PID2(double curVal);
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);
float getGyro();
float getPitch(float ax, float ay, float az);
float getRoll();
float getAvgRoll();

// Uitmiddelen van sensor
#define AANTAL_GEMIDDELD 2
float metingen[AANTAL_GEMIDDELD];

LSM9DS1 imu;
Encoder motor(2,3);

// PID voor opslingeren
const float kp = 8, /*35*/ ki = 0.00, kd = 8;
// PID voor stabiliseren
const float kp2 = 105, ki2 = 0, kd2 = 25;

float x=0.0, P=1.0, Q=0.01, R=0.1;

void setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.begin(115200);

  Wire.begin();

  // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  if (imu.begin() == false) {
    Serial.println("Failed to communicate with LSM9DS1.");
    //while (1);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////

  /* Pin Modes */
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_X_LEFT, OUTPUT);
  pinMode(PIN_X_RIGHT, OUTPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Calibreer rustpunt
   */
  // Bepaal hoek
  /*
  setPoint = getRoll();
  Serial.print("New Setpoint is: ");
  Serial.println(setPoint);
  */
  //imu.calibrate();

  // Motor Encoder
  motor.write(0);
}

/**
 * @brief Regellus algorithme
 */
void regelDit() {
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

  /*
  huidigeHoek = getAvgRoll();
  // Als er geen hoek is, skip deze iteratie/10
  if (huidigeHoek == 999)
    return;*/

  // Op basis van hoek.
  if (!imu.accelAvailable())
    return;
  imu.readAccel();

  /*
  Serial.print(" \tHoogte: ");
  Serial.print(imu.ax);
  */
  
  currentAcceleration = getGyro();
  if (currentAcceleration == 999)
      return;  // Fout, skip deze iteratie

      /*
      -4000 = -180
      -2000 = -90
      0     = 0
      2000  = 90
      4000 = 180
    */
  int hoekFout = motor.read();

  // Kalman filter
  /*
  P = P + Q;
  float K = P / (P + R);
  x = x + K *(hoekFout - x);
  P = (1 - K) * P;
  hoekFout = x;
  */

  double graden =  ((double)hoekFout / 4096 * 180) ;
  Serial.print("Hoek: ");
  Serial.print(graden);

  /*
  Serial.print("90 -90 ");
  Serial.print(" \tCurrentAcceleration: ");
  Serial.print(" ");
  Serial.print(currentAcceleration);
  */
  
  if (abs(graden) > 90) {

    output = PID(currentAcceleration);
    Serial.print(" \tPID1 output: ");
    Serial.print(output);

    huidigePWM = -output;

  } else if (abs(graden) < 25) {
    // Wanneer we boven zitten, gebruik andere PID regelaar

    //int hoekFout = getRoll();

    //Serial.print(" \tHoekfout: ");
    Serial.print(" ");
    Serial.print(graden);

    input = -graden;//-(graden + 0.8 * currentAcceleration);
    //input = currentAcceleration;

    //Serial.print(" \t\tPID2 input: ");
    Serial.print(" ");
    Serial.print(input);

    output = PID2(input);
    //Serial.print(" \tPID2 output: ");
    Serial.print(" ");
    Serial.print(output);

    huidigePWM = output; //- 20;
  }

  //Serial.print(" \tHuidigePWM: ");
  /*
  Serial.print(" ");
  Serial.print(huidigePWM);
*/
  Serial.println();

  setMotorX(huidigePWM);
  /*
  if (abs(huidigeHoek) > abs(grootsteHoek) && abs(huidigeHoek) > StootHoek) {
    stootGegeven = false;
    grootsteHoek = huidigeHoek;

    Serial.print(" \tHuidigeHoek: ");
    Serial.print(huidigeHoek);

    // Bereken aan te leggen versnelling bij grootste hoek
    output = pid.Run(huidigeHoek);
    //output = PID(huidigeHoek);
    Serial.print(" \tPID output: ");
    Serial.print(output);

    // Vergelijk met hudige.
    // Verschil bepaalt in te stellen PWM?
    Serial.print(" \tHuidigePWM: ");
    Serial.print(huidigePWM);
    //huidigePWM = output * 5;
    huidigePWM += output;  // Tel op?

    // Begrens PWM tot max
    if (huidigePWM > 255)
      huidigePWM = 255;
    else if (huidigePWM < -255)
      huidigePWM = -255;

    Serial.print(" \tNieuwe PWM: ");
    Serial.print(huidigePWM);
    Serial.println();
  }*/

  /*
    Reset als tussen [-10, 10] graden
    MAAR HIER GEVEN WE OOK DE STOOT!
    BUG: Doe dit slechts 1 keer tot reset!!
  */
  /*
  if (abs(huidigeHoek) < StootHoek && !stootGegeven) {
    setMotorX(huidigePWM);
    Serial.print("Stoot");
    Serial.println(huidigePWM);

    grootsteHoek = 0;
    stootGegeven = true;
  }*/
}

void loop() {
  /**
   * Regel systeem aan de hand van de uitgelezen waardes
   */
  regelDit();
}

double PID(double curVal) {
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

  // int Fvalue = (int)PIDvalue;

  // Fvalue = map(Fvalue, -90, 90, -100, 100);
  // return Fvalue;
}

double PID2(double curVal) {
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
void setMotorX(int16_t dc) {
  int MAX = 255;
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

float getGyro() {
  if (!imu.gyroAvailable())
    return 999;

  imu.readGyro();
  return imu.calcGyro(imu.gz) / 10;
}

float getRoll() {
  if (!imu.accelAvailable())
    return 999;  // fout

  imu.readAccel();

  float roll = atan(imu.calcAccel(imu.ay) / sqrt(imu.calcAccel(imu.ax) * imu.calcAccel(imu.ax) + imu.calcAccel(imu.az) * imu.calcAccel(imu.az)));
  roll *= 180.0 / PI;
  return roll;
}

/**
 * Berekent de gemiddelde hoek
 */
float getAvgRoll() {
  float gemiddeldeHoek = 0;

  static int index;
  metingen[index] = getRoll();
  if (metingen[index] == 999)
    return 999;

  index++;
  if (index == AANTAL_GEMIDDELD)
    index = 0;

  // Bereken gemiddelde
  for (int i = 0; i < AANTAL_GEMIDDELD; i++) {
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