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

#define PWM_FREQ 0x03
#define PIN_PWM_X 9
#define PIN_X_LEFT 7
#define PIN_X_RIGHT 6

#define DEBUG 0

double PID(double curVal);
void setMotorX(int16_t dc);
void setMotorY(int16_t dc);
float getGyro();
float getMagnet();
float getPitch(float ax, float ay, float az);
float getRoll();
float getAvgRoll();

// Uitmiddelen van sensor
#define AANTAL_GEMIDDELD 2
float metingen[AANTAL_GEMIDDELD];

unsigned long prevTime = 0;
unsigned long curTime = 0;

/// @brief PID values
/*
Goeie waardes
const float kp = 3, ki = 0.00, kd = 10;
*/
const float kp = 10, ki = 0.00, kd = 5;

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
  /*
  setPoint = getRoll();
  Serial.print("New Setpoint is: ");
  Serial.println(setPoint);*/
}

/**
 * @brief Regellus algorithme
 */
void regelDit() {
  // Huidige hoek gemeten door sensor
  float currentAngle, currentAcceleration;
  double output;
  const int StootHoek = 5;

  // Grootste hoek gemeten tot nu toe
  static float grootsteHoek;
  // De huidige PWM die aangelegd wordt aan de motor
  static int huidigePWM;
  static int stoot;
  static bool stootGegeven;

  /**
   * Meting hoek - Roll
   * Links  = -90
   * Rechts = +90
   */
   /*
  huidigeHoek = getAvgRoll();
  // Als er geen hoek is, skip deze iteratie
  if (huidigeHoek == 999)
    return;*/
  currentAcceleration = getGyro() - 4.5;
  //currentAngle = getAvgRoll();

  float mx, my, mz;
  if (!IMU.magneticFieldAvailable())
    return;

/* TODO: Zoeken wanneer die omgekeerd is, tusesn twee hoeken minteken omdraaien*/
  IMU.readMagneticField(mx, my, mz);
  Serial.print(mx);
  Serial.print(" \t");
  Serial.print(my);
  Serial.print(" \t");
  Serial.print(mz);
  Serial.print(" \t");

  /*
  if (currentAngle == 999)
    return;*/

  /**
   * Update telkens de grootste huidige hoek EN als de hoek boven de RESETWAARDE is
   */
  Serial.print(" \tCurrentAngle: ");
  Serial.print(currentAngle);
  
  Serial.print(" \tCurrentAcceleration: ");
  Serial.print(currentAcceleration);

  output = PID(currentAcceleration);
  Serial.print(" \tPID output: ");
  Serial.print(output);

  /**
   * Factor aanpassing
   */
  // Opslingeren
  huidigePWM = -output;

  Serial.print(" \tHuidigePWM: ");
  Serial.print(huidigePWM);

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

  delay(1);  // f = 1/0.002 = 500Hz
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

float getMagnet() {
  float mx, my, mz;

  if (!IMU.magneticFieldAvailable())
    return 999;
  
  IMU.readMagneticField(mx, my, mz);
  Serial.print((int)mx);
  Serial.print(" \t");
  Serial.print((int)my);
  Serial.print(" \t");
  Serial.print((int)mz);
  Serial.print(" \t");
}

float getGyro() {
  float ax, ay, az;

  if (!IMU.gyroscopeAvailable())
    return 999;
  
  IMU.readGyroscope(ax, ay, az);
  return az;
}

float getRoll() {
  float ax, ay, az;

  if (!IMU.accelerationAvailable())
    return 999;  // fout

  IMU.readAcceleration(ax, ay, az);

  float roll = atan(ay / sqrt(ax * ax + az * az));
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