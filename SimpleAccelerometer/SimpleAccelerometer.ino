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

#include <Arduino_LSM9DS1.h>

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Started");

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
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    float pitch = atan(ax/sqrt(ay*ay + az*az));
    float roll  = atan(ay/sqrt(ax*ax + az*az));

    // Convert everything from radians to degrees:
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;
    Serial.print(0);
    Serial.print(" ");
    Serial.print(90);
    //Serial.print(" ");
    //Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    
    /**
     * Roll rechts = 90
     * Evenwicht   = 0
     * Roll links  = -90
     */
  }
}
