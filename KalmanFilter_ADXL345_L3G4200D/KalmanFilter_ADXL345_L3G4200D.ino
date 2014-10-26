/*
    Kalman Filter Example for ADXL345 & L3G4200D. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <L3G4200D.h>
#include <ADXL345.h>
#include <KalmanFilter.h>

L3G4200D gyroscope;
ADXL345 accelerometer;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup() 
{
  Serial.begin(115200);

  // Initialize ADXL345
  while(!accelerometer.begin())
  {
    delay(500);
  }
 
  accelerometer.setRange(ADXL345_RANGE_2G);
  
  // Initialize L3G4200D
  while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    delay(500);
  }
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate(100);
}

void loop()
{
  Vector acc = accelerometer.readNormalize();
  Vector gyr = gyroscope.readNormalize();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  Serial.print(accPitch);
  Serial.print(":");
  Serial.print(accRoll);
  Serial.print(":");
  Serial.print(kalPitch);
  Serial.print(":");
  Serial.print(kalRoll);
  Serial.print(":");
  Serial.print(acc.XAxis);
  Serial.print(":");
  Serial.print(acc.YAxis);
  Serial.print(":");
  Serial.print(acc.ZAxis);
  Serial.print(":");
  Serial.print(gyr.XAxis);
  Serial.print(":");
  Serial.print(gyr.YAxis);
  Serial.print(":");
  Serial.print(gyr.ZAxis);

  Serial.println();
}


