#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <encoder_real.h>
#include <mpu_encoder.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

//int direction1 = 0;
//int direction2 = 0;
float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void intitialize_mpu_encoder(){
  initialize_encoder();
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
}

float roll(){
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter
  kalRoll = kalmanX.update(accRoll, gyr.XAxis)-8;
  return  kalRoll;
}
float pitch(){
   Vector acc = mpu.readNormalizeAccel();
   Vector gyr = mpu.readNormalizeGyro();
   accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
    kalPitch = kalmanY.update(accPitch, gyr.YAxis)+7;
    return  (kalPitch);
}



