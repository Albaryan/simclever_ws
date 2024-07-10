#include "mpu6500.h"
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

/* Mpu6500 object */
bfs::Mpu6500 imu;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  while (!imu.Read());

  accX=imu.accel_x_mps2();
  accY=imu.accel_y_mps2();
  accZ=imu.accel_z_mps2();
  
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer=micros();
}

void loop() {
  /* Check if data read */
  if (imu.Read()) {
  accX = imu.accel_x_mps2();
  accY = imu.accel_y_mps2();
  accZ = imu.accel_z_mps2();
  gyroX = imu.gyro_x_radps();
  gyroY = imu.gyro_y_radps();
  gyroZ = imu.gyro_z_radps();
    
      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();

       double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  
  double gyroXrate = gyroX  ; // Convert to deg/s
  double gyroYrate = gyroY ;// Convert to deg/s

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
    
    Serial.print(roll);
    Serial.print("|");

    Serial.print(pitch);
    Serial.print("#");
  }
}
