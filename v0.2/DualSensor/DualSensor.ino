#include<Wire.h>
#include<Kalman.h>

#define RESTRICT_PITCH

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanX1;
Kalman kalmanY1;

uint8_t IMU = 0x68;
uint8_t IMU1 = 0x69;
const uint16_t I2C_TIMEOUT = 1000;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

double accX1, accY1, accZ1;
double gyroX1, gyroY1, gyroZ1;
int16_t tempRaw1;

double gyroXangle1, gyroYangle1; // Angle calculate using the gyro only
double compAngleX1, compAngleY1; // Calculated angle using a complementary filter
double kalAngleX1, kalAngleY1; // Calculated angle using a Kalman filter

uint32_t timer;
uint32_t timer1;
uint8_t i2cData[14]; // Buffer for I2C data


void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(IMU, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(IMU, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(IMU, 0x75, i2cData, 1));
//  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
//    Serial.print(F("Error reading sensor"));
//    while (1);
//  }
  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(IMU, 0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();

  /*
     2nd Acc
  */

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(IMU1, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(IMU1, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(IMU1, 0x75, i2cData, 1));
//  if (i2cData[0] != 0x69) { // Read "WHO_AM_I" register
//    Serial.print(F("Error reading sensor"));
//    while (1);
//  }
  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(IMU1, 0x3B, i2cData, 6));
  accX1 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY1 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ1 = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll1  = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double pitch1 = atan2(-accX1, accZ1) * RAD_TO_DEG;
#endif

  kalmanX1.setAngle(roll1); // Set starting angle
  kalmanY1.setAngle(pitch1);
  gyroXangle1 = roll1;
  gyroYangle1 = pitch1;
  compAngleX1 = roll1;
  compAngleY1 = pitch1;
  timer1 = micros();

}

void loop() {
  
  /* Update all the values */
  while (i2cRead(IMU,0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
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
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /*
   * 2nd Acc
   */

   /* Update all the values */
  while (i2cRead(IMU1,0x3B, i2cData, 14));
  accX1 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY1 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ1 = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw1 = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX1 = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY1 = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ1 = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt1 = (double)(micros() - timer1) / 1000000; // Calculate delta time
  timer1 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll1  = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double pitch1 = atan2(-accX1, accZ1) * RAD_TO_DEG;
#endif

  double gyroXrate1 = gyroX1 / 131.0; // Convert to deg/s
  double gyroYrate1 = gyroY1 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll1 < -90 && kalAngleX1 > 90) || (roll1 > 90 && kalAngleX1 < -90)) {
    kalmanX1.setAngle(roll1);
    compAngleX1 = roll1;
    kalAngleX1 = roll1;
    gyroXangle1 = roll1;
  } else
    kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt1); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX1) > 90)
    gyroYrate1 = -gyroYrate1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt1);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch1 < -90 && kalAngleY1 > 90) || (pitch1 > 90 && kalAngleY1 < -90)) {
    kalmanY1.setAngle(pitch1);
    compAngleY1 = pitch1;
    kalAngleY1 = pitch1;
    gyroYangle1 = pitch1;
  } else
    kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt1); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY1) > 90)
    gyroXrate1 = -gyroXrate1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt1); // Calculate the angle using a Kalman filter
#endif

  gyroXangle1 += gyroXrate1 * dt1; // Calculate gyro angle without any filter
  gyroYangle1 += gyroYrate1 * dt1;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX1 = 0.93 * (compAngleX1 + gyroXrate1 * dt1) + 0.07 * roll1; // Calculate the angle using a Complimentary filter
  compAngleY1 = 0.93 * (compAngleY1 + gyroYrate1 * dt1) + 0.07 * pitch1;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle1 < -180 || gyroXangle1 > 180)
    gyroXangle1 = kalAngleX1;
  if (gyroYangle1 < -180 || gyroYangle1 > 180)
    gyroYangle1 = kalAngleY1;

  Serial.print(kalAngleX);Serial.print("\t");
  Serial.print(kalAngleY);Serial.print("\t");
  Serial.print(kalAngleX1);Serial.print("\t");
  Serial.print(kalAngleY1);Serial.println("\n");
  
}


uint8_t i2cWrite(uint8_t IMUAddress, uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(IMUAddress, registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t IMUAddress, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t IMUAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
