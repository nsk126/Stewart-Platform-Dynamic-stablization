#include<Wire.h>
#include<Kalman.h>
#include <Adafruit_PWMServoDriver.h>
#include <Quaternion.h>
#include <math.h>

#define RESTRICT_PITCH

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanX1;
Kalman kalmanY1;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

const float topm = 7.727;
const float basem = 10.3748;

//Unit norm quaternion - for all rotations
Quaternion q;

#define A1 0
#define A2 9.95*M_PI/180
#define A3 120*M_PI/180
#define A4 129.95*M_PI/180
#define A5 240*M_PI/180
#define A6 249.95*M_PI/180

const float bk[6][3] = {
  {basem * cos(A1), basem * sin(A1), 0},
  {basem * cos(A2), basem * sin(A2), 0},
  {basem * cos(A3), basem * sin(A3), 0},
  {basem * cos(A4), basem * sin(A4), 0},
  {basem * cos(A5), basem * sin(A5), 0},
  {basem * cos(A6), basem * sin(A6), 0},
};

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);
  
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

  for (int i = 0; i < 6; i++) {
    pwm.writeMicroseconds(i, 1500);
  }
  Serial.println("\r\n\n Init .... ! \n\n\n");
  delay(4000);

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

  Serial.print("1 ");
  Serial.print(kalAngleX);Serial.print(" ");
  Serial.print(kalAngleY);Serial.print(" ");
  Serial.print("2 ");
  Serial.print(kalAngleX1);Serial.print(" ");
  Serial.print(kalAngleY1);Serial.print(" ");

  /*
   * FEEDBACK
   * P Controller
   * 
   */

  float error_X = kalAngleX + kalAngleX1;
  float error_Y = kalAngleY + kalAngleY1;
  Serial.print(String(error_X)+" ");
  Serial.println(String(error_Y)+" ");
  
  

  /*
   * INVERSE KINEMATICS
   */

//  float heave = mapfloat(analogRead(0),0,1023,9.00,14.25);
  float s_heave = 9.8;//midpoint

  //  float surge = mapfloat(analogRead(0),0,1023,-4.0,4.0);
  float s_surge = 0;

  float s_sway = 0;


  //  float s_roll = mapfloat(analogRead(0),0,1023,-21.0,21.0);
  float s_roll = 1.1*error_Y;

  //  float s_pitch = mapfloat(analogRead(0),0,1023,-21.0,21.0);
  float s_pitch = 1.1*error_X;

  //  float s_yaw = mapfloat(analogRead(0),0,1023,-37.5,37.5);
  float s_yaw = 0;


  float T[3] = {s_surge, s_sway, s_heave};
  float R[3] = {s_roll * M_PI / 180, s_pitch * M_PI / 180, s_yaw * M_PI / 180};

  float qk[6][3] = {
    { -cos(A1)*topm + T[0], -sin(A1)*topm + T[1], T[2]},
    { -cos(A2)*topm + T[0], -sin(A2)*topm + T[1], T[2]},
    { -cos(A3)*topm + T[0], -sin(A3)*topm + T[1], T[2]},
    { -cos(A4)*topm + T[0], -sin(A4)*topm + T[1], T[2]},
    { -cos(A5)*topm + T[0], -sin(A5)*topm + T[1], T[2]},
    { -cos(A6)*topm + T[0], -sin(A6)*topm + T[1], T[2]},
  };

  float pk[6][3];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      pk[i][j] = qk[i][j] - T[j];
    }
  }

  Quaternion pk1(pk[0][0], pk[0][1], pk[0][2]);
  Quaternion pk2(pk[1][0], pk[1][1], pk[1][2]);
  Quaternion pk3(pk[2][0], pk[2][1], pk[2][2]);
  Quaternion pk4(pk[3][0], pk[3][1], pk[3][2]);
  Quaternion pk5(pk[4][0], pk[4][1], pk[4][2]);
  Quaternion pk6(pk[5][0], pk[5][1], pk[5][2]);

  q = q.from_euler_rotation(R[0], R[1], R[2]);

  pk1 = q.rotate(pk1);
  pk2 = q.rotate(pk2);
  pk3 = q.rotate(pk3);
  pk4 = q.rotate(pk4);
  pk5 = q.rotate(pk5);
  pk6 = q.rotate(pk6);

  /*
     Considering our earlier statement
     L1 = P4 - B1
     L2 = P5 - B2
     L3 = P6 - B3
     L4 = P1 - B4
     L5 = P2 - B5
     L6 = P3 - B6
  */

  float lk[6][3] = {{
      T[0] + pk4.b - bk[0][0],
      T[1] + pk4.c - bk[0][1],
      T[2] + pk4.d - bk[0][2],
    }, {
      T[0] + pk5.b - bk[1][0],
      T[1] + pk5.c - bk[1][1],
      T[2] + pk5.d - bk[1][2],
    }, {
      T[0] + pk6.b - bk[2][0],
      T[1] + pk6.c - bk[2][1],
      T[2] + pk6.d - bk[2][2],
    }, {
      T[0] + pk1.b - bk[3][0],
      T[1] + pk1.c - bk[3][1],
      T[2] + pk1.d - bk[3][2],
    }, {
      T[0] + pk2.b - bk[4][0],
      T[1] + pk2.c - bk[4][1],
      T[2] + pk2.d - bk[4][2],
    }, {
      T[0] + pk3.b - bk[5][0],
      T[1] + pk3.c - bk[5][1],
      T[2] + pk3.d - bk[5][2],
    }
  };

  /*
     Debug
  */

  //  for (int i = 0; i < 6; i++) {
  //    for (int j = 0; j < 3; j++) {
  //      Serial.print(String(lk[i][j]) + " ");
  //    } Serial.println(); Serial.println();
  //  }

  float len[6];
  for (int i = 0; i < 6; i++) {
    len[i] = norm(lk[i]) - 10.2;
    //    Serial.println(len[i]);
  }

  /* Actuator closed length = 102mm
      Stroke length = 50mm
      This whole file operates calcualations in cm.
      Therefore all norm outputs are to be calculated between 10.2cm and 15.2cm
      As implied -> stroke length(cm) = norm_output(cm) - 10.2cm;
      And stroke ->  0 - 5cm --> 1000uS to 2000uS
  */

  int pulses[6];
  for (int i = 0; i < 6; i++) {
    pulses[i] = mapfloat(len[i], 0.0, 5.0, 1000, 2000);
    //    Serial.print(String(pulses[i])+"  ");
    pwm.writeMicroseconds(i, pulses[i]);
  }//Serial.println();

  
  
}

float norm(float vector[3]) {
  return sqrt( sq(vector[0]) + sq(vector[1]) + sq(vector[2]));
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
