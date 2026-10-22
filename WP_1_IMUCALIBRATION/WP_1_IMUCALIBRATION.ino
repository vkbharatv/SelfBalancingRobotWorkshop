#include <Wire.h>
#include <math.h>
/*

  IMU Calibration and Fusion
  Bord: NodeMCU ESP8266
  By: Dr Bharat Verma
  Date: 22/10/2026
  Workshop Day 1 Topic: IMU Calibration and Fusion using MPU-6050
  Lab: Control System Lab, ECE Department, The LNMIIT, Jaipur, India
  Society: RAS IEEE Student Branch, The LNMIIT, Jaipur, India
  Description: This code demonstrates the calibration and fusion of data from the MPU-6050 IMU.
  Exercise: Calibrate the gyroscope and fuse accelerometer and gyroscope data to compute roll and pitch angles. Afterthat, try different sensitivity settings and complimentary filter coefficients to observe their effects on the output angles.
  Outcome: Participants should complete the calibration and fusion process, and understand how to adjust parameters for optimal performance.

*/

// MPU-6050 Standard I2C Address (0x68)
const uint8_t MPU6050_ADDR = 0x68;

// Register Addresses
const uint8_t REG_CONFIG       = 0x1A; // Low Pass Filter
const uint8_t REG_GYRO_CONFIG  = 0x1B; // Gyro Scale
const uint8_t REG_ACCEL_CONFIG = 0x1C; // Accel Scale
const uint8_t REG_PWR_MGMT_1   = 0x6B; // Power Management
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // Start of Data Registers

// Gyro Calibration Offsets
float gyroX_offset = 0.0, gyroY_offset = 0.0, gyroZ_offset = 0.0;

// Fused Angles & Time Tracking
float roll = 0.0, pitch = 0.0;
unsigned long prevTime = 0;

void calibrateGyro() {
  Serial.println("Calibrating Gyroscope... Keep NodeMCU still!");
  int samples = 500;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // Start reading Gyro registers
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)6, (uint8_t)true);

    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    gyroX_offset += rawGx / 131.0;
    gyroY_offset += rawGy / 131.0;
    gyroZ_offset += rawGz / 131.0;
    delay(3);
  }

  gyroX_offset /= samples;
  gyroY_offset /= samples;
  gyroZ_offset /= samples;
  Serial.println("Calibration Complete!");
}

void setup() {
  Serial.begin(115200);

  // 1. Initialize I2C Pins for NodeMCU (D2 = SDA, D1 = SCL)
  Wire.begin(D2, D1);

  // 2. Set I2C Clock to STANDARD MODE (100 kHz)
  Wire.setClock(100000); 

  // 3. Wake up MPU-6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // 4. Configure Digital Low-Pass Filter (DLPF) to ~42Hz Bandwidth
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(0x00); 
  Wire.endTransmission();

  // 5. Set Accelerometer (+/- 2g) and Gyroscope (+/- 250 dps) Ranges
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_CONFIG);
  Wire.write(0x00); 
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(0x00); 
  Wire.endTransmission();

  // 6. Zero-rate Gyro Calibration
  calibrateGyro();

  prevTime = millis();
}

void loop() {
  // Point to starting register (0x3B)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  // Request 14 Bytes via I2C Standard Mode
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() >= 14) {
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip Temp bytes
    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    // Convert raw values to scale (+/-2g = 16384 LSB/g, +/-250 dps = 131 LSB/dps)
    float Ax = rawAx / 16384.0;
    float Ay = rawAy / 16384.0;
    float Az = rawAz / 16384.0;

    float gyroX = (rawGx / 131.0) - gyroX_offset;
    float gyroY = (rawGy / 131.0) - gyroY_offset;

    // Calculate time elapsed (dt)
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Static Accelerometer angles
    float accelPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * (180.0 / M_PI);
    float accelRoll  = atan2(-Ay, sqrt(Ax * Ax + Az * Az)) * (180.0 / M_PI);

    // Complementary Filter Fusion (96% Gyro, 4% Accel)
    float alpha = 0.96;
    pitch = alpha * (pitch + gyroY * dt) + (1-alpha) * accelPitch;
    roll  = alpha * (roll  + gyroX * dt) + (1-alpha) * accelRoll;

    // Output to Serial Monitor
    Serial.print(roll, 2);  Serial.print(",");
    Serial.println(pitch, 2);
  }

  delay(10); // 50 Hz loop execution rate
}