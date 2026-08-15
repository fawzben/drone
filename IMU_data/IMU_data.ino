#include <Wire.h>

// MPU6500 I2C Address (AD0 disconnected/high = 0x68 or 0x69 depending on breakout pull-up)
const uint8_t MPU_ADDR = 0x68;

// Raw sensor values
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;

// Scaled sensor values (g-force & degrees per second)
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(115200);

  // 1. Fast I2C Bus setup (400kHz Fast-Mode for low latency)
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

  // 2. Wake up MPU6500 (Write 0 to PWR_MGMT_1 register 0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // 3. Set Gyro Range to +/- 2000 deg/s (Register 0x1B -> 0x18)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  // 4. Set Accel Range to +/- 8g (Register 0x1C -> 0x10)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  // 5. Enable Digital Low Pass Filter (DLPF) ~42Hz bandwidth (Register 0x1A -> 0x03)
  // Reduces high-frequency motor vibration noise on the drone frame
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);  // 0x03 = 42Hz, 0x04 = 20Hz, 0x05 = 10Hz, 0x06 = 5Hz
  Wire.endTransmission(true);

  Serial.println("MPU6500 Configured & Ready for Flight Loop!");
}

void loop() {
  // Burst read 14 bytes: 6 Accel + 2 Temp + 6 Gyro bytes in ONE transfer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t) true);

  // Parse 16-bit signed integers (High Byte << 8 | Low Byte)
  rawAccX = Wire.read() << 8 | Wire.read();
  rawAccY = Wire.read() << 8 | Wire.read();
  rawAccZ = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();  // Skip Temp High & Low bytes

  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw LSB to physical values
  // Accelerometer: +/-8g range = 4096 LSB/g
  accX = rawAccX / 4096.0;
  accY = rawAccY / 4096.0;
  accZ = rawAccZ / 4096.0;

  // Gyroscope: +/-2000 deg/s range = 16.4 LSB/(deg/s)
  gyroX = rawGyroX / 16.4;
  gyroY = rawGyroY / 16.4;
  gyroZ = rawGyroZ / 16.4;

  // // Print scaled telemetry (deg/s and g)
  // Serial.print("Gyro (deg/s) - X: "); Serial.print(gyroX, 1);
  // Serial.print(" | Y: "); Serial.print(gyroY, 1);
  // Serial.print(" | Z: "); Serial.print(gyroZ, 1);
  // Serial.print(" || Acc (g) - Z: "); Serial.println(accZ, 2);

  // --- [Serial Plotter Format] ---
  // Lock the Y-axis range between -50 and +50 deg/s
  // Serial.print("Min:-50,Max:50,");
  // // Print "Label:Value" separated by commas, ending with println
  // Serial.print("GyroX:");
  // Serial.print(gyroX);
  // Serial.print(",");

  // Serial.print("GyroY:");
  // Serial.print(gyroY);
  // Serial.print(",");

  // Serial.print("GyroZ:");
  // Serial.print(gyroZ);  // println signals the end of the data point!
  // Serial.print(",");

Serial.print("Min:-10,Max:10,");
  Serial.print("Acc(g):");
  Serial.println(accZ);



  delay(10);  // ~100Hz (250Hz loop rate is actually ideal baseline for basic quadcopter PID control, so 4ms !)
}
