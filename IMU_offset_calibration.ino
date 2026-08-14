#include <Wire.h>


const uint8_t MPU_ADDR = 0x68;

// Calibration Offsets
float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;
float accOffsetX = 0.0;
float accOffsetY = 0.0;
float accOffsetZ = 0.0;

// Function prototype to read raw sensor bytes
void readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  uint8_t axH = Wire.read(); uint8_t axL = Wire.read();
  uint8_t ayH = Wire.read(); uint8_t ayL = Wire.read();
  uint8_t azH = Wire.read(); uint8_t azL = Wire.read();
  Wire.read(); Wire.read(); // Skip temp bytes
  uint8_t gxH = Wire.read(); uint8_t gxL = Wire.read();
  uint8_t gyH = Wire.read(); uint8_t gyL = Wire.read();
  uint8_t gzH = Wire.read(); uint8_t gzL = Wire.read();

  ax = (int16_t)(axH << 8 | axL);
  ay = (int16_t)(ayH << 8 | ayL);
  az = (int16_t)(azH << 8 | azL);
  gx = (int16_t)(gxH << 8 | gxL);
  gy = (int16_t)(gyH << 8 | gyL);
  gz = (int16_t)(gzH << 8 | gzL);

}

const int LED_PIN = 2;

void calibrateSensors(int samples = 1000) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,HIGH); // LED ON : keep the IMU still !
  Serial.println("Calibrating IMU... DO NOT MOVE THE SENSOR!");
  delay(5000); // 5 second pause to settle on desk


  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumAx = 0, sumAy = 0, sumAz = 0;

  for (int i = 0; i < samples; i++) {
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    // Sum physical values (with good units)
    sumAx += (rawAx / 4096.0);
    sumAy += (rawAy / 4096.0);
    sumAz += (rawAz / 4096.0);

    sumGx += (rawGx / 16.4);
    sumGy += (rawGy / 16.4);
    sumGz += (rawGz / 16.4);

    delay(2); // ~500Hz rate 
    }


// Calculate mean offsets
  accOffsetX = sumAx / samples;
  accOffsetY = sumAy / samples;
  accOffsetZ = (sumAz / samples) - 1.0; // Subtract 1g for Earth's gravity

  gyroOffsetX = sumGx / samples;
  gyroOffsetY = sumGy / samples;
  gyroOffsetZ = sumGz / samples;

digitalWrite(LED_PIN, LOW); // LED OFF = Calibration done!
Serial.println("Calibration complete. Loop starting!");


}

void setup() {

Serial.begin(115200);

// Fast I2C Bus setup (400kHz)
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

// Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Enable Digital Low Pass Filter (DLPF ~42Hz bandwidth)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  // Set ranges: Gyro +/-2000 deg/s, Accel +/-8g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  // Run the 1000-sample calibration in setup
  calibrateSensors(1000);

}

void loop() {
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

  // Apply calibration offsets to get clean zero-centered readings
  float cleanAccX = (rawAx / 4096.0) - accOffsetX;
  float cleanAccY = (rawAy / 4096.0) - accOffsetY;
  float cleanAccZ = (rawAz / 4096.0) - accOffsetZ;

  float cleanGyroX = (rawGx / 16.4) - gyroOffsetX;
  float cleanGyroY = (rawGy / 16.4) - gyroOffsetY;
  float cleanGyroZ = (rawGz / 16.4) - gyroOffsetZ;

  // Format for Serial Plotter / Monitor
  Serial.print("GyroX:"); Serial.print(cleanGyroX, 2); Serial.print(",");
  Serial.print("GyroY:"); Serial.print(cleanGyroY, 2); Serial.print(",");
  Serial.print("GyroZ:"); Serial.println(cleanGyroZ, 2);

  delay(20);

}
