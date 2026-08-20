#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const int LED_PIN = 2;

// Calibration Offsets
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;
float accOffsetX = 0.0, accOffsetY = 0.0, accOffsetZ = 0.0;

// Filtered Angles
float roll = 0.0;
float pitch = 0.0;

// Timing variables (for strict non-blocking loop)
unsigned long previousMicros = 0; // for micros() fct to count time since beginning
const unsigned long LOOP_TIME_US = 4000; // 4microseconds = 4ms (250Hz loop rate)
float dt = 0.004; // 4ms in seconds

void readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false); // false keeps the connection to the I2C active
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true); // release the bus (true) once received


// Reads the 14 incoming 8bit bytes in order
uint8_t axH = Wire.read(); uint8_t axL = Wire.read(); // High byte - Low byte
  uint8_t ayH = Wire.read(); uint8_t ayL = Wire.read();
  uint8_t azH = Wire.read(); uint8_t azL = Wire.read();
  Wire.read(); Wire.read(); // Skip temperature bytes
  uint8_t gxH = Wire.read(); uint8_t gxL = Wire.read();
  uint8_t gyH = Wire.read(); uint8_t gyL = Wire.read();
  uint8_t gzH = Wire.read(); uint8_t gzL = Wire.read();


//combination of packs of 8bits into 16bits
ax = (int16_t)(axH << 8 | axL);
ay = (int16_t)(ayH << 8 | ayL);
az = (int16_t)(azH << 8 | azL);
gx = (int16_t)(gxH << 8 | gxL);
gy = (int16_t)(gyH << 8 | gyL);
gz = (int16_t)(gzH << 8 | gzL);

}

void calibrateSensors(int samples = 500) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(1500);

  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumAx = 0, sumAy = 0, sumAz = 0;

  for (int i = 0; i < samples; i++) {
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    sumAx += (rawAx / 4096.0);
    sumAy += (rawAy / 4096.0);
    sumAz += (rawAz / 4096.0);

    sumGx += (rawGx / 16.4);
    sumGy += (rawGy / 16.4);
    sumGz += (rawGz / 16.4);

    delay(2);
  }

  accOffsetX = sumAx / samples;
  accOffsetY = sumAy / samples;
  accOffsetZ = (sumAz / samples) - 1.0;

  gyroOffsetX = sumGx / samples;
  gyroOffsetY = sumGy / samples;
  gyroOffsetZ = sumGz / samples;

  digitalWrite(LED_PIN, LOW);
}

void setup() {

Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // DLPF 42Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  // Gyro +/-2000 deg/s, Acc +/-8g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  calibrateSensors(500);

// Initialize angles with initial accelerometer readings
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  float initAccX = (rawAx / 4096.0) - accOffsetX;
  float initAccY = (rawAy / 4096.0) - accOffsetY;
  float initAccZ = (rawAz / 4096.0) - accOffsetZ;

roll = atan2(initAccY, initAccZ) * (180.0 / PI); // physics in Dynamics.md // atan2(a,b) = arctan(a/b)
pitch = atan2(-initAccX, sqrt(initAccY * initAccY + initAccZ * initAccZ)) * (180.0 / PI);
// not arcsin(a_x/g) for multiple non-practical reasons

previousMicros = micros(); // initialises at 0 microseconds

}

void loop() {
unsigned long currentMicros = micros(); // actualises clock at each loop 
//micros() doesn't freeze code like delay()

// Execute flight loop strictly every 4000 microseconds (250Hz)
  if (currentMicros - previousMicros >= LOOP_TIME_US) {
    // Calculate actual delta time in seconds
    dt = (currentMicros - previousMicros) / 1000000.0;
    previousMicros = currentMicros;

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

      // 1. Scaled & calibrated values
      float accX = (rawAx / 4096.0) - accOffsetX;
      float accY = (rawAy / 4096.0) - accOffsetY;
      float accZ = (rawAz / 4096.0) - accOffsetZ;

      float gyroX = (rawGx / 16.4) - gyroOffsetX;
      float gyroY = (rawGy / 16.4) - gyroOffsetY;
      float gyroZ = (rawGz / 16.4) - gyroOffsetZ;

      // 2. Compute absolute angles from Accelerometer (in degrees)
    float accRoll = atan2(accY, accZ) * (180.0 / PI);
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * (180.0 / PI);

    // 3. Complementary Filter Formula
    // Pitch/Roll gyro mapping: Roll rate is gyroX, Pitch rate is gyroY
    roll = 0.98 * (roll + gyroX * dt) + 0.02 * accRoll;
    pitch = 0.98 * (pitch + gyroY * dt) + 0.02 * accPitch;

  // 4. Output for Serial Plotter
      Serial.print("Min:-180,Max:180,");
      Serial.print("Roll:"); Serial.print(roll, 2); Serial.print(",");
      Serial.print("Pitch:"); Serial.print(pitch, 2); Serial.print(",");
      Serial.print("AccRoll:"); Serial.print(accRoll, 2); Serial.print(",");
      Serial.print("AccPitch:"); Serial.println(accPitch, 2);
  }
}
