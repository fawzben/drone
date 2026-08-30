#include <Wire.h>
#include <Arduino.h>

const uint8_t MPU_ADDR = 0x68;
const int LED_PIN = 2;

// ==========================================
// PID CONTROLLER DEFINITION & PROTOTYPES
// ==========================================
struct PIDController {
  float Kp;
  float Ki;
  float Kd;
  float maxIntegral;
  float maxOutput;
  float integral;
  float prevMeasured;
};

// Forward function prototypes
float computePID(PIDController &pid, float target, float measured, float dt);
void resetPID(PIDController &pid);
void readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
void calibrateSensors(int samples);

// Global PID instances
PIDController pidRoll  = {1.3f, 0.04f, 0.03f, 100.0f, 400.0f};
PIDController pidPitch = {1.3f, 0.04f, 0.03f, 100.0f, 400.0f};
PIDController pidYaw   = {2.5f, 0.02f,  0.0f,  50.0f, 400.0f};

// Sensor Calibration Offsets
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;
float accOffsetX = 0.0, accOffsetY = 0.0, accOffsetZ = 0.0;

// Filtered Angles
float roll = 0.0;
float pitch = 0.0;

// Loop Timing
unsigned long previousMicros = 0;
const unsigned long LOOP_TIME_US = 4000; // 250Hz
float dt = 0.004;

// ==========================================
// IMPLEMENTATIONS
// ==========================================

float computePID(PIDController &pid, float target, float measured, float rate, float dt) {
  float error = target - measured;
  float pTerm = pid.Kp * error;

  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -pid.maxIntegral, pid.maxIntegral);
  float iTerm = pid.Ki * pid.integral;

  float dTerm = -pid.Kd * rate; // we simply add rate as parameter, and use gyroscope values 
  //(as derivative of pitch/roll is simply angular speed)

  float output = pTerm + iTerm + dTerm;
  return constrain(output, -pid.maxOutput, pid.maxOutput);
}

void resetPID(PIDController &pid) {
  pid.integral = 0.0f;
  pid.prevMeasured = 0.0f;
}

void readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  uint8_t axH = Wire.read(); uint8_t axL = Wire.read();
  uint8_t ayH = Wire.read(); uint8_t ayL = Wire.read();
  uint8_t azH = Wire.read(); uint8_t azL = Wire.read();
  Wire.read(); Wire.read();
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

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  calibrateSensors(500);

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  float initAccX = (rawAx / 4096.0) - accOffsetX;
  float initAccY = (rawAy / 4096.0) - accOffsetY;
  float initAccZ = (rawAz / 4096.0) - accOffsetZ;

  roll = atan2(initAccY, initAccZ) * (180.0 / PI);
  pitch = atan2(-initAccX, sqrt(initAccY * initAccY + initAccZ * initAccZ)) * (180.0 / PI);

  previousMicros = micros();
}

void loop() {
  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= LOOP_TIME_US) {
    dt = (currentMicros - previousMicros) / 1000000.0;
    previousMicros = currentMicros;

    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    float accX = (rawAx / 4096.0) - accOffsetX;
    float accY = (rawAy / 4096.0) - accOffsetY;
    float accZ = (rawAz / 4096.0) - accOffsetZ;

    float gyroX = (rawGx / 16.4) - gyroOffsetX;
    float gyroY = (rawGy / 16.4) - gyroOffsetY;
    float gyroZ = (rawGz / 16.4) - gyroOffsetZ;

    float accRoll = atan2(accY, accZ) * (180.0 / PI);
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * (180.0 / PI);

    roll  = 0.98 * (roll  + gyroX * dt) + 0.02 * accRoll;
    pitch = 0.98 * (pitch + gyroY * dt) + 0.02 * accPitch;

    float targetRollAngle  = 0.0f;
    float targetPitchAngle = 0.0f;

    float rollCorrection  = computePID(pidRoll,  targetRollAngle,  roll, gyroX ,  dt);
    float pitchCorrection = computePID(pidPitch, targetPitchAngle, pitch, gyroY , dt);


    Serial.print("Min:-180,Max:180,");
    Serial.print("TargetRoll:");   Serial.print(targetRollAngle);   Serial.print(",");
    Serial.print("ActualRoll:");   Serial.print(roll);              Serial.print(",");
    Serial.print("RollCorrection:"); Serial.println(rollCorrection);
  }
}