#include <Arduino.h>

// PID Controller Data Structure
struct PIDController {
  // Tuning Gains
  float Kp;
  float Ki;
  float Kd;

 // Saturation Limits
  float maxIntegral;
  float maxOutput;

  // State Variables
  float integral;
  float prevMeasured;
  
 
};

// Create PID controller instances for Roll, Pitch, and Yaw
PIDController pidRoll  = {1.3f, 0.04f, 18.0f, 100.0f, 400.0f}; // Kp, Ki, Kd, iMax, outMax
PIDController pidPitch = {1.3f, 0.04f, 18.0f, 100.0f, 400.0f};
PIDController pidYaw   = {2.5f, 0.02f,  0.0f,  50.0f, 400.0f};

// PID Calculation Function
float computePID(PIDController &pid, float target, float measured, float dt) {
  // 1. Calculate Error
  float error = target - measured;

  // 2. Proportional Term (P-term)
  float pTerm = pid.Kp * error;

  // 3. Integral Term (I-term) with Anti-Windup Clamping (so it doesn't get too big)
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -pid.maxIntegral, pid.maxIntegral); //
  float iTerm = pid.Ki * pid.integral;

  // 4. Derivative Term (D-term) on Measurement rather than error (Prevents Derivative Kick if I suddenly push the stick)
  float derivative = (measured - pid.prevMeasured) / dt;
  pid.prevMeasured = measured;
  float dTerm = -pid.Kd * derivative; // negative bc acts as a "braking" force, it counteracts the pTerm

  // 5. Total Output with Saturation Clamping
  float output = pTerm + iTerm + dTerm;
  return constrain(output, -pid.maxOutput, pid.maxOutput);
}

// Reset PID integral and state (used on disarm or ground idle)
void resetPID(PIDController &pid) {
  pid.integral = 0.0f;
  pid.prevMeasured = 0.0f;
}

// Timing variables (simulating a 250Hz loop)
unsigned long previousMicros = 0;
const unsigned long LOOP_TIME_US = 4000; // 4000us = 4ms (250Hz) [cite: 1301, 1618]
float dt = 0.004f; // 4ms in seconds [cite: 1621]

void setup() {
  Serial.begin(115200); 
  previousMicros = micros(); 
}

void loop() {
  unsigned long currentMicros = micros(); 

  if (currentMicros - previousMicros >= LOOP_TIME_US) {
    dt = (currentMicros - previousMicros) / 1000000.0f;
    previousMicros = currentMicros;

    // ==========================================
    // MANUAL TEST INPUTS
    // ==========================================
    // Desired setpoints (e.g., target 0 degrees to stay level)
    float targetRollAngle  = 0.0f; 
    float targetPitchAngle = 0.0f;
    float targetYawRate    = 0.0f;

    // Simulated actual orientation (modify these to test responses)
    // E.g., simulate the drone currently tilted 15 degrees right on Roll
    float actualRollAngle  = 15.0f; 
    float actualPitchAngle = 0.0f;
    float actualYawRate    = 0.0f;

    // ==========================================
    // COMPUTE PID OUTPUTS
    // ==========================================
    float rollCorrection  = computePID(pidRoll,  targetRollAngle,  actualRollAngle,  dt);
    float pitchCorrection = computePID(pidPitch, targetPitchAngle, actualPitchAngle, dt);
    float yawCorrection   = computePID(pidYaw,   targetYawRate,    actualYawRate,    dt);

    // ==========================================
    // TELEMETRY / SERIAL PLOTTER FORMAT
    // ==========================================
    Serial.print("TargetRoll:");   Serial.print(targetRollAngle);   Serial.print(",");
    Serial.print("ActualRoll:");   Serial.print(actualRollAngle);   Serial.print(",");
    Serial.print("RollCorrection:"); Serial.println(rollCorrection);
  }
}
