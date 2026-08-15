#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// library to talk over I2C (inter-integrated circuits) - SDA (data) and SCA (clock) to talk to the IMU

Adafruit_MPU6050 mpu; // defines the IMU

void setup() {
Serial.begin(115200);
Wire.begin(21, 22);
if (!mpu.begin()) {
  Serial.println("ERROR - IMU not found"); //Wakes up the IMU, checks its identity register over I2C, and initializes its internal clock. 
                                          //If the sensor is unplugged or wired incorrectly, it halts execution (while(1)) so bad data isn't processed.
  while (1) {
    delay(10);
  }
}

Serial.println("MPU6050 Initialized Successfully!");

// Configure sensor ranges for drone dynamics
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // max. +- 8G of acceleration
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //max. +- 500 degrees/s of angular speed
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // sets up the included low-pass filter - maybe try without it to see

delay (100);

}
void loop() {
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp); // fetches the data from the IMU

// Print Acceleration (m/s^2)
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(" | Y: "); Serial.print(a.acceleration.y);
  Serial.print(" | Z: "); Serial.print(a.acceleration.z);

  // Print Gyroscope (rad/s)
  Serial.print("  ||  Gyro X: "); Serial.print(g.gyro.x);
  Serial.print(" | Y: "); Serial.print(g.gyro.y);
  Serial.print(" | Z: "); Serial.println(g.gyro.z);

delay (50); // 20Hz for readability

}
