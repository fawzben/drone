#include <Arduino.h>
#include <AlfredoCRSF.h>

// ESP32 Hardware UART2 pins
#define RX_PIN 16
#define TX_PIN 17

// Use ESP32 HardwareSerial port 2
HardwareSerial crsfSerial(2);
AlfredoCRSF crsf;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n--- Testing AlfredoCRSF ---");

  // 1. Start Serial2 at standard CRSF baud rate on pins 16 & 17
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // 2. Initialize the library with our hardware serial stream
  crsf.begin(crsfSerial);
}

void loop() {
  // Feed incoming UART bytes to the parser
  crsf.update();

  // Read standard RC channels (1-indexed: 1 = Roll, 2 = Pitch, 3 = Throttle, 4 = Yaw)
  int roll     = crsf.getChannel(1);
  int pitch    = crsf.getChannel(2);
  int throttle = crsf.getChannel(3);
  int yaw      = crsf.getChannel(4);
  int aux1     = crsf.getChannel(5); // Arm switch

  // Formatted for the Arduino Serial Plotter
  Serial.print("Min:900,");
  Serial.print("Max:2100,");

  Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(",");

  Serial.print("Pitch:");
  Serial.print(pitch);
  Serial.print(",");

  Serial.print("Throttle:");
  Serial.print(throttle);
  Serial.print(",");

  Serial.print("Yaw:");
  Serial.print(yaw);
  Serial.print(",");

  Serial.print("Arm:");
  Serial.println(aux1);

  delay(20); // ~50Hz stream matches typical transmitter rate
}