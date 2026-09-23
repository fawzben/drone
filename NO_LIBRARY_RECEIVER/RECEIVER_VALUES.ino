#include <Arduino.h>
//basically not using CRSF library cus it makes it crash, homemade sauce

// Pins to connect to receiver:
// ESP32 GPIO 16 (RX2) -> Receiver TX
// ESP32 GPIO 17 (TX2) -> Receiver RX (optional, for telemetry)
#define RX_PIN 16
#define TX_PIN 17

// Standard CRSF frame constants
#define CRSF_BAUDRATE 420000 // bc receiver streams at this speed (bits/s)
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_TYPE_RC_CHANNELS 0x16

// 16 RC channels (values typically 1000 - 2000 us, centered at 1500 us)
uint16_t channels[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Parser state machine buffer
uint8_t crsfBuffer[64];
uint8_t bufferIndex = 0;
uint8_t payloadLength = 0;

void processCrsfChannels(const uint8_t *payload) {
  // CRSF packs 16 channels of 11-bit values into 22 bytes
  channels[0]  = ((payload[0]       | payload[1]  << 8)                     & 0x07FF);
  channels[1]  = ((payload[1]  >> 3 | payload[2]  << 5)                     & 0x07FF);
  channels[2]  = ((payload[2]  >> 6 | payload[3]  << 2 | payload[4] << 10)  & 0x07FF);
  channels[3]  = ((payload[4]  >> 1 | payload[5]  << 7)                     & 0x07FF);
  channels[4]  = ((payload[5]  >> 4 | payload[6]  << 4)                     & 0x07FF);

  // Convert raw 11-bit CRSF (172 - 1811) to standard RC microseconds (1000 - 2000 us)
  for (int i = 0; i < 5; i++) {
    channels[i] = map(channels[i], 172, 1811, 988, 2012);
  }
}

//to collect the packets
void readCrsf() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (bufferIndex == 0) {
      if (b == CRSF_SYNC_BYTE) {
        crsfBuffer[bufferIndex++] = b;
      }
    } else if (bufferIndex == 1) {
      payloadLength = b;
      if (payloadLength > 60 || payloadLength < 2) {
        bufferIndex = 0; // Malformed frame length, reset
      } else {
        crsfBuffer[bufferIndex++] = b;
      }
    } else {
      crsfBuffer[bufferIndex++] = b;
      // Frame complete: sync (1 byte) + len byte (1 byte) + payloadLength
      if (bufferIndex == (payloadLength + 2)) {
        if (crsfBuffer[2] == CRSF_TYPE_RC_CHANNELS) {
          processCrsfChannels(&crsfBuffer[3]);
        }
        bufferIndex = 0; // Reset for next incoming frame
      }
    }
  }
}

void setup() {
  // Serial to PC for Plotter
  Serial.begin(115200);
  delay(1000);

  // Native ESP32 Hardware UART2 at CRSF rate
  Serial2.begin(CRSF_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  readCrsf();

  // Serial Plotter format: Min, Max anchors and labeled RC channels
  Serial.print("Min:900,");
  Serial.print("Max:2100,");

  Serial.print("Roll:");
  Serial.print(channels[0]);
  Serial.print(",");

  Serial.print("Pitch:");
  Serial.print(channels[1]);
  Serial.print(",");

  Serial.print("Throttle:");
  Serial.print(channels[2]);
  Serial.print(",");

  Serial.print("Yaw:");
  Serial.println(channels[3]);

  delay(20); // ~50Hz plot update
}