#include <Arduino.h>

// UART2: listens to MAVLink from laptop
static const int UART2_RX_PIN  = 16;
static const int UART2_TX_PIN  = 17;

// UART0: forwards MAVLink to flight controller
static const int UART0_RX_PIN  = 3;
static const int UART0_TX_PIN  = 1;

// Trigger output pin
static const int TRIG_PIN      = 4;

// Serial baud rate
static const uint32_t BAUDRATE = 115200;

enum FrameState {
  WAIT_STX,    // waiting for start byte (0xFD)
  READ_LEN,    // just saw 0xFD, next byte = payload_length
  READ_REST    // forwarding rest of packet until CRC done
};

FrameState state = WAIT_STX;
uint16_t bytesRemaining = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // start UART2 (input from laptop)
  Serial2.begin(BAUDRATE, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  // start UART0 (output to flight controller)
  Serial.begin(BAUDRATE, SERIAL_8N1, UART0_RX_PIN, UART0_TX_PIN);
}

void loop() {

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    // forward every byte immediately to drone controller
    Serial.write(b);

    switch (state) {
      case WAIT_STX:
        if (b == 0xFD) {
          state = READ_LEN;
          digitalWrite(TRIG_PIN, HIGH);
        }
        break;
      case READ_LEN:
        {
          uint8_t payload_len = b;
          // compute how many more bytes remain in this frame
          bytesRemaining = 10 + payload_len;
          state = READ_REST;
        }
        break;
      case READ_REST:
        if (bytesRemaining > 0) {
          bytesRemaining--;
          if (bytesRemaining == 0) {
            // end of packet
            digitalWrite(TRIG_PIN, LOW);
            state = WAIT_STX;
          }
        } else {
          // fallback
          digitalWrite(TRIG_PIN, LOW);
          state = WAIT_STX;
        }
        break;
    }
  }

  while (Serial.available()) {
    uint8_t b = Serial.read();
    Serial2.write(b);
  }
}

