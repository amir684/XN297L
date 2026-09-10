/*
 * AckPayloads -- data back inside the acknowledgement.
 *
 * The receiver pre-loads a reply, and the chip sends it back inside the ACK
 * for the next packet it receives: no second transmission, no switching roles,
 * and the transmitter gets its answer in the same write() call.
 *
 * Flash it on both boards, then in each serial monitor:
 *   - type 0 on one board and 1 on the other
 *   - type T on one of them to make it transmit
 */
#include <SPI.h>
#include <XN297L.h>

// CE and CSN are free choices; SCK, MOSI and MISO are the board's SPI pins.
#if defined(CONFIG_IDF_TARGET_ESP32C3)
const uint16_t PIN_CE = 1, PIN_CSN = 7;      // SPI: SCK 4, MISO 5, MOSI 6
#elif defined(ESP32)
const uint16_t PIN_CE = 4, PIN_CSN = 5;      // SPI: SCK 18, MISO 19, MOSI 23
#elif defined(ARDUINO_ARCH_RP2040)
const uint16_t PIN_CE = 20, PIN_CSN = 17;    // SPI0: SCK 18, MISO 16, MOSI 19
#elif defined(ARDUINO_ARCH_STM32)
const uint16_t PIN_CE = PB0, PIN_CSN = PA4;  // SPI1: SCK PA5, MISO PA6, MOSI PA7
#else
const uint16_t PIN_CE = 9, PIN_CSN = 10;     // AVR: SCK 13, MISO 12, MOSI 11
#endif

XN297L radio(PIN_CE, PIN_CSN);

uint8_t address[][6] = {"1Node", "2Node"};

bool radioNumber = 1;
bool role = false;

uint32_t sent  = 0;    // transmitter's counter
uint32_t reply = 0;    // receiver's counter, returned in the ACK

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  SPI.begin();
  if (!radio.begin()) {
    Serial.println(F("radio is not responding -- run the WiringCheck example"));
    while (true) delay(1000);
  }

  Serial.println(F("Which radio is this? Enter 0 or 1"));
  while (!Serial.available()) delay(10);
  radioNumber = Serial.read() == '1';
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);
  Serial.println(F("*** Enter T to transmit, R to receive"));

  radio.setPALevel(XN297L_PA_LOW);
  radio.enableAckPayload();          // also turns on dynamic payloads

  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);

  // The first reply has to be waiting before the first packet arrives.
  radio.writeAckPayload(1, &reply, sizeof(reply));
  radio.startListening();
}

void loop() {
  if (role) {
    bool ok = radio.write(&sent, sizeof(sent));
    Serial.print(F("sent "));
    Serial.print(sent);

    if (!ok) {
      Serial.println(F("  -- FAILED, no ACK"));
    } else if (radio.available()) {
      uint32_t back = 0;
      radio.read(&back, sizeof(back));
      Serial.print(F("  -- ACK carried "));
      Serial.println(back);
    } else {
      Serial.println(F("  -- ACK was empty"));
    }

    sent++;
    delay(1000);

  } else {
    uint8_t pipe;
    if (radio.available(&pipe)) {
      uint32_t got = 0;
      radio.read(&got, sizeof(got));
      Serial.print(F("received "));
      Serial.print(got);
      Serial.print(F(", replied "));
      Serial.println(reply);

      // Queue the reply for the NEXT packet -- this one's ACK already left.
      reply++;
      radio.writeAckPayload(pipe, &reply, sizeof(reply));
    }
  }

  if (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == 'T' && !role) {
      role = true;
      Serial.println(F("*** TRANSMIT role"));
      radio.stopListening();
    } else if (c == 'R' && role) {
      role = false;
      Serial.println(F("*** RECEIVE role"));
      radio.flush_tx();
      radio.writeAckPayload(1, &reply, sizeof(reply));
      radio.startListening();
    }
  }
}
