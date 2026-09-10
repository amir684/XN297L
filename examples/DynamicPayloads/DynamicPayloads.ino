/*
 * DynamicPayloads -- packets exactly as long as their contents.
 *
 * With dynamic payloads each packet carries its own length, so the receiver
 * no longer has to agree on a fixed size in advance.
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
const uint16_t PIN_CE = PA4, PIN_CSN = PB0;  // SPI1: SCK PA5, MISO PA6, MOSI PA7
#else
const uint16_t PIN_CE = 9, PIN_CSN = 10;     // AVR: SCK 13, MISO 12, MOSI 11
#endif

XN297L radio(PIN_CE, PIN_CSN);

uint8_t address[][6] = {"1Node", "2Node"};

bool radioNumber = 1;
bool role = false;

// All fit the 32-byte FIFO; the last one fills it exactly.
const char *messages[] = {
  "hi",
  "XN297L",
  "dynamic payloads",
  "thirty-two bytes fill this FIFO!",
};
const uint8_t MESSAGE_COUNT = sizeof(messages) / sizeof(messages[0]);
uint8_t next = 0;

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
  radio.enableDynamicPayloads();

  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);
  radio.startListening();
}

void loop() {
  if (role) {
    const char *msg = messages[next];
    uint8_t len = strlen(msg);
    bool ok = radio.write(msg, len);

    Serial.print(ok ? F("sent ") : F("FAILED "));
    Serial.print(len);
    Serial.print(F(" bytes: "));
    Serial.println(msg);

    next = (next + 1) % MESSAGE_COUNT;
    delay(1000);

  } else if (radio.available()) {
    // A length of 0 means the chip reported something impossible; the library
    // has already flushed it.
    uint8_t len = radio.getDynamicPayloadSize();
    if (len) {
      char buf[33];
      radio.read(buf, len);
      buf[len] = '\0';
      Serial.print(F("received "));
      Serial.print(len);
      Serial.print(F(" bytes: "));
      Serial.println(buf);
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
      radio.startListening();
    }
  }
}
