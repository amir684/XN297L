/*
 * GettingStarted -- two radios, one sketch.
 *
 * The same program as the RF24 library's GettingStarted example, so code
 * written for an nRF24L01 carries over almost line for line.
 *
 * Flash it on both boards, then in each serial monitor:
 *   - type 0 on one board and 1 on the other   (which address each uses)
 *   - type T on one of them                    (make it the transmitter)
 *
 * The transmitter sends a float every second and reports how long the write
 * took and how many retransmits it needed. R switches a board back to
 * receiving.
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

// Addresses are 5 bytes; each string literal's terminating NUL is not sent.
uint8_t address[][6] = {"1Node", "2Node"};

bool radioNumber = 1;   // 0 or 1, chosen over serial
bool role = false;      // true = transmitter
float payload = 0.0;

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

  // Two boards on one desk at full power saturate each other's receivers.
  radio.setPALevel(XN297L_PA_LOW);
  radio.setPayloadSize(sizeof(payload));

  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);

  if (role) radio.stopListening();
  else      radio.startListening();
}

void loop() {
  if (role) {
    unsigned long start = micros();
    bool ok = radio.write(&payload, sizeof(payload));
    unsigned long took = micros() - start;

    if (ok) {
      Serial.print(F("sent "));
      Serial.print(payload);
      Serial.print(F("  in "));
      Serial.print(took);
      Serial.print(F(" us, retries "));
      Serial.println(radio.getARC());
      payload += 0.01;
    } else {
      Serial.println(F("send failed -- no ACK"));
    }
    delay(1000);

  } else {
    uint8_t pipe;
    if (radio.available(&pipe)) {
      uint8_t bytes = radio.getPayloadSize();
      radio.read(&payload, bytes);
      Serial.print(F("received "));
      Serial.print(bytes);
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);
      Serial.print(F(": "));
      Serial.println(payload);
    }
  }

  if (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == 'T' && !role) {
      role = true;
      Serial.println(F("*** TRANSMIT role -- enter R to switch back"));
      radio.stopListening();
    } else if (c == 'R' && role) {
      role = false;
      Serial.println(F("*** RECEIVE role -- enter T to switch"));
      radio.startListening();
    }
  }
}
