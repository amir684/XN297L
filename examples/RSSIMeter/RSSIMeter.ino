/*
 * RSSIMeter -- signal strength from a chip whose datasheet never says how.
 *
 * The XN297L datasheet lists an RSSI register but not how to enable it, and
 * out of the box it reads zero. This library turns it on -- the story of how
 * that was found is in the README.
 *
 * Two things to know before trusting the number:
 *   - It is a relative 4-bit reading, not dBm. On the modules tested it runs
 *     from about 1 (another room) to 8 (boards touching), with a couple of
 *     counts of noise, so read the mean rather than single samples.
 *   - Enabling it costs about 6 dB of receive sensitivity. This sketch arms it
 *     for one packet in SAMPLE_EVERY and leaves the receiver at full
 *     sensitivity the rest of the time.
 *
 * The other board runs the GettingStarted example as radio 0, transmitting.
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

uint8_t address[][6] = {"1Node", "2Node"};   // must match GettingStarted
const uint8_t SAMPLE_EVERY = 4;

float    payload;
uint32_t packets = 0;
bool     armed   = true;

// Rolling mean of the last 8 samples.
uint8_t  history[8];
uint8_t  historyCount = 0;
uint8_t  historyPos   = 0;

uint8_t addSample(uint8_t v) {
  history[historyPos] = v;
  historyPos = (historyPos + 1) % 8;
  if (historyCount < 8) historyCount++;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < historyCount; i++) sum += history[i];
  return (sum + historyCount / 2) / historyCount;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  SPI.begin();
  if (!radio.begin()) {
    Serial.println(F("radio is not responding -- run the WiringCheck example"));
    while (true) delay(1000);
  }

  radio.setPALevel(XN297L_PA_LOW);
  radio.setPayloadSize(sizeof(payload));
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);

  radio.enableRSSI();          // armed for the first packet
  radio.startListening();

  Serial.println(F("waiting for GettingStarted (radio 0, transmitting)..."));
}

void loop() {
  if (!radio.available()) return;

  uint8_t rssi = radio.getRSSI();    // latched when available() saw the packet
  radio.read(&payload, sizeof(payload));
  packets++;

  if (armed) {
    uint8_t mean = addSample(rssi);
    Serial.print(F("rssi "));
    Serial.print(rssi);
    Serial.print(F("   mean "));
    Serial.print(mean);
    Serial.print(F("   "));
    for (uint8_t i = 0; i < mean; i++) Serial.print('#');
    Serial.println();
  }

  // Arm for the next packet only on sampling rounds. Both calls are no more
  // than a register write, and enableRSSI() takes care of stepping out of RX
  // for it and back.
  bool nextArmed = (packets % SAMPLE_EVERY) == 0;
  if (nextArmed != armed) {
    if (nextArmed) radio.enableRSSI();
    else           radio.disableRSSI();
    armed = nextArmed;
  }
}
