/*
 * WiringCheck -- is the module alive?
 *
 * The first sketch to run on a new XN297L. It proves the SPI wiring in both
 * directions and prints every register, without needing a second radio.
 *
 * If begin() reports the chip is not responding, check in this order:
 *   1. 3V3 and GND actually reach the module. The XN297L is a 2.2-3.3 V part;
 *      5 V on VDD destroys it.
 *   2. MOSI and MISO are not swapped, and CLK / CSN are on the pins below.
 *   3. On a 5 V board (Uno, Nano, Mega) the logic lines need a level shifter,
 *      and the module needs its own 3.3 V regulator -- the Uno's 3V3 pin
 *      cannot supply the transmit current.
 *   4. The 16 MHz crystal on the module is intact.
 *
 * A register dump of all 00 or all FF means there is no SPI at all.
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

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}   // give native-USB boards a moment

  SPI.begin();

  if (radio.begin()) {
    Serial.println(F("XN297L is alive -- SPI works in both directions."));
  } else {
    Serial.println(F("XN297L is NOT responding. See the checklist at the top of this sketch."));
  }

  radio.printDetails();
}

void loop() {}
