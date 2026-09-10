# XN297L

[![CI](https://github.com/amir684/XN297L/actions/workflows/ci.yml/badge.svg)](https://github.com/amir684/XN297L/actions/workflows/ci.yml)

An Arduino library for the **Panchip XN297L** 2.4 GHz transceiver — with an
RF24-compatible API, and the RSSI readout the datasheet never explains.

The XN297L is the radio inside a great many cheap toy drones, RC cars and
remotes. Its register map looks like an nRF24L01's, which is exactly why nRF24
drivers fail on it: SPI works, every register reads back correctly, and nothing
ever goes on air. This library is written from the Panchip datasheet, handles the
register-level differences that cause that, and was developed against modules
desoldered from a 2015 toy drone.

- **RF24-compatible API.** `openWritingPipe`, `write`, `available`, `read` —
  nRF24L01 code ports across almost line for line.
- **RSSI.** The register is documented; the way to turn it on is not. Found by bit
  sweep, cross-checked against Panchip's own init code, and verified against
  distance.
- **Portable.** ESP32, ESP32-C3, AVR, RP2040 and STM32, compile-tested in CI on
  every push -- and small enough for a 32 KB STM32F030.

Everything here was measured, and the dead ends are documented alongside the
results — knowing what was already ruled out is most of the value when a chip has
no real documentation.

---

## Contents

- [Installation](#installation)
- [Quick start](#quick-start)
- [Wiring](#wiring)
- [Examples](#examples)
- [API](#api)
- [The XN297L is not an nRF24L01](#the-xn297l-is-not-an-nrf24l01)
- [RSSI: the undocumented feature](#rssi-the-undocumented-feature)
- [Link quality without RSSI](#link-quality-without-rssi)
- [Verification status](#verification-status)
- [Demo project](#demo-project)
- [Roadmap](#roadmap)
- [Sources](#sources)

---

## Installation

**Arduino IDE** — *Sketch → Include Library → Manage Libraries*, search for
**XN297L**.

**PlatformIO**

```ini
lib_deps = amir684/XN297L
```

**Manually** — download this repository as a ZIP and use *Sketch → Include Library
→ Add .ZIP Library*.

---

## Quick start

```cpp
#include <XN297L.h>

XN297L radio(4, 5);                  // CE, CSN
const uint8_t address[5] = {'N', 'o', 'd', 'e', '1'};

void setup() {
  Serial.begin(115200);
  if (!radio.begin()) Serial.println("XN297L not responding");

  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop() {
  const char msg[] = "hello";
  bool acked = radio.write(msg, sizeof(msg));
  Serial.println(acked ? "sent" : "no ACK");
  delay(1000);
}
```

The receiving side:

```cpp
radio.openReadingPipe(1, address);
radio.startListening();

if (radio.available()) {
  char msg[32];
  radio.read(msg, sizeof(msg));
}
```

Both sides have to agree on the payload size — 32 bytes unless you call
`setPayloadSize()`, or use `enableDynamicPayloads()` and stop worrying about it.

---

## Wiring

| XN297L | ESP32 | ESP32-C3 | Pico | STM32 ³ | Uno / Nano ¹ |
|---|---|---|---|---|---|
| `3V3` | 3V3 | 3V3 | 3V3 | 3V3 | **separate 3.3 V regulator** |
| `GND` | GND | GND | GND | GND | GND |
| `CLK` | 18 | 4 | GP18 | PA5 | 13 |
| `MISO` | 19 | 5 ² | GP16 | PA6 | 12 |
| `MOSI` | 23 | 6 ² | GP19 | PA7 | 11 |
| `CSN` | 5 | 7 | GP17 | PB0 | 10 |
| `CE` | 4 | 1 | GP20 | PA4 | 9 |
| `IRQ` | optional | optional | optional | optional | optional |

CE and CSN are free choices; the rest are each board's default SPI pins.

¹ **5 V boards need two extra parts.** The XN297L's absolute maximum input is
3.6 V, so the logic lines need a **level shifter**. And the Uno's 3V3 pin cannot
supply the transmit current (up to 66 mA at 11 dBm), so the module needs **its own
3.3 V regulator**.

² ESP32-C3 boards with a built-in 0.42" OLED use GPIO 5 and 6 for the display. Pick
other pins and pass them to `SPI.begin(sck, miso, mosi)` before `radio.begin()` —
see the demo project.

³ STM32F030K6 or F103 Blue Pill, on SPI1. CE on PA4 and CSN on PB0 are the IMU link
board's wiring, which has run on hardware.

**Supply.** The XN297L runs on 2.2–3.3 V. Put **10 µF across the module's 3V3 and
GND**, close to the pads. Supply sag during transmit bursts is the single most
common cause of a link that works intermittently and looks like a range problem.

The `IRQ` pin is active-low and optional. The library polls; connect it if you want
to use interrupts (`maskIRQ()` and `whatHappened()` are there for that).

---

## Examples

| Example | What it shows |
|---|---|
| **WiringCheck** | Is the module alive? Proves SPI in both directions and dumps every register. Run this first. |
| **GettingStarted** | Two radios, one sketch — the RF24 example of the same name, ported. |
| **RSSIMeter** | Signal strength, duty-cycled to avoid the sensitivity cost. |
| **DynamicPayloads** | Packets exactly as long as their contents. |
| **AckPayloads** | Data returned inside the acknowledgement. |

**PlatformIO users** can build any example without copying it anywhere:
[`extras/ExampleRunner`](extras/ExampleRunner/platformio.ini) points PlatformIO's
source directory at an example and builds it against this repository. It has
environments for ESP32, ESP32-C3, Uno and the 32 KB STM32F030K6 -- every example
fits on that one, at 17.7 to 25.1 KB.

---

## API

### RF24-compatible

Names and behaviour follow [RF24](https://github.com/nRF24/RF24), including its
pipe 0 handling: `openWritingPipe()` borrows pipe 0 to receive ACKs,
`startListening()` restores the reading address or closes pipe 0, and
`stopListening()` points it back at the writing address.

| | |
|---|---|
| `begin()` | starts SPI, resets and configures; `true` if the chip answers |
| `isChipConnected()` | SPI round trip in both directions |
| `powerUp()` / `powerDown()` | |
| `startListening()` / `stopListening()` | |
| `available()` / `available(&pipe)` | |
| `read(buf, len)` | |
| `write(buf, len)` | blocks until ACKed or retries run out |
| `write(buf, len, true)` | no ACK requested — works straight after `begin()` |
| `openWritingPipe(addr)` | |
| `openReadingPipe(pipe, addr)` / `closeReadingPipe(pipe)` | pipes 2–5 take only the low byte |
| `setAddressWidth(3..5)` | |
| `setRetries(delay, count)` | delay step 250 µs |
| `setChannel()` / `getChannel()` | 2400 + n MHz |
| `setPayloadSize()` / `getPayloadSize()` | 1–32 |
| `enableDynamicPayloads()` / `disableDynamicPayloads()` / `getDynamicPayloadSize()` | |
| `enableAckPayload()` / `writeAckPayload()` | |
| `enableDynamicAck()` | |
| `setAutoAck(bool)` / `setAutoAck(pipe, bool)` | |
| `setPALevel()` / `getPALevel()` | |
| `setDataRate()` / `getDataRate()` | |
| `setCRCLength()` / `getCRCLength()` / `disableCRC()` | |
| `getARC()` | retransmits on the last write |
| `rxFifoFull()`, `flush_rx()`, `flush_tx()` | |
| `maskIRQ()`, `whatHappened()` | |
| `printDetails(Stream&)` | |

**Porting from RF24:** rename the constants — `RF24_PA_LOW` → `XN297L_PA_LOW`,
`RF24_1MBPS` → `XN297L_1MBPS`, `RF24_CRC_16` → `XN297L_CRC_16`.

### XN297L only

| | |
|---|---|
| `enableRSSI(attenuation)` / `disableRSSI()` | the undocumented detector — costs sensitivity, see below |
| `getRSSI()` | 0–15, latched by `available()` |
| `readRegister()` / `writeRegister()` | single byte or multi-byte |
| `getStatus()` | |

**Build option:** `-DXN297L_NO_DETAILS` compiles out `printDetails()` and its
register names -- 1.2 KB back on an STM32F030, where that is 4% of the flash.

### Output power

| constant | dBm | TX current |
|---|---|---|
| `XN297L_PA_MINUS23DBM` / `XN297L_PA_MIN` | −23 | ~9 mA |
| `XN297L_PA_MINUS10DBM` | −10 | |
| `XN297L_PA_MINUS1DBM` / `XN297L_PA_LOW` | −1 | ~16 mA |
| `XN297L_PA_4DBM` | 4 | |
| `XN297L_PA_5DBM` | 5 | ~20 mA |
| `XN297L_PA_9DBM` / `XN297L_PA_HIGH` | 9 | ~30 mA — **default** |
| `XN297L_PA_11DBM` / `XN297L_PA_MAX` | 11 | ~66 mA |

The default is 9 dBm rather than the maximum: 2 dB less for under half the current,
which keeps a weak supply out of trouble. The receiver itself saturates above
0 dBm, so two boards on one desk at full power can link *worse* than at `LOW`.

Data rates are `XN297L_1MBPS` (default), `XN297L_2MBPS` and `XN297L_250KBPS`. The
datasheet asks for a ±20 ppm crystal at 250 kbps.

---

## The XN297L is not an nRF24L01

The register map looks compatible, which is exactly the trap. Four differences
matter, and each is handled and explained where it lives in
[`src/XN297L.cpp`](src/XN297L.cpp).

### 1. `CONFIG` bit 7 is `EN_PM`, and it must be set

On the nRF24L01 this bit is reserved-zero. On the XN297L, Table 4-1 requires
`EN_PM = 1` for **both** RX and TX — it is what moves the chip from standby-I to
standby-III, and TX and RX are only reachable from there.

A ported nRF24 driver leaves it clear, and the symptom is brutal to debug: SPI works
perfectly, the self-test passes, the register dump looks right, and not one packet
ever moves.

### 2. `RF_SETUP` is a 2-bit rate and a 6-bit power code

| field | bits | values |
|---|---|---|
| `RF_DR` | 7:6 | `00` = 1 Mbps, `01` = 2 Mbps, `11` = 250 kbps, `10` reserved |
| `RF_PWR` | 5:0 | a code from a fixed table — see [Output power](#output-power) |

An nRF24 "max power" value lands on a combination that does not exist.

### 3. Reset is a command with a data byte

`0x53 0x5A` holds the chip in reset, `0x53 0xA5` releases it. A bare `0x53` leaves
it latched in reset.

### 4. SPI is capped at 1 Mbps in power-down and standby-I

Which is where initialisation happens. `begin()` runs at 1 MHz and only steps up
after reaching standby-III; `powerDown()` drops back.

### Also

- `FEATURE` carries `MUX_PA_IRQ`, `CE_SEL` and `DATA_LEN_SEL`, which have no nRF24
  equivalent.
- The datasheet's `FIFO_STATUS` table has the names and descriptions of bits 0 and
  1 **crossed**. The library uses `STATUS.RX_P_NO` (`111` = empty) to detect data,
  and the reset values to pin down which bit is which.
- Channels on multiples of the 16 MHz crystal — 0, 16, 32, 48, 64, 80 — lose about
  2 dB of sensitivity. The default is 78.
- **API-compatible is not air-compatible.** The XN297 family frames packets
  differently from the nRF24L01; an XN297L does not talk to an nRF24L01 radio
  directly.

---

## RSSI: the undocumented feature

**Short version:** the XN297L can report RSSI. It is off by default, the enable is in
no XN297L datasheet, and turning it on costs about 6 dB of receive sensitivity — so
enable it for the packets you want to measure, and disable it again.

```cpp
radio.enableRSSI();
// ... available() latches the reading when a packet arrives
uint8_t strength = radio.getRSSI();   // 0-15, relative
radio.disableRSSI();
```

### The problem

Register `0x09` is documented — `RSSI_RT` in bits 7:4, `RSSI_SY` in bits 3:0 — but
marked as a *"special function register ... declared in the software design
reference"*, a document Panchip never published. It reads `0x00` no matter when you
sample it.

### Finding the reference

The full **XN297** (non-L) Chinese datasheet does document it, and a copy survives
as `XN297_complete.pdf` in
[foldedtoad/xn297_cal](https://github.com/foldedtoad/xn297_cal):

| location | field | meaning |
|---|---|---|
| `RF_SETUP` bit 7 | `RSSI_EN` | `1` = enabled, **resets to 0** |
| `RF_SETUP` bit 5 | `RSSI_SEL` | sample through the filter or not |
| `CONFIG` bit 7 | `DATAOUT_SEL` | selects what `0x09` reports |
| `RF_CAL` bits 48:47 | `RSSI_GAIN_CTR` | `00` = 0 dB, `01` = −6, `10` = −12, `11` = −18 |

Panchip marks all of it **(测试用)** — *"for test use"*.

### Why that did not work

Both enable bits were **reused on the L variant**: `RF_SETUP` bit 7 became the top
of `RF_DR`, and `CONFIG` bit 7 became `EN_PM`.

Confirmed by experiment. With `RF_SETUP` bit 7 set, data still crossed but the
auto-ACK stopped and `0x09` stayed at zero — that is `RF_DR` moving to the reserved
`10` code, not `RSSI_EN`. Both boards changed together so packets still got
through, but the ACK window scales with the data rate, so it broke.

### Finding it by sweep

A sweep flipped **one bit at a time** in the undocumented registers `0x19`, `0x1B`
and `0x1E` — 56 candidates — and watched `0x09`. Single bits rather than brute
force: seven bytes is 2⁵⁶ combinations, and staying one bit from the factory state
is both the more informative search and the safer one.

**The first sweep produced three false positives.** Its test for "reception still
works" was only that something reached the RX FIFO — which noise satisfies. The bits
it flagged break packet validation, after which the chip decodes noise continuously
and `0x09` fills with garbage:

```
[ 34050] rx #572657937    lost=12963   rssi=88
[ 34051] rx #1145315874   lost=17331   rssi=22
```

Putting a magic word in the payload and counting **valid packets separately from
junk** fixed the test. Two candidates survived, with zero junk:

| `RF_CAL` (0x1E) bit | `RSSI_GAIN_CTR` | `0x09` read |
|---|---|---|
| 15 | `01` = −6 dB | `0x66` |
| 16 | `10` = −12 dB | `0x06` |

They are adjacent, and they land exactly where `RSSI_GAIN_CTR` sits in the XN297
field map — a position **predicted before the sweep ran** — if the L variant's
24-bit `RF_CAL` is the top 24 bits of the XN297's 56-bit one. The readings behave
like an attenuator: `RSSI_RT` drops from 6 to 0 as the field goes from `01` to `10`.

**Independent confirmation** turned up later in Panchip's own reference
initialisation, preserved in a PY32 example driver: it writes
`RF_CAL = F6 3F 5D`, and decoding that sets bit 16 — the same field.

There appears to be **no separate enable** on the XN297L. `RSSI_GAIN_CTR = 00`, the
reset value, simply means no measurement.

### Verified

Readings track distance — about **8 with the boards touching, 1 from another
room** — with no packet loss.

`getRSSI()` returns the low nibble, `RSSI_SY`. The high nibble `RSSI_RT` reads 0 in
practice: it is a real-time measurement, and the burst is over before software can
read the register. `RSSI_SY` is latched at packet sync and holds. In practice the
readings span **0–8**, so scale bar graphs to 8, not 15. There are a couple of counts
of noise between packets — average it.

### The cost, and working around it

The field is *"RSSI 的信号增益衰减的选择位"* — *signal gain attenuation* select
bits — and the attenuation is **not confined to the detector**. With it enabled on
both ends of a link, retries climbed from 0 to 2–3 and unacknowledged packets stopped
arriving entirely:

```
[   182] ok  retries=2   (no answer)      answers=0/183
```

Acknowledged packets survived because auto-ack gives them 15 retransmits to absorb
the loss. Unacknowledged ones had no such margin.

But the attenuation only exists **while the bits are set**, and setting them is one
register write. So measure a fraction of the packets and run at full sensitivity the
rest of the time — `enableRSSI()` and `disableRSSI()` are safe to call while
listening. The **RSSIMeter** example samples one packet in four.

Treat the number as a relative, test-mode reading: good for comparing antennas or
placements, not for absolute dBm.

---

## Link quality without RSSI

Two metrics that are documented, free, and cost no sensitivity:

- **`getARC()`** — retransmits the last `write()` needed. `0` is a clean link.
- **Counter gaps** — number your packets and count the ones that never arrive.

These are the ones to build on for anything long-running.

---

## Verification status

| | |
|---|---|
| Reset, power states, `begin()`, `isChipConnected()` | ✅ hardware — ESP32, ESP32-C3, STM32F030K6 |
| Auto-ack `write()`, retries, `getARC()` | ✅ hardware |
| NOACK `write(buf, len, true)` | ✅ hardware |
| 32-byte static payloads | ✅ hardware |
| RSSI | ✅ hardware |
| Channel, power levels, 1 Mbps | ✅ hardware |
| Multiple pipes, dynamic payloads, ACK payloads | implemented per datasheet — examples provided, not yet run |
| 2 Mbps, 250 kbps | implemented per datasheet, not yet run |
| Auto-ack stream on a 32 KB STM32F030K6 | ✅ hardware — the IMU link project |
| AVR, RP2040, STM32F103 | compile-tested in CI, not yet run |

Run one of the unverified examples on your hardware? Open an issue with the result
either way.

---

## Demo project

[`extras/XN297L_Demo`](extras/XN297L_Demo) is one PlatformIO project holding every
board this library has run on, built against the library in this repository. Open
the folder and each firmware is an env in the PlatformIO sidebar:

| env | board | role |
|---|---|---|
| `tx` / `rx` | ESP32 WROOM | the original pair |
| `tx_c3` | ESP32-C3 SuperMini | dashboard sender: battery, temperature, LED |
| `rx_c3` | ESP32-C3 + 0.42" OLED | dashboard receiver, six pages |
| `imu_tx_stm32` | STM32F030K6T6 + MPU6881 | IMU sender, flashed over ST-Link |
| `imu_rx_c3` | ESP32-C3 + 0.42" OLED | IMU receiver, eight pages |
| `scan` | ESP32 WROOM | the register sweep that found the RSSI enable |

```
cd extras/XN297L_Demo
pio run -e tx_c3 -t upload
pio run -e rx_c3 -t upload
```

### ESP32-C3 dashboard

An ESP32-C3 SuperMini transmitter reporting its battery and temperature to an
ESP32-C3 receiver with a built-in 0.42" OLED -- the project this library grew out
of.

The BOOT button cycles six pages — overview, temperature, battery, signal, stats, and
an auto-scaled temperature graph. The project also carries the `scan` environment
that found the RSSI enable, in case another chip revision needs it found again.

Some things learned along the way that apply beyond this project:

**The 0.42" 72×40 OLED** is cut differently from U8g2's device table, in both axes:

| symptom | cause | fix |
|---|---|---|
| first character clipped on the left | window starts 2 columns later than U8g2 assumes | shift drawing 2 px |
| dirty stripe down the right edge | 2 visible columns lie outside U8g2's 72-column buffer, so they keep power-on garbage | zero the controller RAM once at boot |
| clipped at the bottom, space at the top | window starts on a different COM row | SSD1306 `0xD3` display offset = 12 |

The vertical one cannot be fixed by moving the drawing — the buffer is 40 rows and
content already starts at row 0. Register `0xD3` moves the window itself. U8g2's
hardware-I²C path also calls `Wire.begin()` with no arguments, which picks the
chip's default pins rather than the board's; use the software-I²C constructor. And do
not borrow pre-charge (`0xD9`) or VCOMH (`0xDB`) values from generic 128×64 init
sequences — they blanked this panel.

**ESP32-C3 battery sense** — a 1:2 divider (100 k / 100 k + 100 nF) on `GPIO0`. The
C3's ADC is linear only to about 2.5 V, unlike the original ESP32, and `GPIO2` — the
other free ADC1 channel — is a boot strapping pin a divider can hold on its
threshold. Sample between transmissions: a 66 mA burst reads as a flat cell.

**ESP32-C3 temperature** is the die, not the room — useful for trends only. The
original ESP32 has no usable sensor at all.

### STM32F030 IMU link

An **STM32F030K6T6**
reads an **MPU6881** over software I²C, fuses roll and pitch with a complementary
filter, measures its own battery, and sends ten auto-acked packets a second to an
ESP32-C3 with the 0.42" OLED, which cycles eight pages from an artificial horizon
to a link-quality view. By [r-d-PB](https://github.com/r-d-PB), MIT licensed and
included with thanks.

```
cd extras/XN297L_Demo
pio run -e imu_rx_c3    -t upload      # display board, over USB
pio run -e imu_tx_stm32 -t upload      # sensor board, over ST-Link
```

Its full write-up -- wiring, battery sense, the packet format, troubleshooting --
is in [`IMU_LINK.md`](extras/XN297L_Demo/IMU_LINK.md).

It is the proof that the library fits a small part: the whole sender — sensor,
filter, battery ADC and radio, register dump included — is 30.8 KB of the
STM32F030K6's 32 KB. That is on PlatformIO's `ststm32` 19.7.1 (STM32duino 2.12),
which the project pins. The STM32duino 3.0 core in 20.0.0 adds about 5.8 KB, and
there the sender only fits with LTO on top of the flag below -- 29.9 KB, but not
yet run on hardware. Two things make the budget work at all, and both carry over
to other projects:

- `analogRead()` drags in about 2.9 KB of HAL. Driving the ADC through its
  registers does the same job in roughly 500 bytes, and reading VREFINT alongside
  the divider references the battery to the real VDDA instead of an assumed 3.3 V —
  which matters exactly when the cell is low and the regulator starts to drop out.
- `-DXN297L_NO_DETAILS` hands back the register dump's 1.2 KB when the last of
  the flash counts -- 30.8 KB drops to 29.5 KB.

The packet's first 16 bytes are laid out like the dashboard's, so each receiver can
read the other project's sender and ignore what it does not know.

---

## Roadmap

- 64-byte payloads (`FEATURE.DATA_LEN_SEL`)
- Software CE over SPI (`CE_FSPI_ON/OFF`), freeing a GPIO
- An interrupt-driven example
- A/B test of Panchip's reference calibration values against the power-on defaults
- The 3-wire XN297LBW (SOP8), which shares one data line for MOSI and MISO

---

## Sources

- **Panchip XN297L Technology Reference Manual** —
  [v5.2, Sep 2022](https://www.panchip.com/static/upload/file/20221014/1665726592628185.pdf),
  the newest; it adds nothing about RSSI.
- [foldedtoad/xn297_cal](https://github.com/foldedtoad/xn297_cal) — carries
  `XN297_complete.pdf`, the full Chinese XN297 datasheet, plus bit-field maps for the
  calibration registers. The key to the RSSI mechanism.
- [IOsetting/py32f0-template](https://github.com/IOsetting/py32f0-template) —
  an XN297L driver for PY32 that preserves Panchip's reference calibration values.
- [nRF24/RF24](https://github.com/nRF24/RF24) — the API this library follows.
- [Deviation forum — WLtoys Q242G](https://www.deviationtx.com/forum/protocol-development/5290-wltoys-q242g)
  — early discussion of register `0x09`.

---

## License

MIT. The register findings are facts about someone else's silicon — use them.
