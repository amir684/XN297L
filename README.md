# XN297L + ESP32

A working driver and bring-up toolkit for the **Panchip XN297L** 2.4 GHz transceiver,
written from the datasheet and validated against modules desoldered from a 2015
Chinese toy drone.

It also documents three things that are **not in any XN297L datasheet**:

- **RSSI works on the XN297L** — the register is documented but permanently reads
  zero, because the enable lives somewhere Panchip never published. Located here by
  bit sweep, and verified against distance.
- **Four register-level differences from the nRF24L01** that silently break ported
  drivers. One of them stops the radio transmitting while SPI still looks perfect.
- **Calibration for the 0.42" 72×40 OLED** on the ESP32-C3 boards, which is cut
  differently from what U8g2's device table assumes — in both axes.

Everything below was measured on hardware. Failures and dead ends are documented
alongside the results, because knowing what was already ruled out is most of the
value.

---

## Contents

- [Hardware](#hardware)
- [Quick start](#quick-start)
- [Wiring](#wiring)
- [The XN297L is not an nRF24L01](#the-xn297l-is-not-an-nrf24l01)
- [RSSI: the undocumented feature](#rssi-the-undocumented-feature)
- [Link quality without RSSI](#link-quality-without-rssi)
- [The 0.42" OLED](#the-042-oled)
- [Display pages](#display-pages)
- [Project layout](#project-layout)
- [Sources](#sources)

---

## Hardware

| Part | Notes |
|---|---|
| **XN297L module** | `GB-XN297L`, dated 2015-09-08, salvaged from a toy drone remote. 8 pads: `3V3 GND CLK MISO MOSI CSN CE IRQ`. 16 MHz crystal, wire antenna. |
| **ESP32 WROOM** | Original dev boards, envs `tx` / `rx`. |
| **ESP32-C3 SuperMini** | Native USB, on-board LED on GPIO8. Env `tx_c3`. |
| **ESP32-C3 + 0.42" OLED** | 72×40 SSD1306, 01Space-style board. Env `rx_c3`. |

Supply is **2.2–3.3 V** — the ESP32's 3V3 rail sits at the very top of that range.
Put **10 µF across the module's 3V3/GND**, close to the pads. The reference design
shows 1 µF; on breadboard wiring with the radio pulling 30–66 mA in bursts, 10 µF
is not excessive. Supply sag is the single most common cause of a link that works
intermittently and looks like a range problem.

---

## Quick start

```bash
# the original pair
pio run -e tx    -t upload --upload-port COM3
pio run -e rx    -t upload --upload-port COM4

# ESP32-C3 pair, with the OLED on the receiver
pio run -e tx_c3 -t upload --upload-port COM7
pio run -e rx_c3 -t upload --upload-port COM6

pio device monitor -b 115200
```

| env | board | role |
|---|---|---|
| `tx` | ESP32 WROOM | initiator |
| `rx` | ESP32 WROOM | responder |
| `tx_c3` | ESP32-C3 SuperMini | initiator, LED status |
| `rx_c3` | ESP32-C3 + OLED | responder, 5-page display |
| `scan` | either | register sweep used to find RSSI |

All four roles speak the same protocol, so any pairing works — WROOM to C3, C3 to
C3, whatever is on the bench.

Expected output:

```
SPI self-test PASSED -- the module is alive.
[    21] Ttx=34.2C  i hear you=6 (mean 6)  you hear me=7  lost=0  seen=22
```

---

## Wiring

### ESP32 WROOM

| XN297L | ESP32 | |
|---|---|---|
| `3V3` | `3V3` | **not 5 V** |
| `GND` | `GND` | |
| `CLK` | `GPIO18` | VSPI SCK |
| `MISO` | `GPIO19` | VSPI MISO |
| `MOSI` | `GPIO23` | VSPI MOSI |
| `CSN` | `GPIO5` | |
| `CE` | `GPIO4` | |
| `IRQ` | `GPIO16` | wired, not yet used — the driver polls |

### ESP32-C3 (both boards)

| XN297L | ESP32-C3 |
|---|---|
| `CLK` | `GPIO4` |
| `MISO` | `GPIO3` |
| `MOSI` | `GPIO10` |
| `CSN` | `GPIO7` |
| `CE` | `GPIO1` |
| `IRQ` | `GPIO0` |

`GPIO5`/`6` are the OLED's I²C, `2`/`8`/`9` are strapping pins, `20`/`21` are the
UART. The map above avoids all of them, and is shared by **both** C3 boards —
including the SuperMini that has no display and could spare 5/6 — so one jumper
harness fits both and swapping roles needs no rewiring.

---

## The XN297L is not an nRF24L01

The register map looks compatible, which is exactly the trap. Four differences
matter, and each is marked at its definition in [`src/xn297l.h`](src/xn297l.h).

### 1. `CONFIG` bit 7 is `EN_PM`, and it must be set

The big one. On the nRF24L01 this bit is reserved-zero. On the XN297L, Table 4-1
requires `EN_PM = 1` for **both** RX and TX — it is what moves the chip from
standby-I to standby-III, and TX and RX are only reachable from there.

A driver ported from nRF24 code leaves it clear. The symptom is brutal to debug:
SPI works perfectly, the self-test passes, the register dump looks right, and not
one packet ever moves.

```c
_baseConfig = XN_CFG_EN_PM | XN_CFG_EN_CRC | XN_CFG_CRCO | XN_CFG_PWR_UP;
```

### 2. `RF_SETUP` is a 2-bit rate and a 6-bit power code

| field | bits | values |
|---|---|---|
| `RF_DR` | 7:6 | `00` = 1 Mbps, `01` = 2 Mbps, `11` = 250 kbps |
| `RF_PWR` | 5:0 | `100111` = 11 dBm, `010101` = 9, `101100` = 5, `010100` = 4, `101010` = −1, `011001` = −10, `110000` = −23 |

The power field is a **code from a fixed table**, not the nRF24's 0–3 scale. An
nRF24 "max power" value lands on a combination that does not exist.

Current draw is worth knowing before choosing: **11 dBm ≈ 66 mA, 9 dBm ≈ 30 mA.**
Two dB for less than half the current makes 9 dBm the better default on a dev
board regulator.

### 3. Reset is a two-command sequence

```c
0x53, 0x5A   // hold in reset
0x53, 0xA5   // release
```

The data byte is not optional. A bare `0x53` leaves the chip latched in reset.

### 4. SPI is capped at 1 Mbps in power-down and standby-I

Which is where all of initialisation happens. `begin()` runs at 1 MHz and only
steps up to 4 MHz after reaching standby-III.

### Also

- `FEATURE` carries `MUX_PA_IRQ`, `CE_SEL` and `DATA_LEN_SEL`, none of which exist
  on the nRF24.
- The datasheet's `FIFO_STATUS` table has the names and descriptions of bits 0 and
  1 **crossed**. This driver uses `STATUS.RX_P_NO` (`111` = empty) instead, which
  is unambiguous.
- Channels that are multiples of 16 MHz — 0, 16, 32, 48, 64, 80 — cost about 2 dB
  of sensitivity, because they land on the crystal's harmonics. Default here is 78.

---

## RSSI: the undocumented feature

**Short version:** the XN297L can report RSSI. It is off by default, the enable is
not in the XN297L datasheet, and turning it on costs 6 dB of receive sensitivity.

### The problem

Register `0x09` is documented — `RSSI_RT` in bits 7:4, `RSSI_SY` in bits 3:0 — but
it is marked `09*`, *"Special Function Register ... declared in the software design
reference"*, a document Panchip never published. It reads `0x00` no matter when you
sample it.

### Finding the reference

The full **XN297** (non-L) Chinese datasheet documents the whole mechanism, and a
copy survives as `XN297_complete.pdf` in
[foldedtoad/xn297_cal](https://github.com/foldedtoad/xn297_cal):

| location | field | meaning |
|---|---|---|
| `RF_SETUP` bit 7 | `RSSI_EN` | `1` = enabled. **Resets to 0** |
| `RF_SETUP` bit 5 | `RSSI_SEL` | sample through filter or not |
| `CONFIG` bit 7 | `DATAOUT_SEL` | selects what `0x09` reports |
| `RF_CAL` bits 48:47 | `RSSI_GAIN_CTR` | `00` = 0 dB, `01` = −6, `10` = −12, `11` = −18 |

Panchip marks all of it **(测试用)** — *"for test use"*.

### Why that did not work

Both enable bits were **remapped on the L variant**: `RF_SETUP` bit 7 became the
top of `RF_DR`, and `CONFIG` bit 7 became `EN_PM`.

Confirmed by experiment. Setting `RF_SETUP` bit 7:

```
[   1] rx #0   lost=0   seen=1   rssi=00      <- data still arrived
[   1] NO ACK -- the other board did not receive this one
```

Data crossed, the auto-ACK stopped, `0x09` stayed at zero. That is `RF_DR` moving
to `0b10` — **Reserved** in the table — not `RSSI_EN`. Both boards changed together
so packets still got through, but `RX_ACK_TIME` scales with the data rate, so the
ACK window broke.

### Finding it by sweep

The `scan` env flips **one bit at a time** in the undocumented registers `0x19`,
`0x1B` and `0x1E` — 56 candidates — and watches `0x09`. Single-bit flips rather
than brute force: seven bytes is 2⁵⁶ combinations, and staying one bit away from
the factory state is both the more informative search and the safer one.

**The first sweep produced three false positives.** Its test for "reception still
works" was only that something reached the RX FIFO — which noise satisfies. The
bits it flagged break packet validation, after which the chip decodes noise
continuously and `0x09` fills with garbage:

```
[ 34050] rx #572657937    lost=12963   rssi=88
[ 34051] rx #1145315874   lost=17331   rssi=22
```

Adding a magic word to the payload and counting **valid packets separately from
junk** fixed the test. Two candidates survived, with `junk=0`:

| bit in `0x1E` | `RSSI_GAIN_CTR` | `0x09` |
|---|---|---|
| 15 | `01` = −6 dB | `0x66` |
| 16 | `10` = −12 dB | `0x06` |

They are adjacent, and they land exactly where `RSSI_GAIN_CTR` sits in the XN297
field map — **a position predicted before the sweep ran** — if the L variant's
24-bit `0x1E` is the top 24 bits of the XN297's 56-bit `RF_CAL`. The readings
behave like an attenuator: `RSSI_RT` drops from 6 to 0 as the field goes from
`01` to `10`.

There appears to be **no separate enable on the XN297L**. `RSSI_GAIN_CTR = 00`,
the reset value, simply means no measurement.

### Verified

Readings track distance: about **8 touching, 1 in another room**, with `lost=0`
throughout.

Use the **low nibble, `RSSI_SY`**, scale 0–15. The high nibble `RSSI_RT` reads 0 in
practice — it is a real-time measurement and the burst is over before software can
read the register. `RSSI_SY` is latched at packet sync and holds.

In practice the readings span **0–8**, not 0–15, so bar graphs should scale to 8.

### The cost

The field is *"RSSI 的信号增益衰减的选择位"* — **signal gain attenuation** select
bits. The attenuation is **not confined to the detector**. It costs real receive
sensitivity, and with it enabled on both ends the link fell apart:

```
[   182] ok  retries=2   (no answer)      answers=0/183
[   185] NO ACK -- the other board did not receive this one
```

The forward packet survived because auto-ACK gives it 15 retransmits to absorb the
loss. The telemetry answer is sent `NOACK` and has no such margin, so it was the
half that vanished entirely.

### The fix: duty-cycle it

The attenuation only exists **while the bits are set**, and setting them is one SPI
write. So rather than choosing which board pays, both boards arm RSSI for one round
in `RSSI_SAMPLE_EVERY` (default 4) and run at full sensitivity the rest of the time.
Both directions get measured, the link spends three rounds in four with nothing in
its way, and each side carries its last reading forward so the display stays steady.

```c
static const int      RSSI_GAIN         = 1;   // 0 off, 1 -6dB, 2 -12dB, 3 -18dB
static const uint32_t RSSI_SAMPLE_EVERY = 4;
```

**Treat the number as relative.** It is a 4-bit test-mode reading with ±2 counts of
noise; average it. It is good for comparing antennas or placements, not for
absolute dBm.

---

## Link quality without RSSI

Two metrics that are documented, free, and cost no sensitivity:

- **`retries`** — `ARC_CNT` from `OBSERVE_TX`. How many retransmits the packet
  needed. `0` is a clean link.
- **`lost`** — gaps in the counter sequence at the receiver. Packets that never
  arrived at all.

These are the ones to build on for anything long-running.

The protocol reflects this split. The forward packet is **ACKed**, so the initiator
learns `retries`. The telemetry answer is deliberately **not** ACKed
(`W_TX_PAYLOAD_NOACK`, which needs `ACTIVATE 0x50 0x73` and `EN_NOACK` in
`FEATURE`): it is pure telemetry riding on an exchange the peer already
acknowledged. Demanding an ACK for it produced 15 pointless retransmits per round
into a board that had already stopped listening.

---

## The 0.42" OLED

The 72×40 panel on the ESP32-C3 boards needed **three separate corrections**, all
symptoms of one fact: **this panel is cut differently from what U8g2's device table
assumes.**

### Two things that bite everyone

1. It is a **72×40 window inside a 128×64 SSD1306**, offset `(28,24)`. Drive it as
   a plain 128×64 and the image lands off-screen. U8g2 has a device entry for the
   part, so use it.
2. U8g2's **hardware-I²C path calls `Wire.begin()` with no arguments**, which picks
   the chip's default SDA/SCL rather than the board's. Use the software-I²C
   constructor with explicit pins — the panel is 360 bytes, speed is irrelevant.

### The three corrections

| symptom | cause | fix |
|---|---|---|
| first character clipped on the left | window starts 2 columns later than U8g2 assumes | `OLED_X = 2`, shifts drawing |
| dirty stripe down the right edge | 2 visible columns lie **outside** the 72-column buffer, so U8g2 never writes them and they keep power-on garbage | zero the whole 128×64 GDDRAM once at boot |
| clipped at the bottom, space at the top | window starts on a different COM row | `0xD3` display offset = **12** |

The vertical one is the interesting case: it **cannot** be fixed by moving the
drawing. The buffer is 40 rows and the content already starts at row 0 — there is
nothing above to move into. SSD1306 register `0xD3` moves *the window itself*,
which is the only knob that works.

`OLED_CALIBRATE = true` runs a sweep for a different panel: it draws a frame around
the whole buffer and steps `0xD3` from 0 to 32 with the value in large digits.
Whichever value sits flush against all four edges is the answer.

### Brightness

`oled.setContrast(255)` is the safe, documented lever.

**Do not** copy pre-charge (`0xD9`) and VCOMH (`0xDB`) values out of generic 128×64
init sequences. U8g2 tunes those for this specific glass during `begin()`;
overriding them blanked the panel here — both with an out-of-range VCOMH and with a
legal one. That path is left in the code behind `OLED_TUNE_ANALOG`, defaulting off,
with a note saying it failed.

Everything is volatile, so a power cycle recovers from any of it.

These 0.42" panels are simply dim — 72×40 pixels at 1/40 duty on a tiny glass. If
brightness matters, a 0.91" or 1.3" SSD1306 uses the same I²C interface and this
code with only the device constant and offsets changed.

---

## Display pages

The **BOOT button (GPIO9)** cycles five pages. It is polled every pass of `loop()`,
so the page turns under your thumb rather than waiting for the next packet.

| page | content |
|---|---|
| `OVERVIEW` | counter, peer temperature centred, signal bar, both RSSI values, link state |
| `TEMP` | transmitter's temperature in large digits |
| `BATT` | transmitter's cell voltage, percentage, battery gauge |
| `SIGNAL` | RSSI large, running mean, wide bar |
| `STATS` | seen / lost / uptime |
| `GRAPH` | temperature trace, last 68 samples, auto-scaled |

The graph auto-scales to the range actually present — on a die that drifts a degree
over minutes, a fixed scale draws a flat line and says nothing. The number in the
top right is the span the trace covers.

Holding BOOT **through a reset** drops the C3 into download mode. That is the
button doing its original job, not a fault.

### Battery sense on the initiator

A single Li-ion cell through a 1:2 divider into `GPIO0` (ADC1_CH0):

```
BAT+ ---[ R1 100k ]---+--- GPIO0
                      |
                      +---[ R2 100k ]--- GND
                      |
                      +---[ 100nF ]----- GND
```

**Why these values.** The C3's ADC is linear only to about **2.5 V** — unlike the
original ESP32, which reaches ~3.1 V — so a full 4.2 V cell has to land at 2.10 V.
That is what 1:2 is for, and it leaves 0.4 V of headroom while still using 60% of
the range. Drain is 21 µA, negligible beside the C3 itself.

The **100 nF is not optional**: the ADC samples through an internal capacitor, and
a 50 kΩ source cannot charge it in the sampling window. Without it, readings come
out low and noisy.

**Why `GPIO0`.** ADC1 on the C3 is GPIO0–4 only (ADC2 is unusable). `1`, `3` and
`4` carry `CE`/`MISO`/`CLK`. That leaves `GPIO0` and `GPIO2` — and `GPIO2` is a
**boot strapping pin**, where a divider parking it near 2.1 V sits right on the VIH
threshold and makes booting intermittent. `GPIO0` is nominally `IRQ` in the shared
pin map, but nothing uses it: the driver polls. On the initiator, leave `IRQ`
unconnected and the pin is free.

**Calibration.** `BATT_CAL` in [`src/main.cpp`](src/main.cpp) trims out resistor
tolerance and residual ADC error:

1. read the cell with a multimeter
2. read what the board reports
3. `BATT_CAL = multimeter / reported`

**Sampling.** The reading is taken at the top of `loop()`, after the inter-packet
delay, while the radio is idle. Measuring during a transmit catches the 66 mA burst
pulling the rail down and reports a flat cell.

Percentage comes from an open-circuit Li-ion curve. Voltage alone sags under load
and recovers at rest, so treat it as an indication, not a fuel gauge — and note
that a SuperMini's LDO needs roughly 3.4 V in to hold 3.3 V out, so it falls out of
regulation before the cell is genuinely empty.

### Temperature

The ESP32-C3 has a usable on-chip temperature sensor; the original ESP32 does not.
Boards without one send a sentinel and the display shows `--` rather than a number
that would look like a measurement.

It reads **die temperature, not ambient** — expect it to sit above room temperature
by an amount that moves with CPU load. Good for trends and for chip thermal
monitoring; not a substitute for a real sensor.

---

## Project layout

```
src/
  xn297l.h      driver: register map, SPI commands, API
  xn297l.cpp    driver implementation
  main.cpp      all roles, selected by build flag
platformio.ini  five environments
README.he.md    Hebrew version of this document
```

The driver is deliberately minimal — fixed 32-byte payloads, one pipe, auto-ack on.
It is a bring-up driver meant to prove hardware works and then grow into a sensor
link. Natural next steps are marked in the code: dynamic payload length, using the
`IRQ` pin instead of polling, and pipes 1–5 for multiple transmitters.

---

## Sources

- [foldedtoad/xn297_cal](https://github.com/foldedtoad/xn297_cal) — carries
  `XN297_complete.pdf`, the full Chinese XN297 datasheet, plus bit-field maps for
  `BB_CAL` / `RF_CAL` / `DEMOD_CAL`. The key to the RSSI mechanism.
- [XN297L Technology Reference Manual v5.2, Sep 2022](https://www.panchip.com/static/upload/file/20221014/1665726592628185.pdf)
  — the newest official document. Checked: it adds nothing about RSSI.
- [Deviation forum — WLtoys Q242G](https://www.deviationtx.com/forum/protocol-development/5290-wltoys-q242g)
  — first mention of `DATAOUT_SEL` and register `0x09`.

---

## License

MIT. The datasheet findings are facts about someone else's silicon — use them.
