# Wireless IMU link: STM32F030 + MPU6881 -> XN297L -> ESP32-C3 OLED

> An example project of the [XN297L library](../../README.md). It began as a
> standalone project (MIT, copyright r-d-PB -- see [LICENSE](LICENSE)) with its
> own copy of the radio driver; here the same firmware builds against the
> library in the root of this repository. Run the commands below from this
> folder.

A 2.4 GHz telemetry link built out of two very small boards.

An **STM32F030K6T6** reads an **MPU6881** over software I2C, fuses the
accelerometer and gyro into roll and pitch, measures its own battery, and
transmits ten packets a second through a **Panchip XN297L**. An **ESP32-C3**
with an on-board 0.42" OLED receives them and shows the attitude, the raw axes,
the sensor's die temperature, the cell voltage and the link quality, one page
at a time.

```
  +---------------------+                        +----------------------+
  |  STM32F030K6T6      |                        |  ESP32-C3 + 0.42"    |
  |  32 KB flash, 4 KB  |       2.4 GHz          |  OLED, 72x40         |
  |                     |    ch 78, 1 Mbps       |                      |
  |  MPU6881 --I2C-->   |  ==================>   |   --> 8 pages        |
  |  divider --ADC-->   |     32-byte packets    |       BOOT cycles    |
  |            --SPI--> |     10 per second      |                      |
  |            XN297L   |     auto-acked         |   XN297L             |
  +---------------------+                        +----------------------+
```

The whole sender fits in 32 KB of flash with room to spare, which is most of
what shapes the design decisions below.

## Why this is interesting

- **The XN297L is not an nRF24L01**, however much its register map looks like
  one. Four differences matter, and all four are called out where they are used
  in the library's [`src/XN297L.cpp`](../../src/XN297L.cpp). The one that stops everything dead is `CONFIG`
  bit 7: on the nRF24 it is reserved-zero, here it is `EN_PM`, and the radio
  will not key up without it.
- **The radio's RSSI is undocumented.** Panchip's XN297L manual lists register
  0x09 and then defers to a software reference that was never published. The
  library's `enableRSSI()` writes the bits that were found empirically in the
  upstream project, and the receiver's signal bar reads them.
- **Everything is sized for a 32 KB part.** `analogRead()` was the obvious way
  to read the battery and it cost 2.9 KB, so the ADC is driven through its
  registers instead, in about 500 bytes. That story is in the comments.
- **The packet is backwards compatible.** Its first 16 bytes are byte-for-byte
  the upstream demo's `LinkPacket`, so that project's stock receiver still
  decodes these packets and simply ignores the sensor half.

## Credits

The radio driver, the receiver's display plumbing and the RSSI discovery come
from [amir684/XN297L](https://github.com/amir684/XN297L). That is where the
XN297L's departures from the nRF24L01 were worked out, and where this panel's
72x40 geometry was measured rather than guessed.

## Layout

The radio driver is the XN297L library at the repository root.

```
platformio.ini                 two envs, one per board
include/link_packet.h          the wire format, shared by both ends
src/main_tx_stm32.cpp          sender: MPU6881 + battery -> radio
src/main_rx_c3.cpp             receiver: radio -> OLED
boards/genericSTM32F030K6.json board definition for the F030K6T6
reference/main_sensor_only.cpp the original sketch, sensor to serial, no radio
```

## Hardware

| Part | Notes |
| --- | --- |
| STM32F030K6T6 | 48 MHz Cortex-M0, 32 KB flash, 4 KB RAM |
| MPU6881 | register-compatible with the MPU6500 |
| XN297L module | 8-pad Panchip module with a 16 MHz crystal, one per board |
| ESP32-C3 0.42" OLED board | 01Space ESP32-C3-0.42LCD and clones |
| ST-Link | to flash the STM32 |
| USB-TTL adapter | to read the STM32's serial output |

## Wiring

### Sender, STM32F030K6T6

| Signal | Pin | Goes to |
| --- | --- | --- |
| SDA | PB6 | MPU6881 SDA |
| SCL | PB7 | MPU6881 SCL |
| SCK | PA5 | XN297L SCK |
| MISO | PA6 | XN297L MISO |
| MOSI | PA7 | XN297L MOSI |
| CE | PA4 | XN297L CE |
| CSN | PB0 | XN297L CSN |
| BATT | PA1 | centre of the battery divider |
| TX | PA2 | RX of the USB-TTL adapter |
| RX | PA3 | TX of the USB-TTL adapter |
| SWDIO / SWCLK | PA13 / PA14 | ST-Link |

The MPU's AD0 pin picks its address, low for 0x68 and high for 0x69. Either
works, the firmware probes both. The radio's IRQ pin stays unconnected: the
driver polls STATUS.

CE and CSN are plain GPIO as far as the driver is concerned, so it does not
matter that PA4 happens to be SPI1's hardware NSS pin.

The hardware I2C1 on this package is SCL=PB6 / SDA=PB7, the opposite of the
wiring above, so the sender bit-bangs I2C in software to match the wiring as it
already is. Swap the two sensor wires and set `USE_HW_I2C` to 1 if you would
rather use the peripheral.

### Receiver, ESP32-C3 with the 0.42" OLED

| Signal | GPIO |
| --- | --- |
| SCK | 4 |
| MISO | 3 |
| MOSI | 10 |
| CSN | 7 |
| CE | 1 |
| IRQ | 0 (wired, unused) |
| OLED SDA / SCL | 5 / 6, on board |
| page button | 9, the BOOT button |

Identical to the upstream repo's C3 pin map, so one jumper harness fits both
projects.

### Battery sense

A 1:2 divider from the cell to PA1:

```
BAT+ ---[ R1 100k ]---+--- PA1
                      |
                      +---[ R2 100k ]--- GND
                      |
                      +---[ 100nF ]----- GND
```

Two equal resistors, so a full 4.2 V cell lands at 2.10 V, well inside the
3.3 V the ADC can convert, and the pair draws about 21 uA. The 100 nF is what
lets a 50 kOhm source drive the sample-and-hold; leave it out and the reading
comes back low.

PA1 is one of the two ADC-capable pins this design leaves free. The other, PA0,
is left alone on purpose: it is the WKUP1 pin, and a divider parking it at
2.1 V would fight any later standby-and-wake work.

The sender reads VREFINT alongside the divider, so the number is referenced to
what VDDA actually is rather than to an assumed 3.3 V. That matters exactly
when the cell is low and the regulator has started to drop out.

To trim for real resistors: measure the cell with a multimeter, read what the
board reports, and set `BATT_CAL` in the sender to the ratio of the two. A
divider's error is pure gain, so one point corrects the whole range.

### Power

At +11 dBm the XN297L draws about 66 mA in bursts. Put 10 uF and 100 nF right
across the module's own supply pads on both boards. If the link is flaky or the
STM32 resets while transmitting, lower `RF_POWER` in the sender to
`XN297L_PA_4DBM` before suspecting anything else.

## Build and flash

Needs [PlatformIO](https://platformio.org/). Both boards live in one project:

```
pio run -e rx_c3    -t upload      # display board, over USB
pio run -e tx_stm32 -t upload      # sensor board, over ST-Link
pio run -e tx_stm32 -t monitor     # needs monitor_port set to the USB-TTL COM
pio run                            # builds the sender, the default env
```

Flash the receiver first so it is already listening, then the sender. The
receiver's screen goes from "waiting for a packet" to the horizon page on the
first packet that arrives. Both ends print to serial, so a link that will not
come up can be diagnosed from either side.

Current sizes:

| Env | Flash | RAM |
| --- | --- | --- |
| tx_stm32 | 30760 of 32768 | 1464 of 4096 |
| rx_c3 | 290734 of 1310720 | 15124 |

Measured on PlatformIO's `ststm32` 19.7.1 (STM32duino 2.12), which
`platformio.ini` pins for the sender. The STM32duino 3.0 core that 20.0.0 brings
adds about 5.8 KB: the sender overflows by 3.8 KB as it is, and still by 2.5 KB
with `XN297L_NO_DETAILS`. Adding `-flto` as well brings it to 29,880 bytes, but
that build has not been run on hardware.

## What the sender does

1. Scans I2C, finds the MPU6881, configures +/-2 g and +/-250 deg/s with the
   41 Hz filters and a 200 Hz sample rate.
2. Averages the gyro for two seconds to find its zero offset. **Keep the board
   still until the banner says the calibration is done** - whatever it is doing
   during those two seconds becomes its definition of "not moving".
3. Samples at 100 Hz and runs a complementary filter, 0.98 to the gyro. Roll
   and pitch only; yaw needs a magnetometer, which this part does not have.
4. Sends every tenth sample, so ten packets a second, each one auto-acked.

The sender never listens. Auto-ack is a hardware exchange, so it already learns
whether each packet arrived and how many retransmits it took, and a software
reply would add nothing. Dropping the reply is also what frees the receiver
from turnaround timing and makes a 10 Hz stream comfortable.

## The screen

The BOOT button cycles the pages:

| Page | Shows |
| --- | --- |
| HORIZON | artificial horizon, the line is the sender's idea of level |
| ANGLES | roll and pitch as large numbers |
| ACCEL | X / Y / Z in g |
| GYRO | X / Y / Z in degrees per second |
| TEMP | MPU6881 die temperature, large |
| BATT | cell voltage, rough state of charge, and a battery icon |
| GRAPH | that temperature over the last 68 packets, auto-scaled |
| LINK | signal bar, packet rate, packets seen and lost, retransmits, uptime |

Redrawing the panel over software I2C takes tens of milliseconds, so the screen
refreshes ten times a second at most and only when something changed.

## The packet

One 32-byte payload, defined once in `include/link_packet.h` and included by
both ends, so there is no chance of the two drifting apart:

| Bytes | Field | Notes |
| --- | --- | --- |
| 0-3 | magic | tells a real packet from noise in the FIFO |
| 4-7 | counter | gaps in it are lost packets |
| 8-9 | lost | unused by this sender |
| 10 | retries | ARC_CNT for the sender's previous packet |
| 11 | rssi | always "not measuring" from the STM32 |
| 12-13 | tempDeciC | MPU6881 die temperature |
| 14-15 | battMv | cell voltage in millivolts |
| 16-21 | accMg | X, Y, Z in milli-g |
| 22-27 | gyroDeciDps | X, Y, Z in tenths of a degree per second |
| 28-29 | rollDeci | tenths of a degree |
| 30-31 | pitchDeci | tenths of a degree |

Both ends are little-endian ARM and every field is naturally aligned, so the
layout is identical on the Cortex-M0 and the RISC-V C3. A `static_assert` keeps
that true if anyone edits the fields.

## Troubleshooting

**"SPI self-test FAILED"** - the radio is not answering on MISO. The firmware
dumps every register before it halts; all `00` or all `FF` means no SPI at all,
so check power, then MOSI/MISO not being swapped, then wire length, then the
16 MHz crystal.

**Sensor found, radio fine, no packets on the receiver** - channel, address and
payload size have to match on both ends. They are constants at the top of each
main file and both default to channel 78 with address `C2 C2 C2 C2 C2`.

**Packets arrive but `lost` climbs** - normal at the edge of range. Check the
LINK page: retransmits above zero mean the sender is working to get through.
Lower the data rate or raise the power before moving the antenna.

**The sender resets, or the sensor reads nonsense while transmitting** - the
supply is sagging under the transmit burst. Add the 10 uF, or drop `RF_POWER`.

**Roll and pitch drift** - the gyro was calibrated while the board was moving.
Reset it lying still. Yaw is not reported at all and cannot be: there is no
magnetometer to hold it.

**The battery reads high or low by a few percent** - that is the resistors, not
the ADC. Trim `BATT_CAL` as described under Battery sense.

**Flash is at 91% on the STM32** - 32 KB is the whole budget. The register dump
is the cheapest 1.2 KB to give back: uncomment `-D XN297L_NO_DETAILS` in
`platformio.ini` once the wiring is proven.

## License

MIT, see [LICENSE](LICENSE). The XN297L driver and the receiver's display code
derive from [amir684/XN297L](https://github.com/amir684/XN297L).
