# XN297L demo

Every board the [XN297L library](../../README.md) has run on, as one PlatformIO
project built against the library in the repository root. Open this folder in
VS Code and each firmware is an env in the PlatformIO sidebar.

| env | board | what it does |
|---|---|---|
| `tx` | ESP32 WROOM | initiator: a packet a second, reads back the responder's view of the link |
| `rx` | ESP32 WROOM | responder |
| `tx_c3` | ESP32-C3 SuperMini | initiator with battery sense, temperature and an LED |
| `rx_c3` | ESP32-C3 + 0.42" OLED | responder with a six-page display |
| `imu_tx_stm32` | STM32F030K6T6 + MPU6881 | IMU sender, flashed over ST-Link |
| `imu_rx_c3` | ESP32-C3 + 0.42" OLED | IMU receiver, eight pages |
| `scan` | ESP32 WROOM | the register sweep that found the RSSI enable |

`tx`, `tx_c3`, `rx` and `rx_c3` speak one protocol and pair in any combination.
`imu_tx_stm32` pairs with `imu_rx_c3`.

```
pio run -e tx_c3 -t upload --upload-port COM7
pio run -e rx_c3 -t upload --upload-port COM6
```

Wiring, and what was learned building all of this, is in the
[library README](../../README.md). The IMU link has its own write-up in
[IMU_LINK.md](IMU_LINK.md); its code is MIT licensed by r-d-PB, see
[IMU_LINK_LICENSE](IMU_LINK_LICENSE).
