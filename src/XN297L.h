/*
 * XN297L -- driver for the Panchip XN297L 2.4 GHz transceiver.
 * https://github.com/amir684/XN297L
 *
 * The API follows the RF24 library, so code written for an nRF24L01 ports
 * across almost line for line. The chip underneath is not an nRF24L01, though:
 * four register-level differences break drivers written for one, and each is
 * handled and explained where it lives in XN297L.cpp.
 *
 * API-compatible is not air-compatible. The XN297 family frames its packets
 * differently from the nRF24L01, so an XN297L does not talk to an nRF24L01 radio
 * directly.
 *
 * Written against the Panchip XN297L Technology Reference Manual (v4.8 and
 * v5.2). Section and table numbers in the comments refer to it.
 *
 * MIT licensed.
 */
#ifndef XN297L_H
#define XN297L_H

#include <Arduino.h>
#include <SPI.h>

#define XN297L_VERSION "1.0.0"

// The global SPI object is not a SPIClass on every core -- arduino-pico calls
// its class SPIClassRP2040, for one -- so take the type from the object itself.
#if defined(__AVR__)
typedef SPIClass XN297L_SPIClass;
#else
#include <type_traits>
typedef std::remove_reference<decltype(SPI)>::type XN297L_SPIClass;
#endif

// SPI clock once the radio is running. Initialisation always runs at 1 Mbps
// regardless: the chip caps SPI there in power-down and standby-I (Section 7).
#define XN297L_SPI_SPEED 4000000UL

// ---- SPI commands (Table 7-2) -----------------------------------------------
#define XN297L_CMD_R_REGISTER          0x00  // | register
#define XN297L_CMD_W_REGISTER          0x20  // | register
#define XN297L_CMD_R_RX_PAYLOAD        0x61
#define XN297L_CMD_W_TX_PAYLOAD        0xA0
#define XN297L_CMD_FLUSH_TX            0xE1
#define XN297L_CMD_FLUSH_RX            0xE2
#define XN297L_CMD_REUSE_TX_PL         0xE3
#define XN297L_CMD_ACTIVATE            0x50  // + 0x73
#define XN297L_CMD_R_RX_PL_WID         0x60
#define XN297L_CMD_W_ACK_PAYLOAD       0xA8  // | pipe
#define XN297L_CMD_W_TX_PAYLOAD_NOACK  0xB0
#define XN297L_CMD_RST_FSPI            0x53  // + 0x5A hold / 0xA5 release
#define XN297L_CMD_NOP                 0xFF

// ---- registers (Table 8-1) --------------------------------------------------
#define XN297L_REG_CONFIG       0x00
#define XN297L_REG_EN_AA        0x01
#define XN297L_REG_EN_RXADDR    0x02
#define XN297L_REG_SETUP_AW     0x03
#define XN297L_REG_SETUP_RETR   0x04
#define XN297L_REG_RF_CH        0x05
#define XN297L_REG_RF_SETUP     0x06
#define XN297L_REG_STATUS       0x07
#define XN297L_REG_OBSERVE_TX   0x08
#define XN297L_REG_RSSI         0x09  // RSSI_RT 7:4, RSSI_SY 3:0
#define XN297L_REG_RX_ADDR_P0   0x0A  // P1 = 0x0B ... P5 = 0x0F
#define XN297L_REG_TX_ADDR      0x10
#define XN297L_REG_RX_PW_P0     0x11  // P1 = 0x12 ... P5 = 0x16
#define XN297L_REG_FIFO_STATUS  0x17
#define XN297L_REG_DYNPD        0x1C
#define XN297L_REG_FEATURE      0x1D
// Analog / calibration block. Does not exist on the nRF24L01, and the manual
// gives widths but not contents -- see the RSSI notes in XN297L.cpp.
#define XN297L_REG_DEMOD_CAL    0x19  // 1 byte
#define XN297L_REG_RF_CAL2      0x1A  // 6 bytes
#define XN297L_REG_DEM_CAL2     0x1B  // 3 bytes
#define XN297L_REG_RF_CAL       0x1E  // 3 bytes
#define XN297L_REG_BB_CAL       0x1F  // 5 bytes

// Output power. RF_PWR is a 6-bit code from a fixed table, not the nRF24's
// four-step scale, and the steps are uneven. Transmit current climbs steeply at
// the top: about 30 mA at 9 dBm against 66 mA at 11 dBm.
typedef enum {
  XN297L_PA_MINUS23DBM = 0x30,
  XN297L_PA_MINUS10DBM = 0x19,
  XN297L_PA_MINUS1DBM  = 0x2A,
  XN297L_PA_4DBM       = 0x14,
  XN297L_PA_5DBM       = 0x2C,
  XN297L_PA_9DBM       = 0x15,
  XN297L_PA_11DBM      = 0x27,

  // RF24-style names
  XN297L_PA_MIN  = XN297L_PA_MINUS23DBM,
  XN297L_PA_LOW  = XN297L_PA_MINUS1DBM,
  XN297L_PA_HIGH = XN297L_PA_9DBM,
  XN297L_PA_MAX  = XN297L_PA_11DBM,
} xn297l_pa_dbm_e;

typedef enum {
  XN297L_1MBPS   = 0,
  XN297L_2MBPS   = 1,
  XN297L_250KBPS = 3,
} xn297l_datarate_e;

typedef enum {
  XN297L_CRC_DISABLED = 0,
  XN297L_CRC_8,
  XN297L_CRC_16,
} xn297l_crclength_e;

// RSSI_GAIN_CTR. Any setting other than OFF switches the RSSI detector on, and
// every one of them costs receive sensitivity -- see enableRSSI().
typedef enum {
  XN297L_RSSI_OFF       = 0,
  XN297L_RSSI_MINUS6DB  = 1,
  XN297L_RSSI_MINUS12DB = 2,
  XN297L_RSSI_MINUS18DB = 3,
} xn297l_rssi_atten_e;

class XN297L {
public:
  XN297L(uint16_t cePin, uint16_t csnPin);

  // Starts the SPI bus if needed, then resets and configures the chip. For
  // non-default SPI pins (ESP32), call SPI.begin(sck, miso, mosi) first.
  // Returns true when the chip answers over SPI -- see isChipConnected().
  bool begin(XN297L_SPIClass *spi = &SPI, uint32_t spiSpeed = XN297L_SPI_SPEED);

  // Round-trips two patterns through TX_ADDR and puts the original back, so it
  // proves MOSI, MISO, CLK and CSN together and is safe to call at any time.
  bool isChipConnected();

  // ---- RF24-compatible ------------------------------------------------------
  void powerUp();
  void powerDown();

  void startListening();
  void stopListening();

  bool available();
  bool available(uint8_t *pipeNum);
  void read(void *buf, uint8_t len);

  // Blocks until the packet is acknowledged or retries run out. multicast
  // sends without asking for an ACK, and returns once the packet is on air.
  bool write(const void *buf, uint8_t len);
  bool write(const void *buf, uint8_t len, bool multicast);

  void openWritingPipe(const uint8_t *address);
  void openReadingPipe(uint8_t pipe, const uint8_t *address);  // pipes 0-5
  void closeReadingPipe(uint8_t pipe);

  void setAddressWidth(uint8_t width);              // 3-5 bytes
  void setRetries(uint8_t delay, uint8_t count);    // delay: (n+1)*250 us
  void setChannel(uint8_t channel);                 // 2400 + channel MHz
  uint8_t getChannel();

  void setPayloadSize(uint8_t size);                // 1-32, static payloads
  uint8_t getPayloadSize();
  void enableDynamicPayloads();
  void disableDynamicPayloads();
  uint8_t getDynamicPayloadSize();

  void enableAckPayload();
  bool writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
  void enableDynamicAck();

  void setAutoAck(bool enable);
  void setAutoAck(uint8_t pipe, bool enable);

  void setPALevel(xn297l_pa_dbm_e level);
  xn297l_pa_dbm_e getPALevel();
  bool setDataRate(xn297l_datarate_e rate);
  xn297l_datarate_e getDataRate();
  void setCRCLength(xn297l_crclength_e length);
  xn297l_crclength_e getCRCLength();
  void disableCRC();

  uint8_t getARC();                                 // retransmits, last write
  bool rxFifoFull();
  void flush_rx();
  void flush_tx();
  void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);
  void whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready);
  void printDetails(Stream &out = Serial);

  // ---- XN297L only ----------------------------------------------------------

  // Switches the undocumented RSSI detector on. The reading is relative, not
  // dBm, and the attenuation is real: expect about 6 dB less receive
  // sensitivity while it is on. Enable it for the packets you want to measure
  // and disable it again -- it is a register write, cheap to toggle, and safe
  // to call while listening.
  void enableRSSI(xn297l_rssi_atten_e attenuation = XN297L_RSSI_MINUS6DB);
  void disableRSSI();

  // RSSI_SY, 0-15, latched by available() when it finds a packet. Only
  // meaningful while RSSI is enabled.
  uint8_t getRSSI();

  // ---- register access ------------------------------------------------------
  uint8_t readRegister(uint8_t reg);
  void readRegister(uint8_t reg, uint8_t *buf, uint8_t len);
  void writeRegister(uint8_t reg, uint8_t value);
  void writeRegister(uint8_t reg, const uint8_t *buf, uint8_t len);
  uint8_t getStatus();

private:
  void beginTransaction();
  void endTransaction();
  void ce(bool level);
  uint8_t command(uint8_t cmd);
  uint8_t command(uint8_t cmd, uint8_t data);
  void setRSSIControl(uint8_t code);
  void printRow(Stream &out, const __FlashStringHelper *name, uint8_t reg,
                uint8_t len);

  uint16_t _cePin;
  uint16_t _csnPin;
  XN297L_SPIClass *_spi;
  uint32_t _spiSpeed;        // running speed
  uint32_t _spiNow;          // speed in use -- capped while powered down
  uint8_t  _status;
  uint8_t  _config;          // shadow of CONFIG
  uint8_t  _payloadSize;
  uint8_t  _addrWidth;
  uint8_t  _pipe0Writing[5];
  uint8_t  _pipe0Reading[5];
  bool     _p0Rx;
  bool     _dynamicPayloads;
  bool     _ackPayloads;
  bool     _ceHigh;
  bool     _rssiOn;
  uint8_t  _lastRssi;
};

#endif  // XN297L_H
