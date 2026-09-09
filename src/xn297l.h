// -----------------------------------------------------------------------------
// Minimal Panchip XN297L driver for ESP32 / Arduino framework.
//
// Written against the Panchip "XN297L Technology Reference Manual", v4.8,
// May 2016. Section numbers in the comments refer to that document.
//
// The register map looks nRF24L01-compatible but is NOT, in four places that
// matter, all of them called out at their definitions below:
//   - CONFIG bit 7 is EN_PM, and it must be 1 for the radio to transmit or
//     receive at all (Table 4-1)
//   - RF_SETUP uses a 2-bit data rate and a 6-bit power code, both with their
//     own encodings (Register Map, 0x06)
//   - FEATURE carries CE_SEL / DATA_LEN_SEL / MUX_PA_IRQ (Register Map, 0x1D)
//   - reset is a two-step command, not a bare opcode (Table 7-2)
//
// Fixed 32-byte payloads, one pipe, auto-ack on. Deliberately simple: this is a
// bring-up driver meant to prove the hardware works, then grow into a sensor
// link.
// -----------------------------------------------------------------------------
#pragma once

#include <Arduino.h>
#include <SPI.h>

// ---- SPI commands (Table 7-2) -----------------------------------------------
#define XN_CMD_R_REGISTER          0x00  // | addr
#define XN_CMD_W_REGISTER          0x20  // | addr
#define XN_CMD_R_RX_PAYLOAD        0x61
#define XN_CMD_W_TX_PAYLOAD        0xA0
#define XN_CMD_FLUSH_TX            0xE1
#define XN_CMD_FLUSH_RX            0xE2
#define XN_CMD_REUSE_TX_PL         0xE3
#define XN_CMD_ACTIVATE            0x50  // + 0x73 to activate, 0x8C to undo
#define XN_CMD_R_RX_PL_WID         0x60
#define XN_CMD_W_ACK_PAYLOAD       0xA8  // | pipe
#define XN_CMD_W_TX_PAYLOAD_NOACK  0xB0
#define XN_CMD_CE_FSPI_ON          0xFD  // + 0x00, software CE (needs CE_SEL=1)
#define XN_CMD_CE_FSPI_OFF         0xFC  // + 0x00
#define XN_CMD_NOP                 0xFF

// Reset takes a data byte and is a HOLD/RELEASE pair, not a one-shot opcode.
// Sending a bare 0x53 leaves the chip latched in reset.
#define XN_CMD_RST_FSPI            0x53
#define XN_RST_HOLD                0x5A
#define XN_RST_RELEASE             0xA5

// ---- registers (Table 8-1) --------------------------------------------------
#define XN_REG_CONFIG      0x00
#define XN_REG_EN_AA       0x01
#define XN_REG_EN_RXADDR   0x02
#define XN_REG_SETUP_AW    0x03
#define XN_REG_SETUP_RETR  0x04
#define XN_REG_RF_CH       0x05
#define XN_REG_RF_SETUP    0x06
#define XN_REG_STATUS      0x07
#define XN_REG_OBSERVE_TX  0x08
#define XN_REG_RSSI        0x09   // RSSI_RT 7:4, RSSI_SY 3:0 (nRF24 calls it RPD)
#define XN_REG_RX_ADDR_P0  0x0A
#define XN_REG_RX_ADDR_P1  0x0B
#define XN_REG_TX_ADDR     0x10
#define XN_REG_RX_PW_P0    0x11
#define XN_REG_FIFO_STATUS 0x17
#define XN_REG_DYNPD       0x1C
#define XN_REG_FEATURE     0x1D
// Analog / calibration block. Not present on the nRF24L01. Widths matter --
// these are multi-byte registers and the datasheet gives the width of each.
#define XN_REG_DEMOD_CAL   0x19   // 1 byte
#define XN_REG_RF_CAL2     0x1A   // 6 bytes
#define XN_REG_DEM_CAL2    0x1B   // 3 bytes
#define XN_REG_RF_CAL      0x1E   // 3 bytes
#define XN_REG_BB_CAL      0x1F   // 5 bytes

// ---- CONFIG bits ----
#define XN_CFG_PRIM_RX     (1 << 0)
#define XN_CFG_PWR_UP      (1 << 1)
#define XN_CFG_CRCO        (1 << 2)   // 1 = 2-byte CRC
#define XN_CFG_EN_CRC      (1 << 3)
#define XN_CFG_MASK_MAX_RT (1 << 4)
#define XN_CFG_MASK_TX_DS  (1 << 5)
#define XN_CFG_MASK_RX_DR  (1 << 6)
// EN_PM enables the power-management block (STB1 -> STB3). Table 4-1 requires
// it set for RX and TX alike. On the nRF24L01 this bit is reserved-zero, so
// code ported from an nRF24 driver leaves it clear and the radio never keys up.
#define XN_CFG_EN_PM       (1 << 7)

// ---- STATUS bits ----
#define XN_ST_TX_FULL      (1 << 0)
#define XN_ST_MAX_RT       (1 << 4)
#define XN_ST_TX_DS        (1 << 5)
#define XN_ST_RX_DR        (1 << 6)
#define XN_ST_RX_P_NO(s)   (((s) >> 1) & 0x07)   // 0..5 = pipe, 7 = FIFO empty

// ---- RF_SETUP (0x06) --------------------------------------------------------
// RF_DR is bits 7:6, RF_PWR is bits 5:0. Both encodings are XN297L-specific:
// the power field is a 6-bit code from a fixed table, not the nRF24 0..3 scale.
#define XN_DR_1MBPS        0x00
#define XN_DR_2MBPS        0x01
#define XN_DR_250KBPS      0x03

#define XN_PWR_11DBM       0x27   // 100111 -- 66 mA, needs a solid supply
#define XN_PWR_9DBM        0x15   // 010101 -- 30 mA
#define XN_PWR_5DBM        0x2C   // 101100
#define XN_PWR_4DBM        0x14   // 010100
#define XN_PWR_MINUS1DBM   0x2A   // 101010
#define XN_PWR_MINUS10DBM  0x19   // 011001
#define XN_PWR_MINUS23DBM  0x30   // 110000

#define XN_RF_SETUP(dr, pwr)  (uint8_t)((((dr) & 0x03) << 6) | ((pwr) & 0x3F))

// RSSI enable bits -- present on the XN297, NOT documented on the XN297L.
//
// The XN297L manual documents register 0x09 (RSSI_RT 7:4, RSSI_SY 3:0) but
// marks it "Special Function Register ... declared in the software design
// reference", a document Panchip never published. The full XN297 datasheet
// does document it: RSSI is gated behind RSSI_EN, which resets to 0, which is
// why 0x09 reads back as zero out of the box.
//
// The catch is that the XN297L remapped exactly these bits -- RF_SETUP 7 became
// part of the 2-bit RF_DR field, and CONFIG 7 became EN_PM. So on true XN297L
// silicon setting RSSI_EN also changes the data rate. Whether a given module
// behaves like an XN297 or an XN297L here is a question the datasheets cannot
// answer; setRssiEnable() is how you ask the chip directly.
#define XN_RF_RSSI_EN      (1 << 7)   // XN297 RF_SETUP bit 7
#define XN_RF_RSSI_SEL     (1 << 5)   // XN297 RF_SETUP bit 5: 1 = filtered

// ---- FIFO_STATUS bits ----
#define XN_FIFO_RX_EMPTY   (1 << 0)
#define XN_FIFO_TX_EMPTY   (1 << 4)

#define XN_PAYLOAD_SIZE    32
#define XN_ADDR_WIDTH      5

// SPI is limited to 1 Mbps while the chip is in power-down or standby-I, and
// 4 Mbps once it is running (Section 7). begin() honours both.
#define XN_SPI_HZ_SLOW     1000000UL
#define XN_SPI_HZ_FAST     4000000UL

class XN297L {
public:
  XN297L(int8_t cePin, int8_t csnPin, int8_t irqPin = -1);

  // Brings up SPI + the chip. Returns true only if the SPI read-back self-test
  // passed, i.e. the module is really answering on MISO.
  bool begin(SPIClass *spi = &SPI, uint32_t runHz = XN_SPI_HZ_FAST);

  // Writes a pattern to TX_ADDR and reads it back. This is the wiring test:
  // it passes only if MOSI, MISO, CLK and CSN are all correct and the module
  // is powered.
  bool selfTest();

  void setChannel(uint8_t ch);                       // 0..83 (2400 + ch MHz)
  void setAddress(const uint8_t *addr, uint8_t len); // len must be 5 here
  void setRfSetup(uint8_t dataRate, uint8_t power);
  // Sets/clears RF_SETUP bit 7. On XN297 silicon that is RSSI_EN and register
  // 0x09 starts reporting; on a true XN297L it is the top bit of RF_DR and the
  // link will change rate or drop. Returns the RF_SETUP value read back.
  uint8_t setRssiEnable(bool on);

  // Turns on the RSSI reporting that the XN297L manual does not document, by
  // writing RSSI_GAIN_CTR in RF_CAL -- bits located by the scan env on this
  // hardware. See README.md.
  //   gain 0 = 00, off (the reset value)   1 = 01, -6 dB
  //        2 = 10, -12 dB                  3 = 11, -18 dB
  // Returns the value register 0x09 reads immediately afterwards.
  uint8_t enableRssi(uint8_t gain = 1);

  void startListening();
  void stopListening();

  // Blocking send. Returns true on TX_DS (the receiver ACKed).
  bool send(const void *buf, uint8_t len, uint32_t timeoutMs = 100);

  // Fire-and-forget send: no auto-ack, no retransmission, so it cannot report
  // whether anyone heard it. Use for data that is already covered by an ACKed
  // exchange -- telemetry riding back on a request the peer just acknowledged.
  // Returns true once the packet has left the FIFO.
  bool sendNoAck(const void *buf, uint8_t len, uint32_t timeoutMs = 100);

  bool available();
  uint8_t read(void *buf, uint8_t maxLen);           // returns bytes copied

  // Diagnostics
  uint8_t readReg(uint8_t reg);
  void    writeReg(uint8_t reg, uint8_t val);
  void    readRegMulti(uint8_t reg, uint8_t *buf, uint8_t len);
  void    writeRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len);
  uint8_t command(uint8_t cmd);
  uint8_t commandWithData(uint8_t cmd, uint8_t data);
  uint8_t status();
  uint8_t rssi() { return readReg(XN_REG_RSSI); }
  // RSSI latched by available() at the moment a packet was detected.
  // High nibble = RSSI_RT (real time), low nibble = RSSI_SY (after sync).
  uint8_t lastRssi() const { return _rxRssi; }
  uint8_t lastObserveTx() const { return _observeTx; }
  void    dumpRegisters(Stream &out);

private:
  void ceHigh() { digitalWrite(_ce, HIGH); }
  void ceLow()  { digitalWrite(_ce, LOW);  }
  void select()   { digitalWrite(_csn, LOW);  }
  void deselect() { digitalWrite(_csn, HIGH); }
  void beginTxn() { _spi->beginTransaction(_settings); select(); }
  void endTxn()   { deselect(); _spi->endTransaction(); }
  void setSpiHz(uint32_t hz);
  void reset();
  void writeCalibration();
  bool transmit(uint8_t writeCmd, const void *buf, uint8_t len,
                uint32_t timeoutMs);

  int8_t     _ce, _csn, _irq;
  SPIClass  *_spi = nullptr;
  SPISettings _settings;
  uint32_t   _runHz = XN_SPI_HZ_FAST;
  uint8_t    _baseConfig = 0;   // CONFIG with EN_PM/CRC/PWR_UP, minus PRIM_RX
  uint8_t    _addr[XN_ADDR_WIDTH] = {0};
  uint8_t    _observeTx = 0;
  uint8_t    _rxRssi = 0;
  bool       _listening = false;
};
