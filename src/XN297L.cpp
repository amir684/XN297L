#include "XN297L.h"

#include <string.h>

// ---- register bits ----------------------------------------------------------
#define XN_CFG_PRIM_RX      0x01
#define XN_CFG_PWR_UP       0x02
#define XN_CFG_CRCO         0x04   // 1 = 2-byte CRC
#define XN_CFG_EN_CRC       0x08
#define XN_CFG_MASK_MAX_RT  0x10
#define XN_CFG_MASK_TX_DS   0x20
#define XN_CFG_MASK_RX_DR   0x40
#define XN_CFG_EN_PM        0x80   // not on the nRF24L01 -- see powerUp()

#define XN_ST_TX_FULL       0x01
#define XN_ST_MAX_RT        0x10
#define XN_ST_TX_DS         0x20
#define XN_ST_RX_DR         0x40
#define XN_ST_IRQ_ALL       (XN_ST_RX_DR | XN_ST_TX_DS | XN_ST_MAX_RT)

#define XN_FEAT_EN_NOACK    0x01
#define XN_FEAT_EN_ACK_PAY  0x02
#define XN_FEAT_EN_DPL      0x04

#define XN_FIFO_RX_FULL     0x02

#define XN_SPI_SPEED_INIT   1000000UL
#define XN_WRITE_TIMEOUT_MS 200

// BB_CAL (0x1F) -- 40 bits of RF/baseband timing, assembled from the per-field
// reset values the manual lists for it:
//
//   bits 39:32  Reserved       = 0x46   (only this value allowed)
//   bit  31     INVERTER       = 1
//   bit  30     DAC_MODE       = 0
//   bits 29:24  DAC_BASAL      = 011100
//   bits 23:21  TRX_TIME       = 011     -> 31.5 us
//   bits 20:16  EX_PA_TIME     = 00111   -> 112 us
//   bits 15:11  TX_SETUP_TIME  = 01101   -> 208 us
//   bits 10:6   RX_SETUP_TIME  = 10100   -> 320 us
//   bits  5:0   RX_ACK_TIME    = 001010  -> 320 us at 1 Mbps
//
// Packed MSB first that is 46 9C 67 6D 0A; data goes over SPI LS byte first
// (Section 7.1). Writing the reset values changes nothing, but the two Enhanced
// BURST timing conditions in Section 5.6 are expressed in exactly these fields,
// so this is where to look when a link works at one data rate and not another.
static const uint8_t XN_BB_CAL_DEFAULT[5] = {0x0A, 0x6D, 0x67, 0x9C, 0x46};

#ifndef XN297L_NO_DETAILS
static void printHex(Stream &out, uint8_t v) {
  if (v < 0x10) out.print('0');
  out.print(v, HEX);
}
#endif

XN297L::XN297L(uint16_t cePin, uint16_t csnPin)
    : _cePin(cePin),
      _csnPin(csnPin),
      _spi(nullptr),
      _spiSpeed(XN297L_SPI_SPEED),
      _spiNow(XN_SPI_SPEED_INIT),
      _status(0),
      _config(0),
      _payloadSize(32),
      _addrWidth(5),
      _p0Rx(false),
      _dynamicPayloads(false),
      _ackPayloads(false),
      _ceHigh(false),
      _rssiOn(false),
      _lastRssi(0) {
  memset(_pipe0Writing, 0, sizeof(_pipe0Writing));
  memset(_pipe0Reading, 0, sizeof(_pipe0Reading));
}

// ---- low level --------------------------------------------------------------

void XN297L::beginTransaction() {
  _spi->beginTransaction(SPISettings(_spiNow, MSBFIRST, SPI_MODE0));
  digitalWrite(_csnPin, LOW);
}

void XN297L::endTransaction() {
  digitalWrite(_csnPin, HIGH);
  _spi->endTransaction();
}

void XN297L::ce(bool level) {
  digitalWrite(_cePin, level ? HIGH : LOW);
  _ceHigh = level;
}

uint8_t XN297L::command(uint8_t cmd) {
  beginTransaction();
  _status = _spi->transfer(cmd);
  endTransaction();
  return _status;
}

uint8_t XN297L::command(uint8_t cmd, uint8_t data) {
  beginTransaction();
  _status = _spi->transfer(cmd);
  _spi->transfer(data);
  endTransaction();
  return _status;
}

uint8_t XN297L::readRegister(uint8_t reg) {
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_R_REGISTER | (reg & 0x1F));
  uint8_t value = _spi->transfer(XN297L_CMD_NOP);
  endTransaction();
  return value;
}

void XN297L::readRegister(uint8_t reg, uint8_t *buf, uint8_t len) {
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_R_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) buf[i] = _spi->transfer(XN297L_CMD_NOP);
  endTransaction();
}

void XN297L::writeRegister(uint8_t reg, uint8_t value) {
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_W_REGISTER | (reg & 0x1F));
  _spi->transfer(value);
  endTransaction();
}

void XN297L::writeRegister(uint8_t reg, const uint8_t *buf, uint8_t len) {
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_W_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) _spi->transfer(buf[i]);
  endTransaction();
}

uint8_t XN297L::getStatus() { return command(XN297L_CMD_NOP); }

// ---- setup ------------------------------------------------------------------

bool XN297L::begin(XN297L_SPIClass *spi, uint32_t spiSpeed) {
  _spi = spi;
  _spiSpeed = spiSpeed;

  // RF24's begin() starts the bus itself, so ported sketches have no
  // SPI.begin() of their own. A second begin() is harmless on every core --
  // and on ESP32 it is a no-op, so custom pins set up beforehand survive.
  _spi->begin();

  // Everything until powerUp() happens in power-down or standby-I, where the
  // chip only takes SPI at up to 1 Mbps (Section 7).
  _spiNow = XN_SPI_SPEED_INIT;

  pinMode(_cePin, OUTPUT);
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  ce(LOW);

  delay(20);                                  // supply settle + 10 ms POR

  // Reset is a command WITH a data byte: 0x5A holds the chip in reset, 0xA5
  // releases it (Table 7-2). A bare 0x53, as an nRF24-style opcode, leaves it
  // latched in reset.
  command(XN297L_CMD_RST_FSPI, 0x5A);
  delay(1);
  command(XN297L_CMD_RST_FSPI, 0xA5);
  delay(10);

  // Configure while powered down -- W_REGISTER is only legal in power-down or
  // standby (Table 7-2).
  _config = XN_CFG_EN_CRC | XN_CFG_CRCO;
  writeRegister(XN297L_REG_CONFIG, _config);
  delay(5);

  writeRegister(XN297L_REG_BB_CAL, XN_BB_CAL_DEFAULT, sizeof(XN_BB_CAL_DEFAULT));

  writeRegister(XN297L_REG_EN_AA, 0x3F);      // auto-ack on every pipe
  writeRegister(XN297L_REG_EN_RXADDR, 0x01);  // pipe 0, which carries ACKs
  setAddressWidth(5);
  setRetries(2, 15);                          // 750 us, 15 retries
  setChannel(78);                             // clear of the crystal harmonics
  writeRegister(XN297L_REG_RF_SETUP, (XN297L_1MBPS << 6) | XN297L_PA_9DBM);

  // ACTIVATE unlocks R_RX_PL_WID, W_ACK_PAYLOAD and W_TX_PAYLOAD_NOACK. It is
  // only accepted in power-down or standby, which is where the chip is now.
  command(XN297L_CMD_ACTIVATE, 0x73);
  writeRegister(XN297L_REG_DYNPD, 0x00);
  // FEATURE bits 6:3 have no nRF24 equivalent. Zero keeps the IRQ pin as IRQ,
  // CE on the pin rather than SPI, and 32-byte FIFOs. NOACK writes are allowed
  // from the start, so write(buf, len, true) works without enableDynamicAck().
  writeRegister(XN297L_REG_FEATURE, XN_FEAT_EN_NOACK);
  _dynamicPayloads = false;
  _ackPayloads = false;

  setPayloadSize(32);

  flush_rx();
  flush_tx();
  writeRegister(XN297L_REG_STATUS, XN_ST_IRQ_ALL);

  bool connected = isChipConnected();

  powerUp();
  return connected;
}

bool XN297L::isChipConnected() {
  // TX_ADDR is a full-width register the chip echoes back verbatim. A second,
  // different pattern rules out a MISO line that happens to mirror whatever
  // was just clocked out.
  static const uint8_t a[5] = {0xC2, 0xA5, 0x5A, 0x3C, 0xE7};
  static const uint8_t b[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
  uint8_t saved[5], back[5];

  // Register writes are not accepted in RX, so step out of it if listening.
  bool wasListening = _ceHigh;
  if (wasListening) {
    ce(LOW);
    delayMicroseconds(100);
  }

  readRegister(XN297L_REG_TX_ADDR, saved, 5);

  writeRegister(XN297L_REG_TX_ADDR, a, 5);
  readRegister(XN297L_REG_TX_ADDR, back, 5);
  bool ok = memcmp(a, back, 5) == 0;

  if (ok) {
    writeRegister(XN297L_REG_TX_ADDR, b, 5);
    readRegister(XN297L_REG_TX_ADDR, back, 5);
    ok = memcmp(b, back, 5) == 0;
  }

  writeRegister(XN297L_REG_TX_ADDR, saved, 5);

  if (wasListening) {
    ce(HIGH);
    delayMicroseconds(400);
  }
  return ok;
}

void XN297L::powerUp() {
  if (_config & XN_CFG_PWR_UP) return;

  // PWR_UP alone only reaches standby-I. It takes EN_PM on top to reach
  // standby-III, and TX and RX are reachable only from there (Table 4-1).
  //
  // This is the difference that matters most. EN_PM is reserved-zero on the
  // nRF24L01, so a driver ported from one leaves it clear -- and then SPI
  // works, every register reads back correctly, and the radio never transmits
  // or receives a single packet.
  _config |= XN_CFG_PWR_UP;
  writeRegister(XN297L_REG_CONFIG, _config);
  delay(10);                                  // power-down -> standby-I

  _config |= XN_CFG_EN_PM;
  writeRegister(XN297L_REG_CONFIG, _config);
  delayMicroseconds(100);                     // standby-I -> standby-III

  _spiNow = _spiSpeed;                        // the 1 Mbps SPI cap is lifted
}

void XN297L::powerDown() {
  ce(LOW);
  _config &= (uint8_t)~(XN_CFG_PWR_UP | XN_CFG_EN_PM);
  writeRegister(XN297L_REG_CONFIG, _config);
  _spiNow = XN_SPI_SPEED_INIT;                // back under the power-down cap
}

// ---- receive ----------------------------------------------------------------

void XN297L::startListening() {
  powerUp();

  _config |= XN_CFG_PRIM_RX;
  writeRegister(XN297L_REG_CONFIG, _config);
  writeRegister(XN297L_REG_STATUS, XN_ST_IRQ_ALL);

  // RF24 semantics: openWritingPipe() borrowed pipe 0 for ACKs, so restore the
  // reading address if the application opened pipe 0 for reading, and close
  // pipe 0 otherwise. Done before CE goes high -- register writes are not
  // accepted in RX.
  if (_p0Rx) {
    writeRegister(XN297L_REG_RX_ADDR_P0, _pipe0Reading, _addrWidth);
  } else {
    closeReadingPipe(0);
  }

  ce(HIGH);
  delayMicroseconds(400);                     // standby-III -> RX: 320 us
}

void XN297L::stopListening() {
  ce(LOW);
  delayMicroseconds(100);                     // RX -> standby-III: 10 us

  if (_ackPayloads) flush_tx();

  _config &= (uint8_t)~XN_CFG_PRIM_RX;
  writeRegister(XN297L_REG_CONFIG, _config);

  // Pipe 0 receives the ACKs for whatever goes to TX_ADDR.
  writeRegister(XN297L_REG_RX_ADDR_P0, _pipe0Writing, _addrWidth);
  writeRegister(XN297L_REG_EN_RXADDR, readRegister(XN297L_REG_EN_RXADDR) | 0x01);
}

bool XN297L::available() { return available(nullptr); }

bool XN297L::available(uint8_t *pipeNum) {
  // STATUS.RX_P_NO reads 111 when the RX FIFO is empty. Used rather than
  // FIFO_STATUS, whose bit 0 and 1 names and descriptions the manual has
  // crossed over.
  uint8_t pipe = (getStatus() >> 1) & 0x07;
  if (pipe > 5) return false;
  if (pipeNum) *pipeNum = pipe;

  // RSSI_SY is captured at packet sync. This is the earliest point software
  // can read it, and later reads -- after R_RX_PAYLOAD, or once CE drops --
  // are not guaranteed to still hold it.
  if (_rssiOn) _lastRssi = readRegister(XN297L_REG_RSSI);
  return true;
}

void XN297L::read(void *buf, uint8_t len) {
  uint8_t width = _dynamicPayloads ? getDynamicPayloadSize() : _payloadSize;
  if (width == 0) return;

  // Clock out the whole payload even if the caller wants less of it: the chip
  // drops the entry from the FIFO once it has been read.
  uint8_t *p = (uint8_t *)buf;
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_R_RX_PAYLOAD);
  for (uint8_t i = 0; i < width; i++) {
    uint8_t b = _spi->transfer(XN297L_CMD_NOP);
    if (i < len) p[i] = b;
  }
  endTransaction();

  writeRegister(XN297L_REG_STATUS, XN_ST_RX_DR);
}

// ---- transmit ---------------------------------------------------------------

bool XN297L::write(const void *buf, uint8_t len) { return write(buf, len, false); }

bool XN297L::write(const void *buf, uint8_t len, bool multicast) {
  if (_config & XN_CFG_PRIM_RX) stopListening();

  flush_tx();
  writeRegister(XN297L_REG_STATUS, XN_ST_IRQ_ALL);

  // A static payload's length is set by how many bytes are clocked in, and has
  // to equal the receiver's RX_PW (Section 6.4.4) -- so pad or truncate to it.
  uint8_t width;
  if (_dynamicPayloads) {
    width = len > 32 ? 32 : len;
    if (width == 0) width = 1;
  } else {
    width = _payloadSize;
  }

  const uint8_t *p = (const uint8_t *)buf;
  beginTransaction();
  _status = _spi->transfer(multicast ? XN297L_CMD_W_TX_PAYLOAD_NOACK
                                     : XN297L_CMD_W_TX_PAYLOAD);
  for (uint8_t i = 0; i < width; i++) _spi->transfer(i < len ? p[i] : 0x00);
  endTransaction();

  // CE has to stay high for more than 30 us for the transmission to take
  // effect (Section 5.3). It is held through the ACK wait too; once the FIFO
  // drains the chip simply parks in standby-II.
  ce(HIGH);
  delayMicroseconds(40);

  uint8_t st;
  uint32_t start = millis();
  do {
    st = getStatus();
    if (st & (XN_ST_TX_DS | XN_ST_MAX_RT)) break;
  } while (millis() - start < XN_WRITE_TIMEOUT_MS);
  ce(LOW);

  writeRegister(XN297L_REG_STATUS, XN_ST_IRQ_ALL);

  // On MAX_RT the payload stays in the FIFO on purpose (Section 5.8). One
  // packet at a time is the contract here, so drop it.
  if (!(st & XN_ST_TX_DS)) {
    flush_tx();
    return false;
  }
  return true;
}

// ---- addressing -------------------------------------------------------------

void XN297L::openWritingPipe(const uint8_t *address) {
  // Pipe 0 receives the ACKs for whatever goes to TX_ADDR, so it has to carry
  // the same address (Section 5.2).
  writeRegister(XN297L_REG_RX_ADDR_P0, address, _addrWidth);
  writeRegister(XN297L_REG_TX_ADDR, address, _addrWidth);
  memcpy(_pipe0Writing, address, _addrWidth);
}

void XN297L::openReadingPipe(uint8_t pipe, const uint8_t *address) {
  if (pipe > 5) return;

  if (pipe == 0) {
    memcpy(_pipe0Reading, address, _addrWidth);
    _p0Rx = true;
  }

  if (pipe > 1) {
    // Pipes 2-5 hold only their least significant byte; the rest is shared
    // with pipe 1 (Table 5-3).
    writeRegister(XN297L_REG_RX_ADDR_P0 + pipe, address, 1);
  } else if (pipe == 1 || (_config & XN_CFG_PRIM_RX)) {
    // Pipe 0 is only written now if listening. Otherwise it is on loan to
    // openWritingPipe(), and startListening() restores it.
    writeRegister(XN297L_REG_RX_ADDR_P0 + pipe, address, _addrWidth);
  }

  writeRegister(XN297L_REG_RX_PW_P0 + pipe, _payloadSize);
  writeRegister(XN297L_REG_EN_RXADDR,
                readRegister(XN297L_REG_EN_RXADDR) | (uint8_t)(1 << pipe));
}

void XN297L::closeReadingPipe(uint8_t pipe) {
  if (pipe > 5) return;
  writeRegister(XN297L_REG_EN_RXADDR,
                readRegister(XN297L_REG_EN_RXADDR) & (uint8_t)~(1 << pipe));
  if (pipe == 0) _p0Rx = false;
}

void XN297L::setAddressWidth(uint8_t width) {
  if (width < 3) width = 3;
  if (width > 5) width = 5;
  writeRegister(XN297L_REG_SETUP_AW, width - 2);
  _addrWidth = width;
}

// ---- link settings ----------------------------------------------------------

void XN297L::setRetries(uint8_t delay, uint8_t count) {
  if (delay > 15) delay = 15;
  if (count > 15) count = 15;
  writeRegister(XN297L_REG_SETUP_RETR, (uint8_t)(delay << 4) | count);
}

void XN297L::setChannel(uint8_t channel) {
  // The XN297L band is 2400-2483 MHz, channels 0-83. Channels on a multiple of
  // the 16 MHz crystal -- 0, 16, 32, 48, 64, 80 -- lose about 2 dB of
  // sensitivity (Section 3.3, note 1).
  if (channel > 125) channel = 125;
  writeRegister(XN297L_REG_RF_CH, channel);
}

uint8_t XN297L::getChannel() { return readRegister(XN297L_REG_RF_CH); }

void XN297L::setPayloadSize(uint8_t size) {
  if (size < 1) size = 1;
  if (size > 32) size = 32;
  _payloadSize = size;
  for (uint8_t i = 0; i < 6; i++) writeRegister(XN297L_REG_RX_PW_P0 + i, size);
}

uint8_t XN297L::getPayloadSize() { return _payloadSize; }

void XN297L::enableDynamicPayloads() {
  // DPL on a pipe also needs auto-ack on that pipe, which begin() enables.
  writeRegister(XN297L_REG_FEATURE,
                readRegister(XN297L_REG_FEATURE) | XN_FEAT_EN_DPL);
  writeRegister(XN297L_REG_DYNPD, 0x3F);
  _dynamicPayloads = true;
}

void XN297L::disableDynamicPayloads() {
  writeRegister(XN297L_REG_FEATURE,
                readRegister(XN297L_REG_FEATURE) &
                    (uint8_t)~(XN_FEAT_EN_DPL | XN_FEAT_EN_ACK_PAY));
  writeRegister(XN297L_REG_DYNPD, 0x00);
  _dynamicPayloads = false;
  _ackPayloads = false;
}

uint8_t XN297L::getDynamicPayloadSize() {
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_R_RX_PL_WID);
  uint8_t width = _spi->transfer(XN297L_CMD_NOP);
  endTransaction();

  // A width over 32 means a corrupt packet made it into the FIFO. Flush it,
  // as the nRF24 datasheet prescribes, rather than hand back garbage.
  if (width > 32) {
    flush_rx();
    return 0;
  }
  return width;
}

void XN297L::enableAckPayload() {
  if (_ackPayloads) return;
  // ACK payloads ride in dynamic-length packets, so this switches those on for
  // pipes 0 and 1 as well.
  writeRegister(XN297L_REG_FEATURE, readRegister(XN297L_REG_FEATURE) |
                                        XN_FEAT_EN_ACK_PAY | XN_FEAT_EN_DPL);
  writeRegister(XN297L_REG_DYNPD, readRegister(XN297L_REG_DYNPD) | 0x03);
  _dynamicPayloads = true;
  _ackPayloads = true;
}

bool XN297L::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len) {
  if (!_ackPayloads || pipe > 5) return false;
  if (len > 32) len = 32;

  // At most two ACK payloads can be pending at once.
  const uint8_t *p = (const uint8_t *)buf;
  beginTransaction();
  _status = _spi->transfer(XN297L_CMD_W_ACK_PAYLOAD | pipe);
  for (uint8_t i = 0; i < len; i++) _spi->transfer(p[i]);
  endTransaction();

  return !(_status & XN_ST_TX_FULL);
}

void XN297L::enableDynamicAck() {
  writeRegister(XN297L_REG_FEATURE,
                readRegister(XN297L_REG_FEATURE) | XN_FEAT_EN_NOACK);
}

void XN297L::setAutoAck(bool enable) {
  writeRegister(XN297L_REG_EN_AA, enable ? 0x3F : 0x00);
}

void XN297L::setAutoAck(uint8_t pipe, bool enable) {
  if (pipe > 5) return;
  uint8_t aa = readRegister(XN297L_REG_EN_AA);
  if (enable) aa |= (uint8_t)(1 << pipe);
  else        aa &= (uint8_t)~(1 << pipe);
  writeRegister(XN297L_REG_EN_AA, aa);
}

void XN297L::setPALevel(xn297l_pa_dbm_e level) {
  uint8_t v = readRegister(XN297L_REG_RF_SETUP);
  writeRegister(XN297L_REG_RF_SETUP, (v & 0xC0) | ((uint8_t)level & 0x3F));
}

xn297l_pa_dbm_e XN297L::getPALevel() {
  return (xn297l_pa_dbm_e)(readRegister(XN297L_REG_RF_SETUP) & 0x3F);
}

bool XN297L::setDataRate(xn297l_datarate_e rate) {
  // RF_DR is the top two bits, with its own encoding: 00 = 1 Mbps,
  // 01 = 2 Mbps, 11 = 250 kbps, and 10 is reserved. Setting that reserved
  // combination leaves data crossing but breaks auto-ack timing.
  uint8_t v = readRegister(XN297L_REG_RF_SETUP);
  v = (v & 0x3F) | (uint8_t)(((uint8_t)rate & 0x03) << 6);
  writeRegister(XN297L_REG_RF_SETUP, v);
  return readRegister(XN297L_REG_RF_SETUP) == v;
}

xn297l_datarate_e XN297L::getDataRate() {
  return (xn297l_datarate_e)((readRegister(XN297L_REG_RF_SETUP) >> 6) & 0x03);
}

void XN297L::setCRCLength(xn297l_crclength_e length) {
  _config &= (uint8_t)~(XN_CFG_EN_CRC | XN_CFG_CRCO);
  if (length == XN297L_CRC_8)       _config |= XN_CFG_EN_CRC;
  else if (length == XN297L_CRC_16) _config |= XN_CFG_EN_CRC | XN_CFG_CRCO;
  writeRegister(XN297L_REG_CONFIG, _config);
}

xn297l_crclength_e XN297L::getCRCLength() {
  // EN_CRC is forced on whenever any pipe has auto-ack, whatever CONFIG says.
  uint8_t config = readRegister(XN297L_REG_CONFIG);
  if ((config & XN_CFG_EN_CRC) || readRegister(XN297L_REG_EN_AA)) {
    return (config & XN_CFG_CRCO) ? XN297L_CRC_16 : XN297L_CRC_8;
  }
  return XN297L_CRC_DISABLED;
}

void XN297L::disableCRC() { setCRCLength(XN297L_CRC_DISABLED); }

// ---- status -----------------------------------------------------------------

uint8_t XN297L::getARC() { return readRegister(XN297L_REG_OBSERVE_TX) & 0x0F; }

bool XN297L::rxFifoFull() {
  // Bit 1, going by the reset values -- the manual's labels for bits 0 and 1
  // are swapped.
  return readRegister(XN297L_REG_FIFO_STATUS) & XN_FIFO_RX_FULL;
}

void XN297L::flush_rx() { command(XN297L_CMD_FLUSH_RX); }

void XN297L::flush_tx() { command(XN297L_CMD_FLUSH_TX); }

void XN297L::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready) {
  _config &= (uint8_t)~(XN_CFG_MASK_TX_DS | XN_CFG_MASK_MAX_RT | XN_CFG_MASK_RX_DR);
  if (tx_ok)    _config |= XN_CFG_MASK_TX_DS;
  if (tx_fail)  _config |= XN_CFG_MASK_MAX_RT;
  if (rx_ready) _config |= XN_CFG_MASK_RX_DR;
  writeRegister(XN297L_REG_CONFIG, _config);
}

void XN297L::whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready) {
  uint8_t st = getStatus();
  writeRegister(XN297L_REG_STATUS, XN_ST_IRQ_ALL);
  tx_ok    = st & XN_ST_TX_DS;
  tx_fail  = st & XN_ST_MAX_RT;
  rx_ready = st & XN_ST_RX_DR;
}

// ---- RSSI -------------------------------------------------------------------
//
// The XN297L manual documents register 0x09 -- RSSI_RT in bits 7:4, RSSI_SY in
// bits 3:0 -- as a "special function register declared in the software design
// reference", which Panchip never published. Out of the box it reads zero.
//
// The full XN297 (non-L) datasheet does document RSSI, behind RF_SETUP.RSSI_EN
// and CONFIG.DATAOUT_SEL. Both bits were reused on the XN297L -- as RF_DR and
// EN_PM -- which an experiment confirmed: setting RF_SETUP bit 7 moved the data
// rate instead of enabling anything.
//
// The enable was found by sweeping single-bit flips of the undocumented
// registers while counting valid packets against junk, so that "the chip now
// decodes noise" could not pass for success. Exactly two bits move 0x09 with
// reception intact: RF_CAL (0x1E) bits 15 and 16.
//
// They land precisely where RSSI_GAIN_CTR sits in the XN297 field map -- its
// RF_CAL bits 48:47, a 0/-6/-12/-18 dB attenuator -- if the XN297L's 24-bit
// RF_CAL is the top 24 bits of the XN297's 56-bit one. That mapping predicted
// both positions before the sweep ran. Independently, Panchip's own reference
// initialisation writes RF_CAL = F6 3F 5D, which sets bit 16: the same field.
//
// There is no separate enable. RSSI_GAIN_CTR = 00, the reset value, simply
// means no measurement.
//
// The cost: the attenuation is not confined to the detector. With it on at both
// ends of a link, retries climbed and unacknowledged packets stopped arriving,
// which is why enableRSSI() and disableRSSI() are meant to be toggled around
// the packets being measured.

void XN297L::setRSSIControl(uint8_t code) {
  // Register writes are not accepted in RX (Table 7-2), so step out of it for
  // the write if listening, and back in afterwards.
  bool wasListening = _ceHigh;
  if (wasListening) {
    ce(LOW);
    delayMicroseconds(100);
  }

  uint8_t rf[3];
  readRegister(XN297L_REG_RF_CAL, rf, 3);
  rf[1] = (uint8_t)((rf[1] & 0x7F) | ((code & 0x01) << 7));   // bit 15
  rf[2] = (uint8_t)((rf[2] & 0xFE) | ((code >> 1) & 0x01));   // bit 16
  writeRegister(XN297L_REG_RF_CAL, rf, 3);
  _rssiOn = (code & 0x03) != 0;

  if (wasListening) {
    ce(HIGH);
    delayMicroseconds(400);
  }
}

void XN297L::enableRSSI(xn297l_rssi_atten_e attenuation) {
  setRSSIControl((uint8_t)attenuation);
}

void XN297L::disableRSSI() { setRSSIControl(XN297L_RSSI_OFF); }

uint8_t XN297L::getRSSI() { return _lastRssi & 0x0F; }

// ---- diagnostics ------------------------------------------------------------
// Compiled out by -DXN297L_NO_DETAILS, for parts where the register names are
// a meaningful share of the flash.
#ifndef XN297L_NO_DETAILS


void XN297L::printRow(Stream &out, const __FlashStringHelper *name, uint8_t reg,
                      uint8_t len) {
  uint8_t buf[6];
  readRegister(reg, buf, len);
  out.print(F("  0x"));
  printHex(out, reg);
  out.print(' ');
  out.print(name);
  out.print(F(" ="));
  for (uint8_t i = 0; i < len; i++) {
    out.print(' ');
    printHex(out, buf[i]);
  }
  out.println();
}

void XN297L::printDetails(Stream &out) {
  out.println(F("--- XN297L ---"));
  printRow(out, F("CONFIG     "), XN297L_REG_CONFIG, 1);
  printRow(out, F("EN_AA      "), XN297L_REG_EN_AA, 1);
  printRow(out, F("EN_RXADDR  "), XN297L_REG_EN_RXADDR, 1);
  printRow(out, F("SETUP_AW   "), XN297L_REG_SETUP_AW, 1);
  printRow(out, F("SETUP_RETR "), XN297L_REG_SETUP_RETR, 1);
  printRow(out, F("RF_CH      "), XN297L_REG_RF_CH, 1);
  printRow(out, F("RF_SETUP   "), XN297L_REG_RF_SETUP, 1);
  printRow(out, F("STATUS     "), XN297L_REG_STATUS, 1);
  printRow(out, F("OBSERVE_TX "), XN297L_REG_OBSERVE_TX, 1);
  printRow(out, F("RSSI       "), XN297L_REG_RSSI, 1);
  printRow(out, F("RX_ADDR_P0 "), XN297L_REG_RX_ADDR_P0, 5);
  printRow(out, F("RX_ADDR_P1 "), XN297L_REG_RX_ADDR_P0 + 1, 5);

  out.print(F("  0x0C RX_ADDR_P2-5 ="));
  for (uint8_t p = 2; p < 6; p++) {
    out.print(' ');
    printHex(out, readRegister(XN297L_REG_RX_ADDR_P0 + p));
  }
  out.println();

  printRow(out, F("TX_ADDR    "), XN297L_REG_TX_ADDR, 5);

  out.print(F("  0x11 RX_PW_P0-5 ="));
  for (uint8_t p = 0; p < 6; p++) {
    out.print(' ');
    printHex(out, readRegister(XN297L_REG_RX_PW_P0 + p));
  }
  out.println();

  printRow(out, F("FIFO_STATUS"), XN297L_REG_FIFO_STATUS, 1);
  printRow(out, F("DYNPD      "), XN297L_REG_DYNPD, 1);
  printRow(out, F("FEATURE    "), XN297L_REG_FEATURE, 1);
  printRow(out, F("DEMOD_CAL  "), XN297L_REG_DEMOD_CAL, 1);
  printRow(out, F("RF_CAL2    "), XN297L_REG_RF_CAL2, 6);
  printRow(out, F("DEM_CAL2   "), XN297L_REG_DEM_CAL2, 3);
  printRow(out, F("RF_CAL     "), XN297L_REG_RF_CAL, 3);
  printRow(out, F("BB_CAL     "), XN297L_REG_BB_CAL, 5);

  static const char *const rates[] = {"1 Mbps", "2 Mbps", "reserved", "250 kbps"};
  out.print(F("  channel "));
  out.print(getChannel());
  out.print(F(", "));
  out.print(rates[getDataRate()]);
  out.print(F(", payload "));
  if (_dynamicPayloads) out.print(F("dynamic"));
  else                  out.print(_payloadSize);
  out.println();
  out.println(F("--------------"));
}

#endif  // XN297L_NO_DETAILS
