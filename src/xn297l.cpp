#include "xn297l.h"

// -----------------------------------------------------------------------------
// BB_CAL (0x1F) -- 40 bits of RF/baseband timing, assembled here from the
// per-field reset values the datasheet lists for register 0x1F:
//
//   bits 39:32  Reserved       = 0x46   (only this value allowed)
//   bit  31     INVERTER       = 1
//   bit  30     DAC_MODE       = 0
//   bits 29:24  DAC_BASAL      = 011100
//   bits 23:21  TRX_TIME       = 011     -> 3*8 + 7.5 = 31.5 us
//   bits 20:16  EX_PA_TIME     = 00111   -> 7*16      = 112 us
//   bits 15:11  TX_SETUP_TIME  = 01101   -> 13*16     = 208 us
//   bits 10:6   RX_SETUP_TIME  = 10100   -> 20*16     = 320 us
//   bits  5:0   RX_ACK_TIME    = 001010  -> 10*32     = 320 us at 1 Mbps
//
// Packed MSB-first that is 46 9C 67 6D 0A; SPI sends data LS byte first
// (Section 7.1), hence the order below.
//
// These are the power-on defaults, so writing them changes nothing today. The
// block is here because the two Enhanced BURST timing conditions in Section 5.6
// are expressed in exactly these fields -- this is where you come when a link
// works at 1 Mbps but not at 250 kbps.
// -----------------------------------------------------------------------------
static const uint8_t XN_BB_CAL_DATA[5] = {0x0A, 0x6D, 0x67, 0x9C, 0x46};

// DEMOD_CAL (0x19, 1 byte), RF_CAL (0x1E, 3 bytes), RF_CAL2 (0x1A, 6 bytes) and
// DEM_CAL2 (0x1B, 3 bytes) are listed only as "Special Function Register" -- the
// manual gives no values and defers to a software reference we do not have. We
// leave them at their power-on defaults rather than write guesses into the
// analog front end.

XN297L::XN297L(int8_t cePin, int8_t csnPin, int8_t irqPin)
    : _ce(cePin), _csn(csnPin), _irq(irqPin),
      _settings(XN_SPI_HZ_SLOW, MSBFIRST, SPI_MODE0) {}

// ---- low level --------------------------------------------------------------

void XN297L::setSpiHz(uint32_t hz) {
  _settings = SPISettings(hz, MSBFIRST, SPI_MODE0);
}

uint8_t XN297L::command(uint8_t cmd) {
  beginTxn();
  uint8_t st = _spi->transfer(cmd);
  endTxn();
  return st;
}

uint8_t XN297L::commandWithData(uint8_t cmd, uint8_t data) {
  beginTxn();
  uint8_t st = _spi->transfer(cmd);
  _spi->transfer(data);
  endTxn();
  return st;
}

uint8_t XN297L::readReg(uint8_t reg) {
  beginTxn();
  _spi->transfer(XN_CMD_R_REGISTER | (reg & 0x1F));
  uint8_t v = _spi->transfer(XN_CMD_NOP);
  endTxn();
  return v;
}

void XN297L::writeReg(uint8_t reg, uint8_t val) {
  beginTxn();
  _spi->transfer(XN_CMD_W_REGISTER | (reg & 0x1F));
  _spi->transfer(val);
  endTxn();
}

void XN297L::readRegMulti(uint8_t reg, uint8_t *buf, uint8_t len) {
  beginTxn();
  _spi->transfer(XN_CMD_R_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) buf[i] = _spi->transfer(XN_CMD_NOP);
  endTxn();
}

void XN297L::writeRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len) {
  beginTxn();
  _spi->transfer(XN_CMD_W_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) _spi->transfer(buf[i]);
  endTxn();
}

uint8_t XN297L::status() { return command(XN_CMD_NOP); }

// ---- setup ------------------------------------------------------------------

void XN297L::reset() {
  // Table 7-2: 0x53 + 0x5A holds the chip in reset, 0x53 + 0xA5 releases it.
  // The data byte is not optional -- a bare 0x53 leaves it latched down.
  commandWithData(XN_CMD_RST_FSPI, XN_RST_HOLD);
  delay(1);
  commandWithData(XN_CMD_RST_FSPI, XN_RST_RELEASE);
  delay(10);                 // POR: 10 ms (Figure 4-1)
}

void XN297L::writeCalibration() {
  writeRegMulti(XN_REG_BB_CAL, XN_BB_CAL_DATA, sizeof(XN_BB_CAL_DATA));
}

bool XN297L::begin(SPIClass *spi, uint32_t runHz) {
  _spi = spi;
  _runHz = runHz;

  pinMode(_csn, OUTPUT);
  pinMode(_ce, OUTPUT);
  deselect();
  ceLow();
  if (_irq >= 0) pinMode(_irq, INPUT_PULLUP);

  // Everything up to the STB3 transition below happens in power-down or
  // standby-I, where the SPI ceiling is 1 Mbps (Section 7).
  setSpiHz(XN_SPI_HZ_SLOW);

  delay(20);                 // supply settle + POR
  reset();

  // Configure while powered down. W_REGISTER is only legal in power-down or
  // standby anyway (Table 7-2).
  writeReg(XN_REG_CONFIG, XN_CFG_EN_CRC | XN_CFG_CRCO);
  delay(5);

  writeCalibration();

  writeReg(XN_REG_EN_AA,      0x01);   // auto-ack on pipe 0 only
  writeReg(XN_REG_EN_RXADDR,  0x01);   // enable pipe 0 only
  writeReg(XN_REG_SETUP_AW,   0x03);   // 5-byte address
  writeReg(XN_REG_SETUP_RETR, 0x2F);   // ARD 750us, ARC 15 retries
  writeReg(XN_REG_RF_CH,      78);     // 2478 MHz
  writeReg(XN_REG_RF_SETUP,   XN_RF_SETUP(XN_DR_1MBPS, XN_PWR_11DBM));
  writeReg(XN_REG_DYNPD,      0x00);   // fixed payload length

  // ACTIVATE (Table 7-2) unlocks R_RX_PL_WID, W_ACK_PAYLOAD and
  // W_TX_PAYLOAD_NOACK. Legal only in power-down or standby, which is where we
  // are. Without it, sendNoAck() writes a command the chip ignores.
  commandWithData(XN_CMD_ACTIVATE, 0x73);
  // FEATURE: IRQ pin carries IRQ (not EN_PA), CE from the pin (not SPI),
  // 32-byte FIFOs, no DPL, no ACK payload, W_TX_PAYLOAD_NOACK enabled.
  writeReg(XN_REG_FEATURE,    0x01);   // EN_NOACK
  writeReg(XN_REG_RX_PW_P0,   XN_PAYLOAD_SIZE);

  command(XN_CMD_FLUSH_TX);
  command(XN_CMD_FLUSH_RX);
  writeReg(XN_REG_STATUS, XN_ST_RX_DR | XN_ST_TX_DS | XN_ST_MAX_RT);

  if (!selfTest()) return false;

  // Power-down -> standby-I -> standby-III. EN_PM is what gets us to STB3, and
  // TX and RX are both reachable only from there (Table 4-1).
  _baseConfig = XN_CFG_EN_PM | XN_CFG_EN_CRC | XN_CFG_CRCO | XN_CFG_PWR_UP;

  writeReg(XN_REG_CONFIG, XN_CFG_EN_CRC | XN_CFG_CRCO | XN_CFG_PWR_UP);
  delay(10);                 // PWR_DN -> STB1: 10 ms
  writeReg(XN_REG_CONFIG, _baseConfig);
  delayMicroseconds(100);    // STB1 -> STB3: 50 us

  setSpiHz(_runHz);          // out of standby-I, the 1 Mbps cap is lifted
  return true;
}

bool XN297L::selfTest() {
  // TX_ADDR is a 5-byte scratch register we own, so round-tripping a pattern
  // through it exercises MOSI, MISO, CLK and CSN in both directions.
  const uint8_t pattern[XN_ADDR_WIDTH] = {0xC2, 0xA5, 0x5A, 0x3C, 0xE7};
  uint8_t back[XN_ADDR_WIDTH] = {0};

  writeRegMulti(XN_REG_TX_ADDR, pattern, XN_ADDR_WIDTH);
  readRegMulti(XN_REG_TX_ADDR, back, XN_ADDR_WIDTH);
  if (memcmp(pattern, back, XN_ADDR_WIDTH) != 0) return false;

  // A second, different pattern rules out a stuck MISO line that happens to
  // mirror whatever we clocked out.
  const uint8_t pattern2[XN_ADDR_WIDTH] = {0x11, 0x22, 0x33, 0x44, 0x55};
  writeRegMulti(XN_REG_TX_ADDR, pattern2, XN_ADDR_WIDTH);
  readRegMulti(XN_REG_TX_ADDR, back, XN_ADDR_WIDTH);
  return memcmp(pattern2, back, XN_ADDR_WIDTH) == 0;
}

void XN297L::setChannel(uint8_t ch) {
  // Section 3.3 note 1: multiples of 16 MHz (channel 0, 16, 32, 48, 64, 80)
  // cost about 2 dB of sensitivity. 78 is the default here and is clear of them.
  writeReg(XN_REG_RF_CH, ch & 0x7F);
}

void XN297L::setRfSetup(uint8_t dataRate, uint8_t power) {
  writeReg(XN_REG_RF_SETUP, XN_RF_SETUP(dataRate, power));
}

// -----------------------------------------------------------------------------
// Undocumented RSSI enable.
//
// The XN297L manual documents register 0x09 but marks it a "Special Function
// Register" and refers to a software reference Panchip never published. The
// XN297 (non-L) full datasheet does document the mechanism -- RF_SETUP.RSSI_EN
// plus CONFIG.DATAOUT_SEL -- but the XN297L reused both of those bits for
// RF_DR and EN_PM, which an experiment on this hardware confirmed.
//
// So it was located empirically instead, by the scan env: it flips one bit at a
// time in the undocumented registers and watches 0x09, counting valid packets
// against junk so that "the chip now accepts noise" cannot masquerade as
// success. Exactly two bits move 0x09 while packet reception stays clean:
//
//   0x1E bit 15  ->  0x09 = 0x66     (good packets, zero junk)
//   0x1E bit 16  ->  0x09 = 0x06     (good packets, zero junk)
//
// Those two are adjacent, and they land exactly where RSSI_GAIN_CTR sits in the
// XN297 field map -- its RF_CAL bits 48:47, a -0/-6/-12/-18 dB attenuator on
// the RSSI path -- if the XN297L's 24-bit 0x1E is the top 24 bits of the
// XN297's 56-bit RF_CAL. The mapping is inferred rather than documented, but it
// predicted both hit locations before the sweep ran, and the readings behave
// like an attenuator: RSSI_RT drops from 6 to 0 as the field goes from 01
// (-6 dB) to 10 (-12 dB).
//
// That also explains the zero we started from. There is probably no separate
// enable on the XN297L: RSSI_GAIN_CTR = 00, the reset value, simply means no
// measurement, and any non-zero setting switches the detector on.
//
// An earlier sweep also flagged 0x1B bit 21, and this driver briefly set it.
// It was an artifact -- that bit breaks packet validation, after which the chip
// decodes noise and 0x09 fills with garbage. Do not set it.
//
// Treat the number as relative. Panchip marks RSSI "for test use" even in the
// datasheet that documents it, so it is not a calibrated dBm figure.
// -----------------------------------------------------------------------------
uint8_t XN297L::enableRssi(uint8_t gain) {
  uint8_t rf[3];
  readRegMulti(XN_REG_RF_CAL, rf, 3);
  rf[1] &= (uint8_t)~(1 << 7);         // 0x1E bit 15 = byte 1, bit 7
  rf[2] &= (uint8_t)~(1 << 0);         // 0x1E bit 16 = byte 2, bit 0
  if (gain & 0x01) rf[1] |= (1 << 7);
  if (gain & 0x02) rf[2] |= (1 << 0);
  writeRegMulti(XN_REG_RF_CAL, rf, 3);
  return readReg(XN_REG_RSSI);
}

uint8_t XN297L::setRssiEnable(bool on) {
  uint8_t v = readReg(XN_REG_RF_SETUP);
  v = on ? (v | XN_RF_RSSI_EN) : (v & (uint8_t)~XN_RF_RSSI_EN);
  writeReg(XN_REG_RF_SETUP, v);
  return readReg(XN_REG_RF_SETUP);
}

void XN297L::setAddress(const uint8_t *addr, uint8_t len) {
  if (len != XN_ADDR_WIDTH) return;
  memcpy(_addr, addr, XN_ADDR_WIDTH);
  // Address bytes go out LS byte first, so addr[0] is the low byte. Both ends
  // use the same array, so the ordering only matters if you compare against a
  // sniffer capture.
  // Pipe 0 must match TX_ADDR so the auto-ack finds its way back to us
  // (Section 5.2).
  writeRegMulti(XN_REG_TX_ADDR,    _addr, XN_ADDR_WIDTH);
  writeRegMulti(XN_REG_RX_ADDR_P0, _addr, XN_ADDR_WIDTH);
}

// ---- receive ----------------------------------------------------------------

void XN297L::startListening() {
  ceLow();
  writeReg(XN_REG_CONFIG, _baseConfig | XN_CFG_PRIM_RX);
  writeReg(XN_REG_STATUS, XN_ST_RX_DR | XN_ST_TX_DS | XN_ST_MAX_RT);
  command(XN_CMD_FLUSH_RX);
  ceHigh();
  delayMicroseconds(400);    // STB3 -> RX: 320 us (Figure 4-1)
  _listening = true;
}

void XN297L::stopListening() {
  ceLow();
  delayMicroseconds(50);     // RX -> STB3: 10 us
  command(XN_CMD_FLUSH_TX);
  writeReg(XN_REG_CONFIG, _baseConfig);
  delayMicroseconds(50);
  _listening = false;
}

bool XN297L::available() {
  // STATUS.RX_P_NO reads 111 when the RX FIFO is empty, and the pipe number
  // otherwise. Preferred over FIFO_STATUS here because the manual's bit table
  // for FIFO_STATUS bits 0 and 1 has its names and descriptions crossed.
  if (XN_ST_RX_P_NO(status()) == 0x07) return false;

  // Latch RSSI here, at the earliest moment software can: RSSI_SY is sampled
  // at packet sync, and reading it after R_RX_PAYLOAD or after CE drops may be
  // too late. Register 0x09 is one of the undocumented "special function"
  // registers, so treat the number as relative, not as dBm.
  _rxRssi = readReg(XN_REG_RSSI);
  return true;
}

uint8_t XN297L::read(void *buf, uint8_t maxLen) {
  uint8_t n = maxLen < XN_PAYLOAD_SIZE ? maxLen : XN_PAYLOAD_SIZE;
  uint8_t *p = (uint8_t *)buf;

  beginTxn();
  _spi->transfer(XN_CMD_R_RX_PAYLOAD);
  for (uint8_t i = 0; i < XN_PAYLOAD_SIZE; i++) {
    uint8_t b = _spi->transfer(XN_CMD_NOP);
    if (i < n) p[i] = b;    // always clock out the full fixed-size payload
  }
  endTxn();

  writeReg(XN_REG_STATUS, XN_ST_RX_DR);
  return n;
}

// ---- transmit ---------------------------------------------------------------

bool XN297L::send(const void *buf, uint8_t len, uint32_t timeoutMs) {
  return transmit(XN_CMD_W_TX_PAYLOAD, buf, len, timeoutMs);
}

bool XN297L::sendNoAck(const void *buf, uint8_t len, uint32_t timeoutMs) {
  return transmit(XN_CMD_W_TX_PAYLOAD_NOACK, buf, len, timeoutMs);
}

bool XN297L::transmit(uint8_t writeCmd, const void *buf, uint8_t len,
                      uint32_t timeoutMs) {
  if (_listening) stopListening();

  command(XN_CMD_FLUSH_TX);
  writeReg(XN_REG_STATUS, XN_ST_RX_DR | XN_ST_TX_DS | XN_ST_MAX_RT);

  const uint8_t *p = (const uint8_t *)buf;
  if (len > XN_PAYLOAD_SIZE) len = XN_PAYLOAD_SIZE;

  // The payload length is set by how many bytes we clock in, and with static
  // lengths it has to equal the receiver's RX_PW_P0 (Section 6.4.4).
  beginTxn();
  _spi->transfer(writeCmd);
  for (uint8_t i = 0; i < XN_PAYLOAD_SIZE; i++)
    _spi->transfer(i < len ? p[i] : 0x00);
  endTxn();

  // Section 5.3: CE must stay high for more than 30 us for the transmission to
  // take effect. We hold it up through the ACK window as well -- with the FIFO
  // drained the chip just parks in standby-II.
  ceHigh();
  delayMicroseconds(40);

  uint32_t start = millis();
  uint8_t st = 0;
  while (millis() - start < timeoutMs) {
    st = status();
    if (st & (XN_ST_TX_DS | XN_ST_MAX_RT)) break;
  }
  ceLow();

  _observeTx = readReg(XN_REG_OBSERVE_TX);
  writeReg(XN_REG_STATUS, XN_ST_RX_DR | XN_ST_TX_DS | XN_ST_MAX_RT);
  // On MAX_RT the payload stays in the FIFO on purpose (Section 5.8); drop it,
  // we are sending fresh data every second.
  if (st & XN_ST_MAX_RT) command(XN_CMD_FLUSH_TX);

  return (st & XN_ST_TX_DS) != 0;
}

// ---- diagnostics ------------------------------------------------------------

void XN297L::dumpRegisters(Stream &out) {
  struct RegInfo { uint8_t reg; uint8_t len; const char *name; };
  static const RegInfo regs[] = {
    {XN_REG_CONFIG,      1, "CONFIG     "}, {XN_REG_EN_AA,       1, "EN_AA      "},
    {XN_REG_EN_RXADDR,   1, "EN_RXADDR  "}, {XN_REG_SETUP_AW,    1, "SETUP_AW   "},
    {XN_REG_SETUP_RETR,  1, "SETUP_RETR "}, {XN_REG_RF_CH,       1, "RF_CH      "},
    {XN_REG_RF_SETUP,    1, "RF_SETUP   "}, {XN_REG_STATUS,      1, "STATUS     "},
    {XN_REG_OBSERVE_TX,  1, "OBSERVE_TX "}, {XN_REG_RSSI,        1, "RSSI       "},
    {XN_REG_RX_ADDR_P0,  5, "RX_ADDR_P0 "}, {XN_REG_TX_ADDR,     5, "TX_ADDR    "},
    {XN_REG_RX_PW_P0,    1, "RX_PW_P0   "}, {XN_REG_FIFO_STATUS, 1, "FIFO_STATUS"},
    {XN_REG_DYNPD,       1, "DYNPD      "}, {XN_REG_FEATURE,     1, "FEATURE    "},
    {XN_REG_BB_CAL,      5, "BB_CAL     "},
  };

  out.println(F("--- XN297L registers ---"));
  for (const RegInfo &r : regs) {
    uint8_t buf[5];
    readRegMulti(r.reg, buf, r.len);
    out.printf("  0x%02X %s =", r.reg, r.name);
    for (uint8_t i = 0; i < r.len; i++) out.printf(" %02X", buf[i]);
    out.println();
  }
  out.println(F("------------------------"));
}
