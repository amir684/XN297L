// -----------------------------------------------------------------------------
// ESP32 + XN297L -- bidirectional link with a mutual quality report.
//
// Both boards run the same code and both transmit. The roles only decide who
// speaks first, so the two never talk over each other:
//
//   tx  = initiator -- sends a packet every second (ACKed), then listens
//   rx  = responder -- listens, and answers each packet with telemetry
//
// The forward packet is ACKed, so the initiator learns how many retransmits it
// took. The answer is deliberately NOT ACKed: it is pure telemetry riding back
// on an exchange the peer already acknowledged, and demanding an ACK for it
// only produced 15 pointless retransmits into a board that had already stopped
// listening.
//
// The two numbers that describe the link, both documented and both free:
//   retries -- ARC_CNT, how hard the initiator had to work to get through
//   lost    -- gaps in the counter sequence, packets the responder never saw
//
// Plus RSSI, which is neither documented nor free -- register 0x09 works once
// RSSI_GAIN_CTR is set, but that setting costs 6 dB of receive sensitivity, so
// only the responder measures. See README.md.
//
//   pio run -e tx -t upload
//   pio run -e rx -t upload
//
// Wiring is identical on both boards -- see README.md.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>

#include "xn297l.h"

#if !defined(ROLE_TX) && !defined(ROLE_RX) && !defined(ROLE_SCAN)
#error "Build with -DROLE_TX, -DROLE_RX or -DROLE_SCAN (tx / rx / scan env)"
#endif

// ---- pin map ----
#if defined(BOARD_C3)
// ESP32-C3 boards. GPIO5/6 belong to the 0.42" OLED where one is fitted, 2/8/9
// are strapping pins, 20/21 are the UART -- everything below avoids those. The
// C3 routes SPI through the GPIO matrix, so these are free choices.
//
// Shared by every C3 board here, including the SuperMini that has no display
// and could spare 5/6: one jumper harness then fits both, and swapping roles
// between boards needs no rewiring.
static const int PIN_SCK  = 4;
static const int PIN_MISO = 3;
static const int PIN_MOSI = 10;
static const int PIN_CSN  = 7;
static const int PIN_CE   = 1;
static const int PIN_IRQ  = 0;
#else
// ESP32 WROOM, hardware VSPI.
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;
static const int PIN_CSN  = 5;
static const int PIN_CE   = 4;
static const int PIN_IRQ  = 16;   // wired but not used yet; polling for now
#endif

#ifdef BOARD_C3_SUPERMINI
// The SuperMini has no display, so its one output is the on-board LED. Worth
// wiring up: it is the only way to see the board is alive and transmitting
// without opening a serial monitor. Active low, and it shares GPIO8 with a
// strapping pin -- fine to drive as an output once boot is over, though it may
// glow during reset.
static const int PIN_LED = 8;

static void ledBlink(bool ok) {
  digitalWrite(PIN_LED, LOW);          // on
  delay(ok ? 20 : 200);                // a long flash means the packet failed
  digitalWrite(PIN_LED, HIGH);         // off
}
#endif

// ---- link settings (must match on both boards) ----
static const uint8_t RF_CHANNEL = 78;                              // 2478 MHz
static uint8_t       RF_ADDRESS[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};

// Running SPI clock. The driver forces 1 MHz during init regardless -- the
// chip caps SPI at 1 Mbps while in power-down / standby-I -- and only steps up
// to this once the radio is in standby-III. Drop it to XN_SPI_HZ_SLOW if long
// wires to a salvaged module make the link flaky.
static const uint32_t SPI_HZ = XN_SPI_HZ_FAST;

// Experiment, already run on this hardware -- leave off. Setting RF_SETUP bit 7
// asked the chip whether it behaves like an XN297 (bit 7 = RSSI_EN) or a real
// XN297L (bit 7 = top of RF_DR). The answer was XN297L: data still crossed, but
// the auto-ACK stopped and 0x09 stayed at zero. See README.md.
static const bool TRY_RSSI_EN = false;

// Undocumented RSSI: RSSI_GAIN_CTR in RF_CAL (0x1E bits 16:15), located by the
// scan env. 0 is the chip's reset value and reports nothing; 1 = -6 dB, 2 =
// -12 dB, 3 = -18 dB; -1 skips the write entirely.
//
// The attenuation is NOT confined to the detector -- it costs real receive
// sensitivity. With it on at both ends the link fell apart: retries climbed to
// 2-3, MAX_RT became common, and the telemetry answer stopped arriving
// entirely (answers=0/183). The forward packet survived because auto-ack gives
// it 15 retransmits to absorb the loss; the answer is sent NOACK and has no
// such margin, so it was the half that vanished.
//
// The attenuation only exists while the bits are set, though, and setting them
// is one SPI write. So instead of choosing which board pays, both boards arm
// RSSI for one round in RSSI_SAMPLE_EVERY and run at full sensitivity the rest
// of the time. Both directions get measured, and the link spends 3 rounds out
// of 4 with nothing in its way.
//
// Set RSSI_GAIN to -1 to take RSSI out of the picture completely.
static const int      RSSI_GAIN         = 1;
static const uint32_t RSSI_SAMPLE_EVERY = 4;

// Gap between the initiator's packets. Drop it to ~100 while running the scan
// env on the other board -- the sweep needs a packet per candidate and there
// are 56 of them.
static const uint32_t TX_INTERVAL_MS = 1000;

// How long the initiator waits for the responder to turn around and answer.
static const uint32_t REPLY_TIMEOUT_MS = 20;
// The responder pauses this long before answering, so the initiator has time
// to finish switching from TX back into RX (standby-III -> RX takes 320us).
static const uint32_t TURNAROUND_MS = 3;

static XN297L radio(PIN_CE, PIN_CSN, PIN_IRQ);

// ---- what travels over the air ----
// Fixed 32-byte payloads, so this only has to stay under 32; the driver pads.
// A fixed magic word leads the payload so a receiver can tell a real packet
// from noise it merely managed to clock into the FIFO. Without this the sweep
// below cannot distinguish "reception still works" from "the chip now accepts
// anything", and that distinction is the whole point of the sweep.
static const uint32_t LINK_MAGIC = 0x4C4E5837UL;

struct LinkPacket {
  uint32_t magic;      // LINK_MAGIC
  uint32_t counter;    // initiator: packet number. responder: echoes it back.
  uint16_t lost;       // responder: packets it never saw, running total
  uint8_t  retries;    // initiator: ARC_CNT on its last transmit
  uint8_t  rssi;       // how strongly the sender heard US, 0-15 (RSSI_SY),
                       // or RSSI_OFF if that board is not measuring
  int16_t  tempDeciC;  // sender's die temperature in tenths of a degree,
                       // or TEMP_NONE on a board without a usable sensor
};

// The ESP32-C3 has a real on-chip temperature sensor. The original ESP32 does
// not have a usable one, so those boards say so rather than send a made-up
// number that would look like a measurement.
static const int16_t TEMP_NONE = INT16_MIN;

__attribute__((unused))
static int16_t readTempDeciC() {
#if defined(BOARD_C3)
  return (int16_t)lroundf(temperatureRead() * 10.0f);
#else
  return TEMP_NONE;
#endif
}

// A board with RSSI disabled has to say so. Sending a plain 0 would read as
// "I hear you at zero", which is a measurement, not an absence of one.
static const uint8_t RSSI_OFF = 0xFF;

// RSSI_SY is a 4-bit field, so 15 is its theoretical top, but on this hardware
// the readings run 0..8 and stop -- most likely because the 6 dB of attenuation
// we have to switch on to get a reading at all eats the upper half of the
// scale. Scaling a bar graph to 15 would leave it stuck around the middle
// forever, so scale to what the hardware actually produces.
//
// This is measured, not specified. If a reading ever exceeds it -- try holding
// the two boards against each other -- raise this to match.
static const uint8_t RSSI_FULL_SCALE = 8;

// The 4-bit reading is noisy -- it swings several counts between consecutive
// packets with nothing moving -- so keep a short running mean alongside it.
static uint8_t  rssiHist[8];
static uint8_t  rssiCount = 0;
static uint8_t  rssiPos   = 0;

__attribute__((unused))
static uint8_t rssiAverage(uint8_t sample) {
  rssiHist[rssiPos] = sample;
  rssiPos = (uint8_t)((rssiPos + 1) % 8);
  if (rssiCount < 8) rssiCount++;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < rssiCount; i++) sum += rssiHist[i];
  return (uint8_t)(sum / rssiCount);
}

// Formats a value received from the peer, which may be "not measuring".
__attribute__((unused))
static const char *peerRssi(uint8_t v, char *buf, size_t n) {
  if (v == RSSI_OFF) snprintf(buf, n, "off");
  else               snprintf(buf, n, "%u", v);
  return buf;
}

// ---- on-board OLED (ESP32-C3 0.42" boards) ----------------------------------
#ifdef BOARD_C3_OLED
#include <Wire.h>
#include <U8g2lib.h>

// Two things about this panel bite everyone who wires it up blind:
//
// 1. It is a 72x40 window inside a 128x64 SSD1306 controller, offset by
//    (28,24). Drive it as a plain 128x64 and the image lands off-screen.
//    U8g2 has a device entry for exactly this part, so the offset is handled.
//
// 2. U8g2's hardware-I2C path calls Wire.begin() with no arguments, which picks
//    the core's default SDA/SCL for the chip -- not this board's. Hence the
//    software-I2C constructor, which takes the pins explicitly.
static const int PIN_OLED_SDA = 5;
static const int PIN_OLED_SCL = 6;

static U8G2_SSD1306_72X40_ER_F_SW_I2C oled(U8G2_R0, PIN_OLED_SCL, PIN_OLED_SDA,
                                           U8X8_PIN_NONE);

// U8g2 maps buffer column 0 to controller column 28, but these panels are not
// all cut the same -- on this one the visible window starts two columns later,
// so buffer columns 0-1 fall off the left edge and clip the first character.
// The offset is baked into U8g2's device table and cannot be set at runtime,
// so shift the drawing instead. Bump to 3 if a sliver is still missing; the
// usable width shrinks to match, so nothing runs off the right.
static const int OLED_X = 2;
static const int OLED_W = 72 - OLED_X;

// Vertical nudge, same idea as OLED_X. Zero on this panel -- the rows below
// already fill 0..39 exactly. If a future panel clips the top instead, this is
// the knob.
static const int OLED_Y = 0;

// Row baselines. u8g2 takes the BASELINE, not the top, and the display is rows
// 0..39 -- so a baseline of 40 puts the glyph one row past the end and clips
// it. The bottom row is uppercase only, no descenders, so a baseline of 39 sits
// flush against the last row.
static const int ROW1_BASE = 6 + OLED_Y;    // 5x7  -> rows 0..6
static const int BAR_TOP   = 9 + OLED_Y;    //         rows 9..21
static const int BAR_H     = 13;
static const int ROW3_BASE = 30 + OLED_Y;   // 5x7  -> rows 24..30
static const int ROW4_BASE = 39 + OLED_Y;   // 6x10 -> rows 32..39

// If the display stays dark, this says whether anything answers on those pins
// at all -- which separates "wrong pins" from "wrong driver".
static void oledScanI2C() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Serial.print(F("I2C scan on SDA/SCL "));
  Serial.printf("%d/%d:", PIN_OLED_SDA, PIN_OLED_SCL);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", addr); found++; }
  }
  if (!found) Serial.print(F(" nothing -- check PIN_OLED_SDA / PIN_OLED_SCL"));
  Serial.println();
  Wire.end();          // hand the pins back before software I2C bit-bangs them
}

// SSD1306 register 0xD3 sets the display offset -- which COM row the panel
// starts showing from. It is the one knob that can move the image UP, which a
// 40-row buffer cannot do on its own: there is nothing above row 0 to draw in.
// U8g2 programs it during begin() for its idea of this panel; this overrides it.
static void oledSetComOffset(uint8_t v) {
  u8x8_t *u8x8 = oled.getU8x8();
  u8x8_cad_StartTransfer(u8x8);
  u8x8_cad_SendCmd(u8x8, 0x0D3);
  u8x8_cad_SendArg(u8x8, (uint8_t)(v & 0x3F));
  u8x8_cad_EndTransfer(u8x8);
}

// Brightness. U8g2 leaves the panel near the SSD1306 reset defaults, which on
// these tiny 0.42" glasses looks washed out. Three registers matter:
//   0x81 contrast        -- the main one, 0..255
//   0xD9 pre-charge      -- a longer pre-charge drives the pixels harder
//   0xDB VCOMH deselect  -- a higher deselect level raises apparent contrast
//
// Running an OLED flat out shortens its life, and a status screen is mostly
// static. Drop OLED_CONTRAST if this ends up somewhere it stays lit for days.
static const uint8_t OLED_CONTRAST = 255;

// The pre-charge and VCOMH writes below blanked this panel outright, both with
// an out-of-range VCOMH and with a legal one -- U8g2 tunes those two registers
// for this specific glass during begin(), and overriding them with values
// borrowed from generic 128x64 init sequences does not survive here. Contrast
// alone is a documented U8g2 call and cannot put the panel in an undefined
// state, so that is the default. Flip this on only to experiment.
static const bool OLED_TUNE_ANALOG = false;

// The charge pump sets the panel's drive voltage, and it is the only lever left
// once contrast is maxed. 0x14 is the standard SSD1306 "on" value; some of the
// SSD1315 parts that ship on these boards take 0x95 for a higher voltage and
// come out noticeably brighter. On a true SSD1306 it either does nothing or
// misbehaves -- recoverable with a power cycle, like everything else here.
// -1 leaves whatever U8g2 configured. Try 0x95, then 0x14.
static const int OLED_CHARGE_PUMP = -1;

static void oledSetChargePump(uint8_t v) {
  u8x8_t *u8x8 = oled.getU8x8();
  u8x8_cad_StartTransfer(u8x8);
  u8x8_cad_SendCmd(u8x8, 0x08D);
  u8x8_cad_SendArg(u8x8, v);
  u8x8_cad_EndTransfer(u8x8);
}

static void oledSetBrightness(uint8_t contrast) {
  oled.setContrast(contrast);                 // 0x81, the one that matters

  if (OLED_CHARGE_PUMP >= 0) oledSetChargePump((uint8_t)OLED_CHARGE_PUMP);

  if (!OLED_TUNE_ANALOG) return;

  u8x8_t *u8x8 = oled.getU8x8();
  u8x8_cad_StartTransfer(u8x8);
  u8x8_cad_SendCmd(u8x8, 0x0D9);              // pre-charge period
  u8x8_cad_SendArg(u8x8, 0x0F1);              // phase 1 = 1, phase 2 = 15
  u8x8_cad_SendCmd(u8x8, 0x0DB);              // VCOMH deselect level
  u8x8_cad_SendArg(u8x8, 0x030);              // only 0x00 / 0x20 / 0x30 are legal
  u8x8_cad_EndTransfer(u8x8);
}

// Measured on this panel with the calibration sweep below: at 12 the 72x40
// buffer sits flush against all four edges of the glass. -1 would leave U8g2's
// own setting alone, which lands the image a few rows low and clips the bottom.
static const int OLED_COM_OFFSET = 12;

// Calibration screen. These 72x40 panels are not all cut alike, and the buffer
// -> glass mapping cannot be read off a datasheet, so measure it instead of
// guessing: this draws the full 72x40 buffer outline, a filled 4x4 square in
// each corner, and tick marks every 8 rows down the left edge. Which of those
// survive on the glass says exactly how the buffer is landing.
// Already run on this hardware -- the answer is OLED_COM_OFFSET above. Turn it
// back on if you swap in a differently-cut panel.
static const bool OLED_CALIBRATE = false;

// Draws the outline of the whole 72x40 buffer with a big number in the middle.
// When the outline sits flush against all four edges of the glass, the number
// on screen is the offset to keep.
static void oledCalibrationFrame(int value) {
  char n[8];
  oled.clearBuffer();

  oled.drawFrame(OLED_X, 0, OLED_W, 40);   // outline, with the known x shift
  oled.drawBox(OLED_X, 0, 4, 4);           // corners
  oled.drawBox(OLED_X + OLED_W - 4, 0, 4, 4);
  oled.drawBox(OLED_X, 36, 4, 4);
  oled.drawBox(OLED_X + OLED_W - 4, 36, 4, 4);

  snprintf(n, sizeof(n), "%d", value);
  oled.setFont(u8g2_font_logisoso16_tn);
  oled.drawStr(OLED_X + 22, 30, n);

  oled.sendBuffer();
}

// Steps the display offset through a range, holding each value long enough to
// judge it. Reading the number off the glass beats guessing at the register.
static void oledCalibrationSweep() {
  Serial.println();
  Serial.println(F("OLED calibration: stepping SSD1306 display offset (0xD3)."));
  Serial.println(F("Watch the glass. When the rectangle sits flush against all"));
  Serial.println(F("four edges, note the big number -- that is your value."));
  Serial.println(F("Then set OLED_COM_OFFSET to it and OLED_CALIBRATE to false."));
  Serial.println();

  for (int v = 0; v <= 32; v++) {
    oledSetComOffset((uint8_t)v);
    oledCalibrationFrame(v);
    Serial.printf("  offset %d\n", v);
    delay(1200);
  }

  Serial.println(F("sweep done -- rerun or pick the best value now."));
}

// Does setContrast reach this panel at all? Steps 0 -> 255 with the value on
// screen. If the glass visibly darkens near 0 and brightens again, contrast
// works and 255 really is the ceiling -- the panel is simply dim. If nothing
// changes across the whole range, the register is not getting through and the
// brightness has to come from somewhere else.
static const bool OLED_CONTRAST_SWEEP = false;

static void oledContrastSweep() {
  Serial.println();
  Serial.println(F("Contrast sweep 0..255. Watch whether the glass changes at all."));
  for (int v = 0; v <= 255; v += 15) {
    oled.setContrast(v);
    oledCalibrationFrame(v);
    Serial.printf("  contrast %d\n", v);
    delay(700);
  }
  oled.setContrast(OLED_CONTRAST);
  Serial.println(F("sweep done."));
}

// The glass shows a slightly wider window than U8g2 writes to: U8g2 fills
// controller columns 28..99, this panel displays 30..101. The last two columns
// are visible but outside the 72-column buffer, so U8g2 never writes them and
// they keep whatever the controller powered up with -- a dirty stripe down the
// right edge that clearBuffer() cannot reach, because those columns do not
// exist as far as the buffer is concerned.
//
// Zero the whole 128x64 GDDRAM once, before anything is drawn. Everything U8g2
// does touch gets overwritten on the first flush; everything it does not stays
// black. Unlike the pre-charge and VCOMH experiments, this only writes pixel
// data -- no configuration register is touched, so it cannot leave the panel in
// an undefined state.
static void oledClearGddram() {
  static uint8_t zeros[128] = {0};
  u8x8_t *u8x8 = oled.getU8x8();

  for (uint8_t page = 0; page < 8; page++) {
    u8x8_cad_StartTransfer(u8x8);
    u8x8_cad_SendCmd(u8x8, (uint8_t)(0x0B0 | page));  // page address
    u8x8_cad_SendCmd(u8x8, 0x000);                    // column low nibble = 0
    u8x8_cad_SendCmd(u8x8, 0x010);                    // column high nibble = 0
    u8x8_cad_SendData(u8x8, 128, zeros);
    u8x8_cad_EndTransfer(u8x8);
  }
}

static void oledBegin() {
  oledScanI2C();
  oled.begin();

  oledSetBrightness(OLED_CONTRAST);
  oledClearGddram();

  if (OLED_CONTRAST_SWEEP) oledContrastSweep();
  if (OLED_CALIBRATE) oledCalibrationSweep();
  if (OLED_COM_OFFSET >= 0) oledSetComOffset((uint8_t)OLED_COM_OFFSET);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(OLED_X, 12 + OLED_Y, "XN297L");
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 27 + OLED_Y, "waiting for");
  oled.drawStr(OLED_X, 37 + OLED_Y, "a packet...");
  oled.sendBuffer();
}

// 72x40 is small enough that every row has to earn its place: packet count and
// losses on top, a bar for how loudly we hear the peer, the two RSSI numbers,
// and a link state that is readable without reading any of the numbers.
// ---- pages -------------------------------------------------------------------
// The BOOT button cycles the screen. It is GPIO9, the C3's boot strapping pin:
// safe to read as an input once running, but holding it down through a reset
// drops the chip into download mode -- that is the button doing its day job,
// not a fault.
static const int PIN_BOOT = 9;

enum OledPage : uint8_t {
  PAGE_OVERVIEW, PAGE_TEMP, PAGE_SIGNAL, PAGE_STATS, PAGE_GRAPH, PAGE_COUNT
};
static uint8_t oledPage = PAGE_OVERVIEW;

// Latest values, kept so a page can be redrawn the instant the button is
// pressed instead of waiting for the next packet to arrive.
static struct {
  uint32_t seen = 0;
  uint16_t lost = 0;
  uint8_t  mine = RSSI_OFF;
  uint8_t  mineAvg = 0;
  uint8_t  theirs = RSSI_OFF;
  int16_t  temp = TEMP_NONE;
  bool     stale = true;
  uint32_t startMs = 0;
} st;

// One temperature sample per received packet, one pixel column each.
static const uint8_t TEMP_HIST = 68;          // == OLED_W - 2
static int16_t tempHist[TEMP_HIST];
static uint8_t tempHistCount = 0;
static uint8_t tempHistPos   = 0;

static void tempHistPush(int16_t v) {
  if (v == TEMP_NONE) return;
  tempHist[tempHistPos] = v;
  tempHistPos = (uint8_t)((tempHistPos + 1) % TEMP_HIST);
  if (tempHistCount < TEMP_HIST) tempHistCount++;
}

static void drawSignalBar(int y, int h) {
  oled.drawFrame(OLED_X, y, OLED_W, h);
  if (!st.stale && st.mine != RSSI_OFF) {
    uint8_t v = st.mine > RSSI_FULL_SCALE ? RSSI_FULL_SCALE : st.mine;
    int w = v * (OLED_W - 2) / RSSI_FULL_SCALE;
    if (w) oled.drawBox(OLED_X + 1, y + 2, w, h - 4);
  }
}

// Centres a string on the usable width for a fixed-width font.
static void drawCentred(const char *s, int baseline, int charW) {
  oled.drawStr(OLED_X + (OLED_W - (int)strlen(s) * charW) / 2, baseline, s);
}

static void pageOverview() {
  char line[24], a[8], b[8];

  oled.setFont(u8g2_font_5x7_tf);
  snprintf(line, sizeof(line), "#%lu", (unsigned long)st.seen);
  oled.drawStr(OLED_X, ROW1_BASE, line);
  snprintf(line, sizeof(line), "L%u", st.lost);
  oled.drawStr(OLED_X + OLED_W - (int)strlen(line) * 5, ROW1_BASE, line);

  if (st.temp == TEMP_NONE) snprintf(line, sizeof(line), "--");
  else snprintf(line, sizeof(line), "%dC", (int)lroundf(st.temp / 10.0f));
  drawCentred(line, ROW1_BASE, 5);

  drawSignalBar(BAR_TOP, BAR_H);

  snprintf(line, sizeof(line), "me%s th%s",
           peerRssi(st.mine, a, sizeof(a)), peerRssi(st.theirs, b, sizeof(b)));
  oled.drawStr(OLED_X, ROW3_BASE, line);

  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(OLED_X, ROW4_BASE, st.stale ? "NO LINK" : "LINK OK");
}

static void pageTemp() {
  char line[16];

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, "TX TEMP");

  if (st.temp == TEMP_NONE) {
    oled.setFont(u8g2_font_6x10_tf);
    drawCentred("no sensor", 28, 6);
    return;
  }

  // logisoso20_tn is digits and punctuation only -- the degree mark and C go
  // next to it in the small font.
  snprintf(line, sizeof(line), "%.1f", st.temp / 10.0f);
  oled.setFont(u8g2_font_logisoso20_tn);
  int w = oled.getStrWidth(line);
  int x = OLED_X + (OLED_W - w - 8) / 2;
  oled.drawStr(x, 36, line);

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(x + w + 2, 22, "o");
  oled.drawStr(x + w + 2, 30, "C");
}

static void pageSignal() {
  char line[16], a[8], b[8];

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, "SIGNAL");
  snprintf(line, sizeof(line), "/%u", RSSI_FULL_SCALE);
  oled.drawStr(OLED_X + OLED_W - (int)strlen(line) * 5, 7, line);

  peerRssi(st.mine, a, sizeof(a));
  oled.setFont(u8g2_font_logisoso16_tn);
  if (st.mine != RSSI_OFF) oled.drawStr(OLED_X, 26, a);

  oled.setFont(u8g2_font_5x7_tf);
  snprintf(line, sizeof(line), "avg%u th%s", st.mineAvg,
           peerRssi(st.theirs, b, sizeof(b)));
  oled.drawStr(OLED_X + 22, 26, line);

  drawSignalBar(30, 10);
}

static void pageStats() {
  char line[20];
  uint32_t up = (millis() - st.startMs) / 1000;

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, "STATS");

  snprintf(line, sizeof(line), "seen %lu", (unsigned long)st.seen);
  oled.drawStr(OLED_X, 17, line);
  snprintf(line, sizeof(line), "lost %u", st.lost);
  oled.drawStr(OLED_X, 26, line);

  if (up < 3600) snprintf(line, sizeof(line), "up %lum%02lus",
                          (unsigned long)(up / 60), (unsigned long)(up % 60));
  else           snprintf(line, sizeof(line), "up %luh%02lum",
                          (unsigned long)(up / 3600), (unsigned long)((up / 60) % 60));
  oled.drawStr(OLED_X, 35, line);
}

static void pageGraph() {
  char line[20];

  oled.setFont(u8g2_font_5x7_tf);
  if (st.temp == TEMP_NONE) snprintf(line, sizeof(line), "TEMP --");
  else snprintf(line, sizeof(line), "TEMP %.1f", st.temp / 10.0f);
  oled.drawStr(OLED_X, 7, line);

  if (tempHistCount < 2) {
    oled.drawStr(OLED_X, 25, "collecting...");
    return;
  }

  // Auto-scale to the range actually present: on a die that drifts a degree
  // over minutes, a fixed scale would draw a flat line and say nothing.
  int16_t mn = INT16_MAX, mx = INT16_MIN;
  for (uint8_t i = 0; i < tempHistCount; i++) {
    int16_t v = tempHist[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  if (mx == mn) mx = (int16_t)(mn + 1);

  const int top = 11, bot = 39;
  int prevX = 0, prevY = 0;
  for (uint8_t i = 0; i < tempHistCount; i++) {
    uint8_t idx = (uint8_t)((tempHistPos + TEMP_HIST - tempHistCount + i) % TEMP_HIST);
    int y = bot - (int)((long)(tempHist[idx] - mn) * (bot - top) / (mx - mn));
    int x = OLED_X + 1 + i;
    if (i) oled.drawLine(prevX, prevY, x, y);
    prevX = x;
    prevY = y;
  }

  // Span of the visible window, so the trace has a scale.
  snprintf(line, sizeof(line), "%.1f", (mx - mn) / 10.0f);
  oled.drawStr(OLED_X + OLED_W - (int)strlen(line) * 5, 7, line);
}

static void oledDraw() {
  oled.clearBuffer();
  switch (oledPage) {
    case PAGE_TEMP:   pageTemp();   break;
    case PAGE_SIGNAL: pageSignal(); break;
    case PAGE_STATS:  pageStats();  break;
    case PAGE_GRAPH:  pageGraph();  break;
    default:          pageOverview(); break;
  }
  oled.sendBuffer();
}

// Falling edge with a debounce window. Polled from loop() so the page changes
// under your thumb rather than on the next packet.
static void oledPollButton() {
  static bool     prev = true;
  static uint32_t last = 0;

  bool now = digitalRead(PIN_BOOT);
  if (prev && !now && millis() - last > 250) {
    last = millis();
    oledPage = (uint8_t)((oledPage + 1) % PAGE_COUNT);
    oledDraw();
  }
  prev = now;
}

static void oledStatus(uint32_t seen, uint16_t lost, uint8_t mine, uint8_t avg,
                       uint8_t theirs, int16_t tempDeciC, bool stale) {
  st.seen    = seen;
  st.lost    = lost;
  st.mine    = mine;
  st.mineAvg = avg;
  st.theirs  = theirs;
  st.temp    = tempDeciC;
  st.stale   = stale;
  if (!stale) tempHistPush(tempDeciC);
  oledDraw();
}
#endif  // BOARD_C3_OLED

// -----------------------------------------------------------------------------

static void haltWithDiagnostics() {
  Serial.println();
  Serial.println(F("SPI self-test FAILED -- the module is not answering on MISO."));
  Serial.println(F("Check, in this order:"));
  Serial.println(F("  1. 3V3 and GND actually reach the module (measure at the pads)"));
  Serial.println(F("  2. MOSI/MISO are not swapped, CLK and CSN are on the right pins"));
  Serial.println(F("  3. wires are short (<10 cm) and the antenna wire is clear of them"));
  Serial.println(F("  4. a 10uF cap across the module 3V3/GND"));
  Serial.println(F("  5. the crystal is intact -- no 16MHz clock, no SPI answer"));
  Serial.println(F("A register dump follows. All 00 or all FF means no SPI at all."));
  radio.dumpRegisters(Serial);

#ifdef BOARD_C3_OLED
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(OLED_X, 12 + OLED_Y, "SPI FAIL");
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 27 + OLED_Y, "check wiring");
  oled.drawStr(OLED_X, 37 + OLED_Y, "see serial");
  oled.sendBuffer();
#endif

  for (;;) {
    delay(2000);
    Serial.println(F("halted -- fix the wiring and reset the board"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("========================================"));
#if defined(ROLE_TX)
  Serial.println(F("  ESP32 + XN297L  --  INITIATOR"));
#elif defined(ROLE_RX)
  Serial.println(F("  ESP32 + XN297L  --  RESPONDER"));
#else
  Serial.println(F("  ESP32 + XN297L  --  RSSI REGISTER SWEEP"));
#endif
  Serial.println(F("========================================"));

#ifdef BOARD_C3_OLED
  oledBegin();
#endif
#ifdef BOARD_C3_SUPERMINI
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);         // active low, so start dark
#endif
#ifdef BOARD_C3_OLED
  pinMode(PIN_BOOT, INPUT_PULLUP);
  st.startMs = millis();
#endif

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

  if (!radio.begin(&SPI, SPI_HZ)) haltWithDiagnostics();

  Serial.println(F("SPI self-test PASSED -- the module is alive."));

  radio.setChannel(RF_CHANNEL);
  radio.setAddress(RF_ADDRESS, sizeof(RF_ADDRESS));

  if (TRY_RSSI_EN) {
    uint8_t before = radio.readReg(XN_REG_RF_SETUP);
    uint8_t after  = radio.setRssiEnable(true);
    Serial.printf("RSSI_EN experiment: RF_SETUP %02X -> %02X\n", before, after);
    Serial.println(F("  link still up + register 09 non-zero => XN297 silicon"));
    Serial.println(F("  link broken                          => real XN297L"));
  }

  if (RSSI_GAIN >= 0) {
    uint8_t v = radio.enableRssi((uint8_t)RSSI_GAIN);
    Serial.printf("RSSI enabled (gain %d): 0x09 now reads %02X\n", RSSI_GAIN, v);
  }

  radio.dumpRegisters(Serial);
  Serial.println();

#ifdef ROLE_RX
  radio.startListening();
#endif
}

// -----------------------------------------------------------------------------

#if defined(ROLE_TX)   // initiator

void loop() {
  static uint32_t counter = 0;
  static uint32_t answered = 0;
  static uint8_t  myRssi = RSSI_OFF;   // how strongly I heard the responder
  static uint8_t  myAvg  = 0;

  LinkPacket out = {};
  out.magic     = LINK_MAGIC;
  out.counter   = counter;
  out.rssi      = myRssi;
  out.tempDeciC = readTempDeciC();

  bool acked   = radio.send(&out, sizeof(out));
  uint8_t tries = radio.lastObserveTx() & 0x0F;

  LinkPacket in = {};
  bool gotAnswer = false;

  // Arm the detector only for the rounds we actually sample on, so the answer
  // path runs at full sensitivity the rest of the time.
  bool sampling = (RSSI_GAIN >= 0) && (counter % RSSI_SAMPLE_EVERY == 0);
  if (RSSI_GAIN >= 0) radio.enableRssi(sampling ? (uint8_t)RSSI_GAIN : 0);

  if (acked) {
    // The ACK already proved the packet arrived; the answer only carries the
    // remote's view of the link, so losing one is not a link failure.
    radio.startListening();
    uint32_t start = millis();
    while (millis() - start < REPLY_TIMEOUT_MS) {
      if (radio.available()) {
        uint8_t latched = radio.lastRssi();
        radio.read(&in, sizeof(in));
        if (in.magic != LINK_MAGIC) continue;   // noise, not an answer
        if (sampling) {
          myRssi = latched & 0x0F;              // RSSI_SY, 0-15
          myAvg  = rssiAverage(myRssi);
        }
        gotAnswer = true;
        answered++;
        break;
      }
    }
    radio.stopListening();
  }

  Serial.printf("[%6lu] ", (unsigned long)counter);
  if (!acked) {
    Serial.println(F("NO ACK -- the other board did not receive this one"));
  } else if (gotAnswer) {
    char mine[8], theirs[8];
    int16_t t = out.tempDeciC;
    if (t == TEMP_NONE) Serial.print(F("T=--     "));
    else                Serial.printf("T=%.1fC  ", t / 10.0f);
    Serial.printf("i hear you=%-3s   you hear me=%-3s   retries=%u   "
                  "remote lost=%u   answers=%lu/%lu\n",
                  peerRssi(myRssi, mine, sizeof(mine)),
                  peerRssi(in.rssi, theirs, sizeof(theirs)),
                  tries, in.lost,
                  (unsigned long)answered, (unsigned long)(counter + 1));
    if (RSSI_GAIN >= 0 && myRssi != RSSI_OFF)
      Serial.printf("           (mean of last 8: %u)\n", myAvg);
  } else {
    Serial.printf("ok  retries=%u   (no answer)      answers=%lu/%lu\n",
                  tries, (unsigned long)answered, (unsigned long)(counter + 1));
  }

#ifdef BOARD_C3_SUPERMINI
  ledBlink(acked);
#endif

  counter++;
  delay(TX_INTERVAL_MS);
}

#elif defined(ROLE_RX)   // responder

void loop() {
  static uint32_t received  = 0;
  static uint32_t lastHeard = 0;
  static uint32_t expected  = 0;
  static uint16_t lost      = 0;
  static bool     started   = false;
  static bool     armed     = (RSSI_GAIN >= 0);   // first packet is a sample
  static uint8_t  lastRssi  = RSSI_OFF;
  static uint8_t  lastAvg   = 0;

#ifdef BOARD_C3_OLED
  oledPollButton();          // polled every pass, so the page turns instantly
#endif

  if (radio.available()) {
    uint8_t rssiAtPacket = radio.lastRssi();
    LinkPacket in = {};
    radio.read(&in, sizeof(in));
    if (in.magic != LINK_MAGIC) return;   // noise, not one of ours
    received++;
    lastHeard = millis();

    // Gaps in the initiator's counter are packets that never reached us. The
    // first packet only establishes where the sequence starts.
    if (started && in.counter > expected) lost += (uint16_t)(in.counter - expected);
    started  = true;
    expected = in.counter + 1;

    // Give the initiator time to get back into RX before we answer.
    delay(TURNAROUND_MS);

    LinkPacket out = {};
    out.magic     = LINK_MAGIC;
    out.counter   = in.counter;
    out.lost      = lost;
    out.tempDeciC = readTempDeciC();
    // rssiAtPacket only means anything if the detector was armed for the round
    // this packet arrived on; otherwise carry the previous reading forward.
    if (armed) {
      lastRssi = (uint8_t)(rssiAtPacket & 0x0F);
      lastAvg  = rssiAverage(lastRssi);
    }
    out.rssi = lastRssi;

    radio.sendNoAck(&out, sizeof(out));

    // Arm or disarm for the next packet, while the radio is still in standby.
    armed = (RSSI_GAIN >= 0) && (received % RSSI_SAMPLE_EVERY == 0);
    if (RSSI_GAIN >= 0) radio.enableRssi(armed ? (uint8_t)RSSI_GAIN : 0);

    radio.startListening();

    // The peer's own measurement rides in on every packet, so both consoles
    // show the same pair of numbers. RSSI_SY (low nibble) is the useful half:
    // it is latched at packet sync, whereas RSSI_RT has already decayed by the
    // time software gets round to reading 0x09.
    char mine[8], theirs[8];
    Serial.printf("[%6lu] ", (unsigned long)received);
    if (in.tempDeciC == TEMP_NONE) Serial.print(F("Ttx=--     "));
    else                           Serial.printf("Ttx=%.1fC  ", in.tempDeciC / 10.0f);
    Serial.printf("i hear you=%-3s (mean %u)   you hear me=%-3s   "
                  "lost=%u   seen=%lu\n",
                  peerRssi(lastRssi, mine, sizeof(mine)), lastAvg,
                  peerRssi(in.rssi, theirs, sizeof(theirs)),
                  lost, (unsigned long)received);

#ifdef BOARD_C3_OLED
    // in.tempDeciC is the initiator's temperature -- that is what goes on the
    // screen, not this board's own.
    oledStatus(received, lost, lastRssi, lastAvg, in.rssi, in.tempDeciC, false);
#endif
  }

  if (millis() - lastHeard > 5000) {
    lastHeard = millis();
    Serial.println(F("...nothing received for 5s"));
#ifdef BOARD_C3_OLED
    oledStatus(received, lost, lastRssi, lastAvg, RSSI_OFF, TEMP_NONE, true);
#endif
  }
}

#else  // ROLE_SCAN -- hunt for whatever gates RSSI on this silicon

// The XN297 gates RSSI behind RF_SETUP.RSSI_EN. On the XN297L that bit became
// the top of RF_DR (proven by experiment -- see README), so if the function
// still exists its enable moved into one of the registers the XN297L manual
// refuses to document. These three are the candidates:
static const uint8_t SCAN_REG[]  = {XN_REG_DEMOD_CAL, XN_REG_DEM_CAL2, XN_REG_RF_CAL};
static const uint8_t SCAN_LEN[]  = {1, 3, 3};
static const uint8_t SCAN_COUNT  = 3;
static const uint8_t MAX_REG_LEN = 3;

// Their power-on values, captured before we touch anything. Every write during
// the sweep is these bytes with exactly one bit flipped -- a full brute force
// over 7 bytes would be 2^56 combinations, and staying one bit from the
// factory state is both the more informative search and the safer one.
static uint8_t scanDefault[SCAN_COUNT][MAX_REG_LEN];

// How long to give each candidate. The initiator has to send at least one
// packet inside this window.
static const uint32_t CANDIDATE_MS = 1500;

static void restoreDefaults() {
  for (uint8_t i = 0; i < SCAN_COUNT; i++)
    radio.writeRegMulti(SCAN_REG[i], scanDefault[i], SCAN_LEN[i]);
}

// Listens for CANDIDATE_MS and returns the largest value register 0x09 showed.
//
// Counts GOOD and BAD packets separately, checked against LINK_MAGIC. The first
// version of this sweep only asked whether anything reached the RX FIFO, and
// that was useless: the bits it flagged as hits were ones that break packet
// validation, after which the chip clocks in noise, the FIFO is never empty,
// and 0x09 fills with junk that has nothing to do with signal strength. A
// candidate is only interesting if real packets still arrive.
//
// Samples 0x09 continuously rather than only on packet arrival, since a bit
// could switch on the real-time power detector without affecting packets.
static uint8_t probe(uint16_t &good, uint16_t &bad) {
  uint8_t best = 0;
  good = 0;
  bad  = 0;

  radio.startListening();
  uint32_t t0 = millis();
  while (millis() - t0 < CANDIDATE_MS) {
    uint8_t v = radio.readReg(XN_REG_RSSI);
    if (v > best) best = v;

    if (radio.available()) {
      uint8_t latched = radio.lastRssi();
      if (latched > best) best = latched;
      LinkPacket in = {};
      radio.read(&in, sizeof(in));
      if (in.magic == LINK_MAGIC) good++; else bad++;
    }
  }
  radio.stopListening();
  return best;
}

void loop() {
  static bool done = false;
  if (done) { delay(5000); return; }
  done = true;

  // Capture the factory values first. These have never been published for the
  // XN297L, so they are worth recording whether or not the sweep finds
  // anything.
  Serial.println(F("power-on values of the undocumented registers:"));
  for (uint8_t i = 0; i < SCAN_COUNT; i++) {
    radio.readRegMulti(SCAN_REG[i], scanDefault[i], SCAN_LEN[i]);
    Serial.printf("  0x%02X =", SCAN_REG[i]);
    for (uint8_t b = 0; b < SCAN_LEN[i]; b++) Serial.printf(" %02X", scanDefault[i][b]);
    Serial.println();
  }
  uint8_t rfcal2[6];
  radio.readRegMulti(XN_REG_RF_CAL2, rfcal2, 6);
  Serial.printf("  0x%02X =", XN_REG_RF_CAL2);
  for (uint8_t b = 0; b < 6; b++) Serial.printf(" %02X", rfcal2[b]);
  Serial.println(F("   (not swept)"));
  Serial.println();

  // Baseline: confirm 0x09 really is dead before we start flipping bits.
  restoreDefaults();
  uint16_t good = 0, bad = 0;
  uint8_t baseline = probe(good, bad);
  Serial.printf("baseline (defaults): 0x09 max = %02X, good = %u, junk = %u\n",
                baseline, good, bad);
  if (good == 0)
    Serial.println(F("  !! no valid packets -- start the tx board before scanning"));
  Serial.println();

  uint8_t hits = 0;
  for (uint8_t i = 0; i < SCAN_COUNT; i++) {
    for (uint8_t byteIdx = 0; byteIdx < SCAN_LEN[i]; byteIdx++) {
      for (uint8_t bit = 0; bit < 8; bit++) {
        uint8_t buf[MAX_REG_LEN];
        memcpy(buf, scanDefault[i], SCAN_LEN[i]);
        buf[byteIdx] ^= (uint8_t)(1 << bit);

        restoreDefaults();
        radio.writeRegMulti(SCAN_REG[i], buf, SCAN_LEN[i]);

        uint8_t best = probe(good, bad);

        // A hit has to move 0x09 AND leave real packets arriving. Junk counted
        // separately, because "0x09 moved" plus "the FIFO filled with garbage"
        // is the failure mode this sweep exists to rule out.
        bool hit = (best > baseline) && (good > 0);

        // Bit numbering is per the datasheet: byte 0 is the LS byte, so this
        // is bit (byteIdx*8 + bit) of the whole register.
        Serial.printf("  0x%02X bit %2u (byte %u.%u) -> 0x09 max = %02X  good=%-3u junk=%-3u%s\n",
                      SCAN_REG[i], byteIdx * 8 + bit, byteIdx, bit, best,
                      good, bad, hit ? "   <<< HIT" : "");
        if (hit) hits++;
      }
    }
  }

  restoreDefaults();
  Serial.println();
  Serial.printf("sweep done: %u candidate(s) moved register 0x09.\n", hits);
  if (hits == 0)
    Serial.println(F("no single-bit enable found in 0x19 / 0x1B / 0x1E."));
  Serial.println(F("registers restored to their power-on values."));
}

#endif
