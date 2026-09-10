// -----------------------------------------------------------------------------
// ESP32-C3 (0.42" OLED board) + XN297L  --  receiver for the STM32/MPU6881 link
//
// Listens for the 32-byte packets the STM32F030 sends ten times a second and
// shows them on the 72x40 glass. The BOOT button cycles through the pages:
//
//   HORIZON  artificial horizon drawn from roll and pitch
//   ANGLES   the same two angles as big numbers
//   ACCEL    X / Y / Z in g
//   GYRO     X / Y / Z in degrees per second
//   TEMP     MPU6881 die temperature, large
//   BATT     sender's cell voltage and a rough state of charge
//   GRAPH    that temperature over the last 68 packets, auto-scaled
//   LINK     signal bar, packets seen and lost, retransmits, uptime
//
// The radio half is the responder from amir684/XN297L with the reply removed:
// the sender uses auto-ack, so the hardware ACK already tells it the packet
// arrived and there is nothing left for a software answer to add. Dropping it
// also frees the receiver from turnaround timing, which is what makes a 10 Hz
// stream comfortable.
//
// Wiring (unchanged from that repo, so one jumper harness fits both boards):
//   SCK 4   MISO 3   MOSI 10   CSN 7   CE 1   IRQ 0 (unused, driver polls)
//   OLED is on-board: SDA 5, SCL 6
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>

#include <XN297L.h>
#include "link_packet.h"

// ---- radio pins (ESP32-C3) ----
// GPIO5/6 belong to the OLED, 2/8/9 are strapping pins, 20/21 are the UART.
// The C3 routes SPI through the GPIO matrix, so the rest are free choices.
static const int PIN_SCK  = 4;
static const int PIN_MISO = 3;
static const int PIN_MOSI = 10;
static const int PIN_CSN  = 7;
static const int PIN_CE   = 1;
// IRQ: GPIO0, wired but unused -- the library polls.

// ---- link settings (must match the sender) ----
static const uint8_t  RF_CHANNEL    = 78;                    // 2478 MHz
static uint8_t        RF_ADDRESS[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};

// Undocumented RSSI, located empirically in the original repo: RSSI_GAIN_CTR
// in RF_CAL. It costs real receive sensitivity -- 1 is -6 dB -- and here it is
// set once and left on, because toggling it means dropping out of RX and this
// board is listening almost all the time. Set to -1 to take RSSI out of the
// picture entirely and get those 6 dB back; the signal bar then reads "off".
static const int RSSI_GAIN = 1;

// RSSI_SY is 4 bits, but on this hardware the readings run 0..8 and stop,
// because the attenuation needed to get any reading eats the top of the scale.
// Scale the bar to what the hardware actually produces.
static const uint8_t RSSI_FULL_SCALE = 8;

// No packet for this long and the screen says so rather than showing numbers
// that quietly stopped being true.
static const uint32_t STALE_MS = 1500;

// The panel is bit-banged I2C and a full flush takes tens of milliseconds --
// long enough to miss a packet if it ran on every one. Ten frames a second is
// already smoother than the eye needs here.
static const uint32_t REDRAW_MS = 100;

static XN297L radio(PIN_CE, PIN_CSN);

// ---- on-board OLED ----------------------------------------------------------
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
// Bump to 3 if a sliver is still missing; the usable width shrinks to match.
static const int OLED_X = 2;
static const int OLED_W = 72 - OLED_X;

// Measured with the calibration sweep in the original repo: at 12 the 72x40
// buffer sits flush against all four edges of this glass.
static const int OLED_COM_OFFSET = 12;
static const uint8_t OLED_CONTRAST = 255;

// SSD1306 register 0xD3 sets which COM row the panel starts showing from. It
// is the one knob that can move the image UP, which a 40-row buffer cannot do
// on its own. U8g2 programs it during begin(); this overrides it.
static void oledSetComOffset(uint8_t v) {
  u8x8_t *u8x8 = oled.getU8x8();
  u8x8_cad_StartTransfer(u8x8);
  u8x8_cad_SendCmd(u8x8, 0x0D3);
  u8x8_cad_SendArg(u8x8, (uint8_t)(v & 0x3F));
  u8x8_cad_EndTransfer(u8x8);
}

// The glass shows a slightly wider window than U8g2 writes to: U8g2 fills
// controller columns 28..99, this panel displays 30..101. The last two columns
// keep whatever the controller powered up with -- a dirty stripe down the right
// edge that clearBuffer() cannot reach, because those columns do not exist as
// far as the buffer is concerned. Zeroing all of GDDRAM once, before anything
// is drawn, blacks them out for good.
static void oledClearGddram() {
  static uint8_t zeros[128] = {0};
  u8x8_t *u8x8 = oled.getU8x8();

  for (uint8_t page = 0; page < 8; page++) {
    u8x8_cad_StartTransfer(u8x8);
    u8x8_cad_SendCmd(u8x8, (uint8_t)(0x0B0 | page));  // page address
    u8x8_cad_SendCmd(u8x8, 0x000);                    // column low nibble
    u8x8_cad_SendCmd(u8x8, 0x010);                    // column high nibble
    u8x8_cad_SendData(u8x8, 128, zeros);
    u8x8_cad_EndTransfer(u8x8);
  }
}

// If the display stays dark, this says whether anything answers on those pins
// at all -- which separates "wrong pins" from "wrong driver".
static void oledScanI2C() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Serial.printf("I2C scan on SDA/SCL %d/%d:", PIN_OLED_SDA, PIN_OLED_SCL);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", addr); found++; }
  }
  if (!found) Serial.print(F(" nothing -- check PIN_OLED_SDA / PIN_OLED_SCL"));
  Serial.println();
  Wire.end();          // hand the pins back before software I2C bit-bangs them
}

static void oledBegin() {
  oledScanI2C();
  oled.begin();
  oled.setContrast(OLED_CONTRAST);
  oledClearGddram();
  oledSetComOffset(OLED_COM_OFFSET);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(OLED_X, 12, "MPU6881");
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 27, "waiting for");
  oled.drawStr(OLED_X, 37, "a packet...");
  oled.sendBuffer();
}

// ---- what the pages draw from -----------------------------------------------
// The newest packet, kept apart from the radio so a page can redraw the instant
// the button is pressed instead of waiting for the next one to arrive.
static struct {
  LinkPacket pkt;
  uint32_t   seen    = 0;
  uint16_t   lost    = 0;
  uint8_t    rssi    = RSSI_OFF;   // how loudly we hear the sender
  uint8_t    rssiAvg = 0;
  bool       stale   = true;
  uint32_t   startMs = 0;
  float      rate    = 0;          // packets per second, smoothed
} st;

// The 4-bit reading swings several counts between packets with nothing moving,
// so keep a short running mean alongside it.
static uint8_t rssiHist[8];
static uint8_t rssiCount = 0;
static uint8_t rssiPos   = 0;

static uint8_t rssiAverage(uint8_t sample) {
  rssiHist[rssiPos] = sample;
  rssiPos = (uint8_t)((rssiPos + 1) % 8);
  if (rssiCount < 8) rssiCount++;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < rssiCount; i++) sum += rssiHist[i];
  return (uint8_t)(sum / rssiCount);
}

// Open-circuit discharge curve of a single Li-ion cell. Voltage alone is a
// rough gauge -- it sags under load and recovers at rest -- so treat the
// percentage as an indication, not a fuel gauge.
static uint8_t batteryPercent(uint16_t mv) {
  static const uint16_t curve[][2] = {
    {4200, 100}, {4100, 90}, {4000, 80}, {3930, 70}, {3870, 60}, {3820, 50},
    {3790, 40}, {3770, 30}, {3740, 20}, {3680, 10}, {3450, 5}, {3000, 0},
  };
  const uint8_t n = sizeof(curve) / sizeof(curve[0]);

  if (mv >= curve[0][0]) return 100;
  if (mv <= curve[n - 1][0]) return 0;
  for (uint8_t i = 1; i < n; i++) {
    if (mv >= curve[i][0]) {
      uint16_t vHi = curve[i - 1][0], vLo = curve[i][0];
      uint16_t pHi = curve[i - 1][1], pLo = curve[i][1];
      return (uint8_t)(pLo + (long)(mv - vLo) * (pHi - pLo) / (vHi - vLo));
    }
  }
  return 0;
}

// One temperature sample per packet, one pixel column each.
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

// ---- pages ------------------------------------------------------------------
// The BOOT button cycles the screen. It is GPIO9, the C3's boot strapping pin:
// safe to read as an input once running, but holding it down through a reset
// drops the chip into download mode -- that is the button doing its day job,
// not a fault.
static const int PIN_BOOT = 9;

enum OledPage : uint8_t {
  PAGE_HORIZON, PAGE_ANGLES, PAGE_ACCEL, PAGE_GYRO,
  PAGE_TEMP, PAGE_BATT, PAGE_GRAPH, PAGE_LINK, PAGE_COUNT
};
static uint8_t oledPage = PAGE_HORIZON;

static void drawCentred(const char *s, int baseline, int charW) {
  oled.drawStr(OLED_X + (OLED_W - (int)strlen(s) * charW) / 2, baseline, s);
}

static void drawRight(const char *s, int baseline, int charW) {
  oled.drawStr(OLED_X + OLED_W - (int)strlen(s) * charW, baseline, s);
}

// Shown on every page that would otherwise be a screen of stale numbers.
static bool drawStaleNotice() {
  if (!st.stale) return false;
  oled.setFont(u8g2_font_6x10_tf);
  drawCentred("NO LINK", 22, 6);
  oled.setFont(u8g2_font_5x7_tf);
  drawCentred("waiting...", 34, 5);
  return true;
}

// An artificial horizon: the line is what the sender calls level, the fixed
// marker in the middle is the board you are holding. Roll tilts the line, pitch
// slides it up and down. It reads at a glance in a way two numbers never do.
static void pageHorizon() {
  char line[24];

  oled.setFont(u8g2_font_5x7_tf);
  snprintf(line, sizeof(line), "R%d", (int)lroundf(st.pkt.rollDeci / 10.0f));
  oled.drawStr(OLED_X, 7, line);
  snprintf(line, sizeof(line), "P%d", (int)lroundf(st.pkt.pitchDeci / 10.0f));
  drawRight(line, 7, 5);

  if (st.stale) {
    oled.setFont(u8g2_font_6x10_tf);
    drawCentred("NO LINK", 28, 6);
    return;
  }

  const int cx = OLED_X + OLED_W / 2;
  const int cy = 25;
  const float roll  = st.pkt.rollDeci  / (10.0f * RAD_TO_DEG);   // radians
  const float pitch = st.pkt.pitchDeci / 10.0f;                  // degrees

  // Half a pixel per degree, clamped so a board turned right over still leaves
  // the line somewhere on the glass instead of vanishing.
  float shift = pitch * 0.5f;
  if (shift >  20.0f) shift =  20.0f;
  if (shift < -20.0f) shift = -20.0f;

  const float half = OLED_W / 2.0f;
  float dx = cosf(roll) * half;
  float dy = sinf(roll) * half;

  int x1 = (int)lroundf(cx - dx), y1 = (int)lroundf(cy - dy + shift);
  int x2 = (int)lroundf(cx + dx), y2 = (int)lroundf(cy + dy + shift);
  oled.drawLine(x1, y1, x2, y2);

  // Fixed reference: two stubs and a centre dot, the way an attitude indicator
  // marks the aircraft itself.
  oled.drawHLine(cx - 12, cy, 7);
  oled.drawHLine(cx + 6,  cy, 7);
  oled.drawPixel(cx, cy);
}

static void pageAngles() {
  char line[16];

  if (drawStaleNotice()) return;

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 16, "R");
  oled.drawStr(OLED_X, 36, "P");

  oled.setFont(u8g2_font_logisoso16_tn);
  snprintf(line, sizeof(line), "%.1f", st.pkt.rollDeci / 10.0f);
  oled.drawStr(OLED_X + OLED_W - oled.getStrWidth(line), 17, line);
  snprintf(line, sizeof(line), "%.1f", st.pkt.pitchDeci / 10.0f);
  oled.drawStr(OLED_X + OLED_W - oled.getStrWidth(line), 37, line);
}

// Three axes, one per row, in the units a person thinks in.
static void drawAxisPage(const char *title, const char *unit,
                         float x, float y, float z, int decimals) {
  char line[20];

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, title);
  drawRight(unit, 7, 5);

  if (st.stale) {
    oled.setFont(u8g2_font_6x10_tf);
    drawCentred("NO LINK", 28, 6);
    return;
  }

  const float v[3] = {x, y, z};
  const char  axis[3] = {'X', 'Y', 'Z'};
  oled.setFont(u8g2_font_6x10_tf);
  for (uint8_t i = 0; i < 3; i++) {
    snprintf(line, sizeof(line), "%c%+.*f", axis[i], decimals, v[i]);
    oled.drawStr(OLED_X, 18 + i * 10, line);
  }
}

static void pageAccel() {
  drawAxisPage("ACCEL", "g", st.pkt.accMg[0] / ACC_MG_PER_G,
               st.pkt.accMg[1] / ACC_MG_PER_G,
               st.pkt.accMg[2] / ACC_MG_PER_G, 2);
}

static void pageGyro() {
  drawAxisPage("GYRO", "d/s", st.pkt.gyroDeciDps[0] / GYRO_DECI_PER_DPS,
               st.pkt.gyroDeciDps[1] / GYRO_DECI_PER_DPS,
               st.pkt.gyroDeciDps[2] / GYRO_DECI_PER_DPS, 1);
}

static void pageTemp() {
  char line[16];

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, "MPU TEMP");

  if (st.stale || st.pkt.tempDeciC == TEMP_NONE) {
    oled.setFont(u8g2_font_6x10_tf);
    drawCentred(st.stale ? "NO LINK" : "no data", 28, 6);
    return;
  }

  // logisoso20_tn is digits and punctuation only -- the degree mark and C go
  // next to it in the small font.
  snprintf(line, sizeof(line), "%.1f", st.pkt.tempDeciC / 10.0f);
  oled.setFont(u8g2_font_logisoso20_tn);
  int w = oled.getStrWidth(line);
  int x = OLED_X + (OLED_W - w - 8) / 2;
  oled.drawStr(x, 36, line);

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(x + w + 2, 22, "o");
  oled.drawStr(x + w + 2, 30, "C");
}

static void pageBattery() {
  char line[16];

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, "TX BATT");

  if (st.stale || st.pkt.battMv == BATT_NONE) {
    oled.setFont(u8g2_font_6x10_tf);
    drawCentred(st.stale ? "NO LINK" : "no sense", 28, 6);
    return;
  }

  uint8_t pct = batteryPercent(st.pkt.battMv);
  snprintf(line, sizeof(line), "%u%%", pct);
  drawRight(line, 7, 5);

  snprintf(line, sizeof(line), "%.2f", st.pkt.battMv / 1000.0f);
  oled.setFont(u8g2_font_logisoso16_tn);
  int w = oled.getStrWidth(line);
  oled.drawStr(OLED_X, 27, line);
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X + w + 2, 27, "V");

  // Battery outline with a nub on the right, filled by charge.
  const int bx = OLED_X, by = 31, bw = OLED_W - 3, bh = 9;
  oled.drawFrame(bx, by, bw, bh);
  oled.drawBox(bx + bw, by + 3, 2, 3);
  int fill = (bw - 2) * pct / 100;
  if (fill) oled.drawBox(bx + 1, by + 1, fill, bh - 2);
}

static void pageGraph() {
  char line[20];

  oled.setFont(u8g2_font_5x7_tf);
  if (tempHistCount == 0) snprintf(line, sizeof(line), "TEMP --");
  else snprintf(line, sizeof(line), "TEMP %.1f", st.pkt.tempDeciC / 10.0f);
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
  drawRight(line, 7, 5);
}

static void pageLink() {
  char line[24];
  uint32_t up = (millis() - st.startMs) / 1000;

  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 7, st.stale ? "NO LINK" : "LINK OK");
  snprintf(line, sizeof(line), "%.1f/s", st.stale ? 0.0f : st.rate);
  drawRight(line, 7, 5);

  // Signal bar.
  const int by = 10, bh = 11;
  oled.drawFrame(OLED_X, by, OLED_W, bh);
  if (!st.stale && st.rssi != RSSI_OFF) {
    uint8_t v = st.rssi > RSSI_FULL_SCALE ? RSSI_FULL_SCALE : st.rssi;
    int w = v * (OLED_W - 2) / RSSI_FULL_SCALE;
    if (w) oled.drawBox(OLED_X + 1, by + 2, w, bh - 4);
  }

  if (st.rssi == RSSI_OFF) snprintf(line, sizeof(line), "rssi off L%u", st.lost);
  else snprintf(line, sizeof(line), "s%u/%u L%u", st.rssi, st.rssiAvg, st.lost);
  oled.drawStr(OLED_X, 30, line);

  if (up < 3600) snprintf(line, sizeof(line), "rtx%u  %lum%02lus", st.pkt.retries,
                          (unsigned long)(up / 60), (unsigned long)(up % 60));
  else           snprintf(line, sizeof(line), "rtx%u  %luh%02lum", st.pkt.retries,
                          (unsigned long)(up / 3600), (unsigned long)((up / 60) % 60));
  oled.drawStr(OLED_X, 39, line);
}

static void oledDraw() {
  oled.clearBuffer();
  switch (oledPage) {
    case PAGE_ANGLES: pageAngles(); break;
    case PAGE_ACCEL:  pageAccel();  break;
    case PAGE_GYRO:   pageGyro();   break;
    case PAGE_TEMP:   pageTemp();   break;
    case PAGE_BATT:   pageBattery(); break;
    case PAGE_GRAPH:  pageGraph();  break;
    case PAGE_LINK:   pageLink();   break;
    default:          pageHorizon(); break;
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

// -----------------------------------------------------------------------------

static void haltWithDiagnostics() {
  Serial.println();
  Serial.println(F("SPI self-test FAILED -- the module is not answering on MISO."));
  Serial.println(F("Check, in this order:"));
  Serial.println(F("  1. 3V3 and GND actually reach the module"));
  Serial.println(F("  2. MOSI/MISO are not swapped, CLK and CSN on the right pins"));
  Serial.println(F("  3. wires are short (<10 cm)"));
  Serial.println(F("  4. a 10uF cap across the module 3V3/GND"));
  Serial.println(F("  5. the crystal is intact -- no 16MHz clock, no SPI answer"));
  radio.printDetails(Serial);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(OLED_X, 12, "SPI FAIL");
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(OLED_X, 27, "check wiring");
  oled.drawStr(OLED_X, 37, "see serial");
  oled.sendBuffer();

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
  Serial.println(F("  XN297L receiver  --  MPU6881 telemetry"));
  Serial.println(F("========================================"));

  oledBegin();
  pinMode(PIN_BOOT, INPUT_PULLUP);
  st.startMs = millis();
  st.pkt = LinkPacket{};

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) haltWithDiagnostics();
  Serial.println(F("SPI self-test PASSED -- the module is alive."));

  radio.setChannel(RF_CHANNEL);
  // The sender writes to this address; listen for it on pipe 1, the way RF24
  // does. Pipe 0 is kept for ACKs to our own writes, and this board never writes.
  radio.openReadingPipe(1, RF_ADDRESS);

  if (RSSI_GAIN >= 0) {
    radio.enableRSSI((xn297l_rssi_atten_e)RSSI_GAIN);
    Serial.print(F("RSSI attenuation code "));
    Serial.println(RSSI_GAIN);
  }

  radio.printDetails(Serial);
  Serial.println();
  radio.startListening();
}

void loop() {
  static uint32_t lastHeard  = 0;
  static uint32_t lastDraw   = 0;
  static uint32_t expected   = 0;
  static bool     started    = false;
  static uint32_t lastPktMs  = 0;
  static bool     dirty      = false;

  oledPollButton();          // polled every pass, so the page turns instantly

  if (radio.available()) {
    uint8_t rssiAtPacket = radio.getRSSI();
    LinkPacket in;
    radio.read(&in, sizeof(in));
    if (in.magic != LINK_MAGIC) return;   // noise, not one of ours

    uint32_t now = millis();
    st.seen++;
    lastHeard = now;

    // Gaps in the sender's counter are packets that never reached us. The first
    // packet only establishes where the sequence starts.
    if (started && in.counter > expected) st.lost += (uint16_t)(in.counter - expected);
    started  = true;
    expected = in.counter + 1;

    // Smoothed arrival rate: a single interval is too jittery to read off a
    // screen, and the average since boot would never show a link going bad.
    if (lastPktMs && now > lastPktMs) {
      float inst = 1000.0f / (now - lastPktMs);
      st.rate = st.rate ? (st.rate * 0.8f + inst * 0.2f) : inst;
    }
    lastPktMs = now;

    st.pkt   = in;
    st.stale = false;
    if (RSSI_GAIN >= 0) {
      st.rssi    = (uint8_t)(rssiAtPacket & 0x0F);
      st.rssiAvg = rssiAverage(st.rssi);
    }
    tempHistPush(in.tempDeciC);
    dirty = true;

    Serial.printf("#%lu R%+6.1f P%+6.1f  A%+6.2f%+6.2f%+6.2f  "
                  "G%+7.1f%+7.1f%+7.1f  T%.1fC  B%.3fV/%u%%  rssi%u  lost%u\n",
                  (unsigned long)in.counter,
                  in.rollDeci / 10.0f, in.pitchDeci / 10.0f,
                  in.accMg[0] / ACC_MG_PER_G, in.accMg[1] / ACC_MG_PER_G,
                  in.accMg[2] / ACC_MG_PER_G,
                  in.gyroDeciDps[0] / GYRO_DECI_PER_DPS,
                  in.gyroDeciDps[1] / GYRO_DECI_PER_DPS,
                  in.gyroDeciDps[2] / GYRO_DECI_PER_DPS,
                  in.tempDeciC / 10.0f,
                  in.battMv / 1000.0f, batteryPercent(in.battMv),
                  st.rssi, st.lost);
  }

  if (!st.stale && millis() - lastHeard > STALE_MS) {
    st.stale = true;
    st.rate  = 0;
    dirty    = true;
    Serial.println(F("...link lost"));
  }

  // Redrawing is the slowest thing this board does, so it happens on its own
  // clock and only when something actually changed.
  if (dirty && millis() - lastDraw >= REDRAW_MS) {
    lastDraw = millis();
    dirty = false;
    oledDraw();
  }
}
