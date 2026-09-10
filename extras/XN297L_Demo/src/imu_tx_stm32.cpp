/*
 * STM32F030K6T6 + MPU6881 + XN297L  --  wireless IMU sender
 * ---------------------------------------------------------
 * Samples the MPU6881 at 100 Hz, runs a complementary filter for roll and
 * pitch, and transmits one 32-byte packet every 100 ms to the ESP32-C3
 * receiver, which shows it on its 0.42" OLED.
 *
 * Wiring
 *   MPU6881            XN297L module        ST-Link
 *     SDA -> PB6         MOSI -> PA7          SWDIO -> PA13
 *     SCL -> PB7         MISO -> PA6          SWCLK -> PA14
 *     VCC -> 3V3         SCK  -> PA5          NRST, 3V3, GND
 *     GND -> GND         CE   -> PA4
 *     AD0 -> GND (0x68)  CSN  -> PB0
 *                        IRQ  -> not connected (the driver polls)
 *                        VDD  -> 3V3, GND -> GND
 *
 *   USB-TTL: PA2 = TX (into the adapter's RX), PA3 = RX, common GND, 115200.
 *
 *   Battery sense: a 1:2 divider from the cell to PA1,
 *
 *     BAT+ ---[ R1 100k ]---+--- PA1
 *                           |
 *                           +---[ R2 100k ]--- GND
 *                           |
 *                           +---[ 100nF ]----- GND
 *
 *   A full 4.2 V cell lands at 2.10 V, comfortably inside the 3.3 V the ADC
 *   can convert, and the pair draws about 21 uA.
 *
 * Power: the radio pulls about 66 mA in bursts at +11 dBm. Put 10 uF plus
 * 100 nF right across the module's own 3V3/GND pads, and if the link is flaky
 * or the MCU resets mid-burst, drop RF_POWER to XN297L_PA_4DBM first -- that is
 * the cheapest fix and costs almost nothing across a room.
 *
 * Note on I2C: the hardware I2C1 of the F030K6 is SCL=PB6 / SDA=PB7, i.e. the
 * opposite of this wiring. So the default here is a software (bit-bang) I2C
 * that matches the wiring as-is. If you swap the two wires you can set
 * USE_HW_I2C to 1 and use the hardware peripheral instead.
 */

#include <Arduino.h>
#include <SPI.h>
// VREFINT_CAL_ADDR lives here. The Arduino layer pulls in the HAL but not the
// LL headers, and this is the only thing from LL that this firmware needs.
#include <stm32f0xx_ll_adc.h>

#include <XN297L.h>
#include "link_packet.h"

#define USE_HW_I2C 0

#define PIN_SDA PB6
#define PIN_SCL PB7

// ---- radio pins (SPI1) ----
// PA5/PA6/PA7 are the only SPI1 pins on this package besides PB3/PB4/PB5, and
// they are all free: PB6/PB7 belong to the sensor, PA2/PA3 to the UART and
// PA13/PA14 to the debugger.
//
// CE and CSN are plain GPIO as far as the driver is concerned -- it toggles
// both by hand and never asks the SPI peripheral for a hardware chip select --
// so it does not matter that PA4 happens to be SPI1's NSS pin. These two match
// the board as it is wired.
static const int PIN_SCK  = PA5;
static const int PIN_MISO = PA6;
static const int PIN_MOSI = PA7;
static const int PIN_CE   = PA4;
static const int PIN_CSN  = PB0;
// IRQ stays unwired: the library polls STATUS.

// ---- battery sense ----
// PA0 and PA1 are the only ADC-capable pins this build leaves free. PA1 gets
// the divider; PA0 is deliberately left alone because it is the WKUP1 pin, and
// a divider parking it at 2.1 V would fight any later standby-and-wake work.
static const int PIN_BATT = PA1;

// The design ratio of the divider. Change only if you change the resistors.
static const float BATT_DIVIDER = 2.0f;

// Trim for the real resistors. A divider's error is pure gain, so one point
// corrects the whole range:
//   1. measure the cell with a multimeter
//   2. read what this board reports
//   3. BATT_CAL = multimeter / reported
//
// Measured on this board: meter 4.102 V, reported 3.860 V -> 4.102 / 3.860.
// That is 6%, more than 1% resistors could explain, so these are almost
// certainly 5% parts -- or not quite the values BATT_DIVIDER assumes. Either
// way the correction is the same, because both show up as gain.
static const float BATT_CAL = 1.0627f;

// Below this the cell is empty enough that the regulator is about to give up.
static const uint16_t BATT_EMPTY_MV = 3000;

// ---- link settings (must match the receiver) ----
static const uint8_t  RF_CHANNEL   = 78;                    // 2478 MHz
static uint8_t        RF_ADDRESS[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
static const xn297l_pa_dbm_e RF_POWER = XN297L_PA_11DBM;
// write() returns as soon as the radio reports ACK or MAX_RT; 15 retries at
// 750 us come to under 20 ms.

// ---- timing ----
// The filter wants a fast, even sample rate; the screen does not. So sample at
// 100 Hz and send every tenth sample. The radio then sits idle 9 ticks out of
// 10, which is also what keeps the supply quiet enough for the sensor.
static const uint32_t SAMPLE_MS = 10;
static const uint8_t  SEND_EVERY = 10;       // -> 10 packets per second
static const uint8_t  PRINT_EVERY = 10;      // serial lines, in packets

// Complementary filter weight for the gyro. 0.98 at 100 Hz gives a time
// constant of about half a second: quick moves follow the gyro, and the
// accelerometer pulls the estimate back to level over the following second.
static const float FILTER_ALPHA = 0.98f;

// ----- MPU6881 / MPU6500 registers -----
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

// Selected ranges: +/-2g and +/-250 deg/s
static const float ACCEL_LSB_PER_G  = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;

static uint8_t mpuAddr = 0x68;

#if USE_HW_I2C
// ============ hardware I2C (needs SCL=PB6, SDA=PB7) ============
#include <Wire.h>

static void i2cBegin() {
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  Wire.setClock(400000);
}

static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

#else
// ============ software I2C (bit-bang) on PB6 / PB7 ============
// A line is released high by the internal pull-up and pulled low by
// switching the pin to output. External 4.7k pull-ups to 3V3 are still
// recommended; most MPU breakout boards already have them.

static const uint16_t I2C_HALF_PERIOD_US = 5;  // roughly 100 kHz
static const uint16_t I2C_TIMEOUT_LOOPS  = 2000;

static inline void sdaHigh() { pinMode(PIN_SDA, INPUT_PULLUP); }
static inline void sdaLow()  { digitalWrite(PIN_SDA, LOW); pinMode(PIN_SDA, OUTPUT); }
static inline void sclLow()  { digitalWrite(PIN_SCL, LOW); pinMode(PIN_SCL, OUTPUT); }
static inline int  sdaRead() { return digitalRead(PIN_SDA); }

// Release SCL and wait for the line to actually rise (clock stretching
// and the slow rise time of a weak pull-up).
static void sclHigh() {
  pinMode(PIN_SCL, INPUT_PULLUP);
  for (uint16_t i = 0; i < I2C_TIMEOUT_LOOPS; i++) {
    if (digitalRead(PIN_SCL) == HIGH) return;
  }
}

static void i2cBegin() {
  digitalWrite(PIN_SDA, LOW);
  digitalWrite(PIN_SCL, LOW);
  sdaHigh();
  sclHigh();
  delayMicroseconds(10);
}

static void i2cStart() {
  sdaHigh();
  sclHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sdaLow();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sclLow();
  delayMicroseconds(I2C_HALF_PERIOD_US);
}

static void i2cStop() {
  sdaLow();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sclHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sdaHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
}

// Returns true if the slave acknowledged.
static bool i2cWriteByte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80) sdaHigh(); else sdaLow();
    data <<= 1;
    delayMicroseconds(I2C_HALF_PERIOD_US);
    sclHigh();
    delayMicroseconds(I2C_HALF_PERIOD_US);
    sclLow();
    delayMicroseconds(I2C_HALF_PERIOD_US);
  }
  sdaHigh();  // release the line to sample ACK
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sclHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  bool ack = (sdaRead() == LOW);
  sclLow();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  return ack;
}

static uint8_t i2cReadByte(bool ack) {
  uint8_t data = 0;
  sdaHigh();
  for (uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(I2C_HALF_PERIOD_US);
    sclHigh();
    delayMicroseconds(I2C_HALF_PERIOD_US);
    data = (uint8_t)((data << 1) | (sdaRead() == HIGH ? 1 : 0));
    sclLow();
  }
  if (ack) sdaLow(); else sdaHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sclHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  sclLow();
  sdaHigh();
  delayMicroseconds(I2C_HALF_PERIOD_US);
  return data;
}

static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  i2cStart();
  bool ok = i2cWriteByte((uint8_t)(addr << 1));
  ok = ok && i2cWriteByte(reg);
  ok = ok && i2cWriteByte(val);
  i2cStop();
  return ok;
}

static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  i2cStart();
  if (!i2cWriteByte((uint8_t)(addr << 1)) || !i2cWriteByte(reg)) {
    i2cStop();
    return false;
  }
  i2cStart();  // repeated start
  if (!i2cWriteByte((uint8_t)((addr << 1) | 1))) {
    i2cStop();
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = i2cReadByte(i < (len - 1));  // NACK on the last byte
  }
  i2cStop();
  return true;
}

static bool i2cPing(uint8_t addr) {
  i2cStart();
  bool ack = i2cWriteByte((uint8_t)(addr << 1));
  i2cStop();
  return ack;
}
#endif

// ===================== MPU6881 =====================

static void i2cScan() {
  Serial.println(F("I2C scan..."));
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (i2cPing(a)) {
      Serial.print(F("  device at 0x"));
      Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) {
    Serial.println(F("  nothing found - check wiring, 3V3 and pull-ups"));
  }
}

static bool mpuInit() {
  if (i2cPing(0x68))      mpuAddr = 0x68;
  else if (i2cPing(0x69)) mpuAddr = 0x69;
  else                    return false;

  i2cWriteReg(mpuAddr, REG_PWR_MGMT_1, 0x80);    // reset
  delay(100);
  i2cWriteReg(mpuAddr, REG_PWR_MGMT_1, 0x01);    // wake up, clock from PLL
  delay(50);
  i2cWriteReg(mpuAddr, REG_PWR_MGMT_2, 0x00);    // all axes enabled
  i2cWriteReg(mpuAddr, REG_CONFIG, 0x03);        // gyro DLPF 41 Hz
  i2cWriteReg(mpuAddr, REG_SMPLRT_DIV, 0x04);    // 1000/(1+4) = 200 Hz
  i2cWriteReg(mpuAddr, REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s
  i2cWriteReg(mpuAddr, REG_ACCEL_CONFIG, 0x00);  // +/-2g
  i2cWriteReg(mpuAddr, REG_ACCEL_CONFIG2, 0x03); // accel DLPF 41 Hz
  delay(50);
  return true;
}

// One full sample set, already scaled.
struct Sample {
  float ax, ay, az;      // g
  float gx, gy, gz;      // deg/s, bias removed
  float tempC;
};

// Gyro zero offset, measured at startup. Raw LSB, subtracted before scaling.
static float gyroBias[3] = {0, 0, 0};

static bool mpuRead(Sample &s) {
  uint8_t raw[14];
  if (!i2cReadRegs(mpuAddr, REG_ACCEL_XOUT_H, raw, 14)) return false;

  int16_t axr = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ayr = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t azr = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t tr  = (int16_t)((raw[6] << 8) | raw[7]);
  int16_t gxr = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gyr = (int16_t)((raw[10] << 8) | raw[11]);
  int16_t gzr = (int16_t)((raw[12] << 8) | raw[13]);

  s.ax = axr / ACCEL_LSB_PER_G;
  s.ay = ayr / ACCEL_LSB_PER_G;
  s.az = azr / ACCEL_LSB_PER_G;
  s.gx = (gxr - gyroBias[0]) / GYRO_LSB_PER_DPS;
  s.gy = (gyr - gyroBias[1]) / GYRO_LSB_PER_DPS;
  s.gz = (gzr - gyroBias[2]) / GYRO_LSB_PER_DPS;
  s.tempC = tr / 333.87f + 21.0f;   // MPU6500/6881 formula
  return true;
}

// Averages the gyro at rest and keeps the result as the zero point. Without
// this the integrated angle walks away by degrees a minute -- an uncalibrated
// MPU6881 typically sits a few LSB off zero on every axis.
//
// Whatever the board is lying on during these two seconds becomes "still", so
// do not pick it up until the banner says the calibration is done.
static void calibrateGyro() {
  const uint16_t N = 200;
  float sum[3] = {0, 0, 0};
  uint16_t taken = 0;

  Serial.print(F("calibrating gyro, keep the board still"));
  for (uint16_t i = 0; i < N; i++) {
    uint8_t raw[14];
    if (i2cReadRegs(mpuAddr, REG_ACCEL_XOUT_H, raw, 14)) {
      sum[0] += (int16_t)((raw[8] << 8) | raw[9]);
      sum[1] += (int16_t)((raw[10] << 8) | raw[11]);
      sum[2] += (int16_t)((raw[12] << 8) | raw[13]);
      taken++;
    }
    if ((i % 40) == 0) Serial.print('.');
    delay(10);
  }
  Serial.println();

  if (!taken) return;
  for (uint8_t k = 0; k < 3; k++) gyroBias[k] = sum[k] / taken;

  Serial.print(F("gyro bias [LSB]: "));
  Serial.print(gyroBias[0], 1); Serial.print(' ');
  Serial.print(gyroBias[1], 1); Serial.print(' ');
  Serial.println(gyroBias[2], 1);
}

// ===================== battery =====================
//
// The ADC is driven through its registers rather than through analogRead().
// Not for speed -- one reading a tenth of a second could hardly care -- but
// because analogRead() drags in the HAL's ADC module, and that measured out at
// about 2.9 KB. On a 32 KB part that is a tenth of the entire budget spent on
// a single-channel conversion. Everything below is RM0360 chapter 13.

// A bit per channel. PA1 is ADC_IN1; VREFINT is channel 17.
static const uint32_t ADC_CH_BATT    = (1UL << 1);
static const uint32_t ADC_CH_VREFINT = (1UL << 17);

static void adcBegin() {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Self-calibration is only legal with the ADC disabled. It costs about
  // 100 us once and takes most of the offset error out of every reading after.
  if (ADC1->CR & ADC_CR_ADEN) {
    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN) {}
  }
  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL) {}

  ADC1->CFGR1 = 0;                      // 12-bit, single conversion, software
  // CKMODE 10 is PCLK/4, i.e. 12 MHz here. The F0's ADC is only specified to
  // 14 MHz, so PCLK/2 would be over the limit.
  ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;
  // 111 is 239.5 ADC clocks, about 20 us. The divider is a 50 kOhm source and
  // the sample-and-hold charges through it; the short settings assume a stiff
  // source and would read low.
  ADC1->SMPR  = 7;

  ADC->CCR |= ADC_CCR_VREFEN;           // switch the reference channel in

  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
  delayMicroseconds(20);                // VREFINT start-up
}

static uint16_t adcRead(uint32_t chMask) {
  ADC1->CHSELR = chMask;
  ADC1->CR |= ADC_CR_ADSTART;
  while (!(ADC1->ISR & ADC_ISR_EOC)) {}
  return (uint16_t)ADC1->DR;            // reading DR clears EOC
}

// The ADC measures against VDDA, so a reading is only ever as good as the
// assumption made about that rail. VREFINT is an on-chip band-gap the factory
// measured at exactly 3.3 V and stored the raw result for, so reading it back
// says what VDDA is right now -- worth having precisely when the cell is low
// and the regulator has started to drop out. Assuming a flat 3.3 V there would
// report a battery healthier than it is.
static uint32_t readVddaMv() {
  uint32_t raw = adcRead(ADC_CH_VREFINT);
  if (!raw) return 3300;
  return (3300UL * (uint32_t)(*VREFINT_CAL_ADDR)) / raw;
}

static uint16_t readBatteryMv() {
  // A single sample wanders a few counts, and averaging is free here.
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += adcRead(ADC_CH_BATT);
  float counts = sum / 16.0f;

  float mv = counts * readVddaMv() / 4095.0f * BATT_DIVIDER * BATT_CAL;
  return (uint16_t)lroundf(mv);
}

// ===================== attitude =====================

static float roll = 0, pitch = 0;    // degrees

// Gravity alone fixes roll and pitch (not heading -- yaw needs a magnetometer,
// which this part does not have). The accelerometer is right on average but
// noisy and fooled by any real acceleration; the gyro is smooth but drifts.
// Blending them keeps the good half of each.
static void updateAttitude(const Sample &s, float dt) {
  float rollAcc  = atan2f(s.ay, s.az) * RAD_TO_DEG;
  float pitchAcc = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;

  roll  = FILTER_ALPHA * (roll  + s.gx * dt) + (1.0f - FILTER_ALPHA) * rollAcc;
  pitch = FILTER_ALPHA * (pitch + s.gy * dt) + (1.0f - FILTER_ALPHA) * pitchAcc;
}

// ===================== radio =====================

static XN297L radio(PIN_CE, PIN_CSN);

static void haltWithDiagnostics() {
  Serial.println();
  Serial.println(F("SPI self-test FAILED - the XN297L is not answering on MISO."));
  Serial.println(F("Check, in this order:"));
  Serial.println(F("  1. 3V3 and GND actually reach the module"));
  Serial.println(F("  2. MOSI/MISO not swapped, SCK and CSN on the right pins"));
  Serial.println(F("  3. wires short (<10 cm), 10uF across the module supply"));
  Serial.println(F("  4. the 16 MHz crystal is intact"));
#ifndef XN297L_NO_DETAILS
  radio.printDetails(Serial);
#endif
  while (1) {
    delay(2000);
    Serial.println(F("halted - fix the wiring and reset"));
  }
}

static int16_t clamp16(float v) {
  if (v >  32767.0f) return  32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

// ===================== setup / loop =====================

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println(F("=== STM32F030K6T6 + MPU6881 + XN297L : sender ==="));
  Serial.print(F("SysClk (Hz): "));
  Serial.println(F_CPU);
  Serial.println(F("SDA=PB6 SCL=PB7 | SCK=PA5 MISO=PA6 MOSI=PA7 CE=PA4 CSN=PB0"));

  // ---- sensor ----
  i2cBegin();
  delay(50);
  i2cScan();

  if (!mpuInit()) {
    Serial.println(F("!! sensor did not answer, stopping."));
    while (1) delay(1000);
  }

  uint8_t who = 0;
  i2cReadRegs(mpuAddr, REG_WHO_AM_I, &who, 1);
  Serial.print(F("sensor at 0x"));
  Serial.print(mpuAddr, HEX);
  Serial.print(F("  WHO_AM_I: 0x"));
  Serial.println(who, HEX);

  calibrateGyro();

  // Start the filter from the accelerometer instead of from zero, so the first
  // packet is already right rather than converging over the first second.
  Sample s;
  if (mpuRead(s)) {
    roll  = atan2f(s.ay, s.az) * RAD_TO_DEG;
    pitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;
  }

  // ---- battery ----
  pinMode(PIN_BATT, INPUT_ANALOG);
  adcBegin();
  Serial.print(F("VDDA: "));
  Serial.print(readVddaMv());
  Serial.print(F(" mV   battery: "));
  Serial.print(readBatteryMv());
  Serial.println(F(" mV"));

  // ---- radio ----
  // STM32duino picks the SPI pins before begin(), unlike the ESP32 core which
  // takes them as begin() arguments.
  SPI.setSCLK(PIN_SCK);
  SPI.setMISO(PIN_MISO);
  SPI.setMOSI(PIN_MOSI);
  SPI.begin();

  if (!radio.begin(&SPI)) haltWithDiagnostics();
  Serial.println(F("XN297L self-test PASSED"));

  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF_POWER);          // 1 Mbps is the library default
  radio.openWritingPipe(RF_ADDRESS);   // the receiver listens for it on pipe 1
  radio.stopListening();

  Serial.println(F("sending 10 packets/s"));
  Serial.println();
}

void loop() {
  static uint32_t nextSample = 0;
  static uint32_t lastMicros = 0;
  static uint8_t  tick = 0;
  static uint32_t counter = 0;
  static uint32_t acked = 0;
  static uint8_t  lastRetries = 0;
  static Sample   s;

  uint32_t now = millis();
  if ((int32_t)(now - nextSample) < 0) return;
  nextSample = now + SAMPLE_MS;

  uint32_t us = micros();
  float dt = lastMicros ? (us - lastMicros) / 1000000.0f : SAMPLE_MS / 1000.0f;
  lastMicros = us;
  // A missed tick, or the ~10 ms a transmit takes, must not be integrated as
  // if it were a normal step.
  if (dt > 0.1f) dt = 0.1f;

  if (!mpuRead(s)) {
    Serial.println(F("I2C read error"));
    return;
  }
  updateAttitude(s, dt);

  if (++tick < SEND_EVERY) return;
  tick = 0;

  LinkPacket p;
  p.magic     = LINK_MAGIC;
  p.counter   = counter;
  p.lost      = 0;
  p.retries   = lastRetries;
  p.rssi      = RSSI_OFF;          // this board never measures its own
  p.tempDeciC = clamp16(s.tempC * 10.0f);
  p.battMv    = readBatteryMv();

  p.accMg[0] = clamp16(s.ax * ACC_MG_PER_G);
  p.accMg[1] = clamp16(s.ay * ACC_MG_PER_G);
  p.accMg[2] = clamp16(s.az * ACC_MG_PER_G);
  p.gyroDeciDps[0] = clamp16(s.gx * GYRO_DECI_PER_DPS);
  p.gyroDeciDps[1] = clamp16(s.gy * GYRO_DECI_PER_DPS);
  p.gyroDeciDps[2] = clamp16(s.gz * GYRO_DECI_PER_DPS);
  p.rollDeci  = clamp16(roll  * ANGLE_DECI_PER_DEG);
  p.pitchDeci = clamp16(pitch * ANGLE_DECI_PER_DEG);

  bool ok = radio.write(&p, sizeof(p));
  if (ok) acked++;
  // ARC_CNT is how many retransmits the packet we just sent needed. It is only
  // readable after the fact, so it rides out with the next packet.
  lastRetries = radio.getARC();
  counter++;

  if ((counter % PRINT_EVERY) == 0) {
    Serial.print(F("#")); Serial.print(counter);
    Serial.print(F("  R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    Serial.print(F("  T:")); Serial.print(s.tempC, 1);
    Serial.print(F("C  acked ")); Serial.print(acked);
    Serial.print('/'); Serial.print(counter);
    Serial.print(F("  retries ")); Serial.println(lastRetries);
  }
}
