/*
 * STM32F030K6T6 + MPU6881 (Register map compatible with MPU6500)
 * -------------------------------------------------------------
 * Wiring:
 *   MPU SDA -> PB6
 *   MPU SCL -> PB7
 *   MPU VCC -> 3V3   (3.3V logic only)
 *   MPU GND -> GND
 *   MPU AD0 -> GND = address 0x68, 3V3 = address 0x69 (auto detected)
 *
 * Serial output: USART1, TX = PA2, RX = PA3, 115200 8N1
 *   PA2 -> RX pin of a USB-TTL adapter, common GND.
 *
 * Note: the hardware I2C1 of the F030K6 is SCL=PB6 / SDA=PB7, i.e. the
 * opposite of this wiring. So the default here is a software (bit-bang)
 * I2C that matches the wiring as-is. If you swap the two wires you can
 * set USE_HW_I2C to 1 and use the hardware peripheral instead.
 */

#include <Arduino.h>

#define USE_HW_I2C 0

#define PIN_SDA PB6
#define PIN_SCL PB7

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

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println(F("=== STM32F030K6T6 + MPU6881 ==="));
  Serial.print(F("SysClk (Hz): "));
  Serial.println(F_CPU);
  Serial.println(F("SDA=PB6  SCL=PB7  UART TX=PA2 RX=PA3"));

  i2cBegin();
  delay(50);
  i2cScan();

  if (!mpuInit()) {
    Serial.println(F("!! sensor did not answer, stopping."));
    while (1) {
      delay(1000);
    }
  }

  uint8_t who = 0;
  i2cReadRegs(mpuAddr, REG_WHO_AM_I, &who, 1);
  Serial.print(F("sensor address: 0x"));
  Serial.print(mpuAddr, HEX);
  Serial.print(F("   WHO_AM_I: 0x"));
  Serial.println(who, HEX);
  Serial.println(F("(MPU6881/6500 usually reports 0x70, MPU6050 reports 0x68)"));
  Serial.println();
}

void loop() {
  uint8_t raw[14];

  if (!i2cReadRegs(mpuAddr, REG_ACCEL_XOUT_H, raw, 14)) {
    Serial.println(F("I2C read error"));
    delay(500);
    return;
  }

  int16_t axr = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ayr = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t azr = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t tr  = (int16_t)((raw[6] << 8) | raw[7]);
  int16_t gxr = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gyr = (int16_t)((raw[10] << 8) | raw[11]);
  int16_t gzr = (int16_t)((raw[12] << 8) | raw[13]);

  float ax = axr / ACCEL_LSB_PER_G;
  float ay = ayr / ACCEL_LSB_PER_G;
  float az = azr / ACCEL_LSB_PER_G;
  float gx = gxr / GYRO_LSB_PER_DPS;
  float gy = gyr / GYRO_LSB_PER_DPS;
  float gz = gzr / GYRO_LSB_PER_DPS;
  float tempC = tr / 333.87f + 21.0f;  // MPU6500/6881 formula

  Serial.print(F("A[g]: "));
  Serial.print(ax, 3); Serial.print(F("  "));
  Serial.print(ay, 3); Serial.print(F("  "));
  Serial.print(az, 3);
  Serial.print(F("  |  G[dps]: "));
  Serial.print(gx, 2); Serial.print(F("  "));
  Serial.print(gy, 2); Serial.print(F("  "));
  Serial.print(gz, 2);
  Serial.print(F("  |  T[C]: "));
  Serial.println(tempC, 2);

  delay(200);
}
