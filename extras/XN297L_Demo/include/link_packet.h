// -----------------------------------------------------------------------------
// What travels over the air between the STM32F030 sender and the ESP32-C3
// receiver. Both sides include this file, so there is exactly one definition of
// the wire format and no chance of the two drifting apart.
//
// The XN297L runs fixed 32-byte payloads here, so the packet is sized to fill
// one exactly: 16 bytes of link bookkeeping, inherited unchanged from the
// original amir684/XN297L demo, and 16 bytes of MPU6881 data in the space that
// demo left empty. The first half is byte-for-byte identical to the original
// LinkPacket, so the stock rx / rx_c3 firmware from that repo still decodes
// this packet -- it just ignores the sensor half.
//
// Both ends are little-endian ARM and every field is naturally aligned, so the
// struct is laid out identically on the Cortex-M0 and the RISC-V C3. The
// static_assert below is what keeps that true if anyone edits the fields.
// -----------------------------------------------------------------------------
#pragma once

#include <stdint.h>

// Leads every packet so the receiver can tell one of ours from noise that
// merely managed to clock into the FIFO. Same value as the original demo.
static const uint32_t LINK_MAGIC = 0x4C4E5837UL;

// "this board is not measuring", as opposed to a reading that happens to be 0.
static const uint8_t  RSSI_OFF   = 0xFF;
static const int16_t  TEMP_NONE  = INT16_MIN;
static const uint16_t BATT_NONE  = 0;

// Fixed-point scales. Integers keep the packet small and cost the receiver one
// divide when it prints; floats over the air would buy nothing here.
//   accel  milli-g          +/-2g      -> +/-2000
//   gyro   tenths of deg/s  +/-250dps  -> +/-2500
//   angle  tenths of degree            -> +/-1800
static const float ACC_MG_PER_G     = 1000.0f;
static const float GYRO_DECI_PER_DPS = 10.0f;
static const float ANGLE_DECI_PER_DEG = 10.0f;

struct LinkPacket {
  // ---- link bookkeeping: the original 16 bytes, unchanged ----
  uint32_t magic;      // LINK_MAGIC
  uint32_t counter;    // sender's packet number; gaps are lost packets
  uint16_t lost;       // unused by this sender, kept so the layout matches
  uint8_t  retries;    // ARC_CNT the sender needed for its previous packet
  uint8_t  rssi;       // RSSI_OFF here: the STM32 never measures its own
  int16_t  tempDeciC;  // MPU6881 die temperature, tenths of a degree
  uint16_t battMv;     // cell voltage through the sender's 1:2 divider

  // ---- MPU6881, the 16 bytes the original packet left unused ----
  int16_t  accMg[3];        // X, Y, Z in milli-g
  int16_t  gyroDeciDps[3];  // X, Y, Z in tenths of a degree per second
  int16_t  rollDeci;        // complementary-filter roll, tenths of a degree
  int16_t  pitchDeci;       // complementary-filter pitch, tenths of a degree
};

#ifdef __cplusplus
static_assert(sizeof(LinkPacket) == 32,
              "LinkPacket must fill exactly one 32-byte XN297L payload");
#endif
