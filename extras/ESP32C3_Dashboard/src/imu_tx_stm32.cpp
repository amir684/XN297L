// The STM32F030 IMU link's sender, built from this project so that its env sits
// in the same PlatformIO sidebar as the dashboard's. PlatformIO has no per-env
// source directory, so this file pulls the real source in from its own project
// rather than copying it -- there is still exactly one sender.
//
// Selected by the imu_tx_stm32 env's build_src_filter; every other env skips it.
#include "../../STM32F030_IMU_Link/src/main_tx_stm32.cpp"
