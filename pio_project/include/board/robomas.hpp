#ifndef NHK2024_BOARD_ROBOMAS_HPP
#define NHK2024_BOARD_ROBOMAS_HPP

#ifdef STM32F446xx

#include "mbed.h"

namespace nhk2024
{
namespace board
{

/// @brief 足回り基盤のピン定義
struct Robomas  // ボード名は仮
{
  // アナログ入力 ---------------------------
  static constexpr PinName kA0 = PA_0;
  static constexpr PinName kA1 = PA_1;
  static constexpr PinName kA2 = PA_4;
  static constexpr PinName kA3 = PB_0;
  static constexpr PinName kA4 = PC_1;
  static constexpr PinName kA5 = PC_0;
  static constexpr PinName kA6 = PC_2;
  static constexpr PinName kA7 = PC_3;

  // デジタル入力 ---------------------------
  static constexpr PinName kD0 = PA_10;
  static constexpr PinName kD1 = PA_9;
  static constexpr PinName kD2 = PC_7;
  static constexpr PinName kD3 = PA_7;
  static constexpr PinName kD4 = PA_6;
  static constexpr PinName kD5 = PB_1;
  static constexpr PinName kD6 = PB_2;
  static constexpr PinName kD7 = PB_15;

  // I2C ------------------------------------
  static constexpr PinName kI2c1Sda = PC_12;
  static constexpr PinName kI2c1Scl = PB_10;
  static constexpr PinName kI2c2Sda = PB_4;
  static constexpr PinName kI2c2Scl = PA_8;

  // CAN ------------------------------------
  static constexpr PinName kCan1Rd = PB_8;
  static constexpr PinName kCan1Td = PB_9;
  static constexpr PinName kCan2Rd = PB_5;
  static constexpr PinName kCan2Td = PB_13;
};

}  // namespace board
}  // namespace nhk2024

#endif

#endif  // NHK2024_BOARD_ROBOMAS_HPP