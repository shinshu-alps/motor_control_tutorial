#pragma once

#ifdef STM32F303x8

#include "mbed.h"

namespace nhk2024
{
namespace board
{

/// @brief モータードライバー基盤
struct MotorDriver
{
  // LED
  static constexpr PinName kLed1 = PA_4;

  // モーター出力
  static constexpr PinName kMotorPwm0 = PB_0;  // D3
  static constexpr PinName kMotorPwm1 = PA_7;  // A6
  static constexpr PinName kMotorPwm2 = PB_4;  // D12
  
  static constexpr PinName kMotorDir0 = PF_1;  // D8
  static constexpr PinName kMotorDir1 = PB_5;  // D11
  static constexpr PinName kMotorDir2 = PA_9;  // D1

  // エンコーダ
  static constexpr PinName kEnc0A = PB_7;  // D4
  static constexpr PinName kEnc0B = PB_6;  // D5
  static constexpr PinName kEnc1A = PB_1;  // D6
  static constexpr PinName kEnc1B = PF_0;  // D7
  static constexpr PinName kEnc2A = PA_0;  // A0
  static constexpr PinName kEnc2B = PA_3;  // A2

  // ポテンショメータ
  static constexpr PinName kPm0 = PA_0;
  static constexpr PinName kPm1 = PA_3;
  static constexpr PinName kPm2 = PA_5;

  // デバッグスイッチ
  static constexpr PinName kDebugSw = PA_10;

  // Enable
  static constexpr PinName kEnable = PA_1;

  // スイッチ
  static constexpr PinName kSw0 = PA_8;
  static constexpr PinName kSw1 = PB_7;
  static constexpr PinName kSw2 = PB_6;
  static constexpr PinName kSw3 = PB_1;
  static constexpr PinName kSw4 = PF_0;
  static constexpr PinName kSw5 = PA_5;

  // CAN
  static constexpr PinName kCanRd = PA_11;  // D10
  static constexpr PinName kCanTd = PA_12;  // D2
};

}  // namespace board
}  // namespace nhk2024

#endif
