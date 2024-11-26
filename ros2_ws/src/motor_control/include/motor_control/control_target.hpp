#ifndef MOTOR_CONTROL_CONTROL_TARGET_HPP_
#define MOTOR_CONTROL_CONTROL_TARGET_HPP_

#include <tuple>

struct ControlTarget
{
  /// @brief 目標値
  float target;
  /// @brief フィードフォワード値
  float ff_value = 0.0f;
};

auto ToTiedTuple(ControlTarget & d)
{
  return std::tie(d.target, d.ff_value);
}

#endif
