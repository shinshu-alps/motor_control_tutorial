#ifndef MOTOR_CONTROL_CTRL_MODE_HPP_
#define MOTOR_CONTROL_CTRL_MODE_HPP_

#include <cstdint>

enum class CtrlMode : uint8_t {
  kNone,
  kAngle,
  kVelocity,
  kAngleVelocity,
};

#endif