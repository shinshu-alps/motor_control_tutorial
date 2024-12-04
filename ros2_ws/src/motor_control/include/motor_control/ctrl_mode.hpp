#ifndef MOTOR_CONTROL_CTRL_MODE_HPP_
#define MOTOR_CONTROL_CTRL_MODE_HPP_

#include <cstdint>

enum class CtrlMode : uint8_t {
  kNone,
  kAngle,
  kVelocity,
  kAngleVelocity,
};

struct CtrlModeFlag
{
  bool angle_mode = false;
  bool velocity_mode = false;
  bool angle_velocity_mode = false;
};
auto ToTiedTuple(CtrlModeFlag & flag)
{
  return std::tie(flag.angle_mode, flag.velocity_mode, flag.angle_velocity_mode);
}
auto GetElementNames(const CtrlModeFlag &)
{
  return std::array<const char *, 3>{"angle_mode", "velocity_mode", "angle_velocity_mode"};
}

#endif