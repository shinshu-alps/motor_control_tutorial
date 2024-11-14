#include "alps_cmn/communication/can_data_frame.hpp"
#include "alps_cmn/control/motor_angle_velocity_controller.hpp"

namespace can_map
{
using Id = alps::cmn::communication::CanDataFrame::Id;

constexpr Id kIdParamPort = 0x001;

constexpr Id kIdTargetAngle = 0x100;
constexpr Id kIdAngleControllerParam = 0x200;
constexpr Id kIdAngleControllerCalcInfo = 0x300;

constexpr Id kIdTargetVelocity = 0x101;
constexpr Id kIdVelocityControllerParam = 0x201;
constexpr Id kIdVelocityControllerCalcInfo = 0x301;

constexpr Id kIdTargetAngleVelocity = 0x102;
constexpr Id kIdAngleVelocityControllerParam = 0x202;
constexpr Id kIdAngleVelocityControllerCalcInfo = 0x302;

using AngleControllerParam = alps::cmn::control::PidParam;
using AngleControllerCalcInfo = alps::cmn::control::MotorAngleControllerCalcInfo;
using AngleVelocityControllerParam = alps::cmn::control::PidParam;

using VelocityControllerParam = alps::cmn::control::MotorVelocityControllerParam;
using VelocityControllerCalcInfo = alps::cmn::control::MotorVelocityControllerCalcInfo;
using AngleVelocityControllerCalcInfo = alps::cmn::control::MotorAngleControllerCalcInfo;
}  // namespace can_map
