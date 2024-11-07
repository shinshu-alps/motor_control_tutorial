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

using AngleControllerParam = alps::cmn::control::PidParam;
using AngleControllerCalcInfo = alps::cmn::control::MotorAngleControllerCalcInfo;

using VelocityControllerParam = alps::cmn::control::MotorVelocityControllerParam;
using VelocityControllerCalcInfo = alps::cmn::control::MotorVelocityControllerCalcInfo;
}  // namespace can_map