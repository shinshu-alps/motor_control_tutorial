#include <memory>

#include "alps_cmn/type/custom_type_serialize_rule.hpp"
#include "alps_interfaces/msg/motor_angle_controller_calc_info.hpp"
#include "alps_lin/communication/socket_can_io.hpp"
#include "alps_ros2/communication/can_param_bridge.hpp"
#include "alps_ros2/communication/can_topic_bridge.hpp"
#include "alps_ros2/log/log_writer.hpp"
#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/type/custom_type_topic_conversion_rule.hpp"
#include "motor_control/can_map.hpp"
#include "motor_control/control_target.hpp"
#include "motor_control_interfaces/msg/angle.hpp"
#include "motor_control_interfaces/msg/control_target.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// CAN <-> ROSの変換のためのコード ---------------------------------------------------------
namespace motor_control_interfaces::msg
{
auto ToTiedTuple(motor_control_interfaces::msg::ControlTarget & msg)
{
  return std::tie(msg.target, msg.ff_value);
}
static_assert(
  alps::cmn::type::SupportsToTiedTuple<motor_control_interfaces::msg::ControlTarget>::value);
}  // namespace motor_control_interfaces::msg

namespace alps::ros2::type
{
template <>
struct TopicConverter<float, motor_control_interfaces::msg::Angle>
{
  static std::optional<motor_control_interfaces::msg::Angle> ToTopic(const float & orig_msg)
  {
    motor_control_interfaces::msg::Angle ros_msg;
    ros_msg.angle = orig_msg;
    return ros_msg;
  }
};
}  // namespace alps::ros2::type

// CanRosBridgeクラス ----------------------------------------------------------------------
class CanRosBridge : public rclcpp::Node
{
public:
  CanRosBridge() : Node("can_ros_bridge")
  {
    // タイマーの作成
    timer_ = this->create_wall_timer(10ms, std::bind(&CanRosBridge::TimerCallback, this));
  }

private:
  void TimerCallback()
  {
    // CAN受信データの更新（CANサブスクライバーのコールバック関数はここで呼び出される）
    transceiver_.ReadUpdate();

    // パラメータの更新処理
    param_port_.Update();
  }

  // 周期タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // CANの通信インスタンス
  alps::lin::communication::SocketCanIo can_io_{};
  alps::cmn::communication::CanTypedDataTransceiver<decltype(can_io_)> transceiver_{can_io_};
  alps::cmn::communication::CanParamPort<20> param_port_{transceiver_, can_map::kIdParamPort};

  // CANトピックブリッジ : ROS -> CAN
  alps::ros2::communication::
    RosToCanTopicBridge<ControlTarget, motor_control_interfaces::msg::ControlTarget>
      ros_to_can_topic_target_angle_{transceiver_, *this, can_map::kIdTargetAngle, "target/angle"};
  alps::ros2::communication::
    RosToCanTopicBridge<ControlTarget, motor_control_interfaces::msg::ControlTarget>
      ros_to_can_topic_target_velocity_{
        transceiver_, *this, can_map::kIdTargetVelocity, "target/velocity"};
  alps::ros2::communication::
    RosToCanTopicBridge<ControlTarget, motor_control_interfaces::msg::ControlTarget>
      ros_to_can_topic_target_angle_velocity_{
        transceiver_, *this, can_map::kIdTargetAngleVelocity, "target/angle_velocity"};

  // CANトピックブリッジ : CAN -> ROS
  alps::ros2::communication::CanToRosTopicBridge<
    can_map::AngleControllerCalcInfo,
    alps_interfaces::msg::MotorAngleControllerCalcInfo>
    can_to_ros_topic_bridge_angle_controller_calc_info_{
      transceiver_, *this, can_map::kIdAngleControllerCalcInfo, "calc_info/angle_controller"};
  alps::ros2::communication::CanToRosTopicBridge<
    can_map::VelocityControllerCalcInfo,
    alps_interfaces::msg::MotorVelocityControllerCalcInfo>
    can_to_ros_topic_bridge_velocity_controller_calc_info_{
      transceiver_, *this, can_map::kIdVelocityControllerCalcInfo, "calc_info/velocity_controller"};
  alps::ros2::communication::CanToRosTopicBridge<
    can_map::AngleVelocityControllerCalcInfo,
    alps_interfaces::msg::MotorAngleControllerCalcInfo>
    can_to_ros_topic_bridge_angle_velocity_controller_calc_info_{
      transceiver_,
      *this,
      can_map::kIdAngleVelocityControllerCalcInfo,
      "calc_info/angle_velocity_controller"};
  alps::ros2::communication::
    CanToRosTopicBridge<can_map::NowAngle, motor_control_interfaces::msg::Angle>
      can_to_ros_topic_bridge_now_angle_{transceiver_, *this, can_map::kIdNowAngle, "now_angle"};

  // CANパラメータブリッジ : CAN -> ROS
  alps::ros2::communication::RosToCanParamBridge<can_map::AngleControllerParam>
    ros_to_can_param_angle_controller_param_{
      transceiver_,
      param_port_,
      *this,
      can_map::kIdAngleControllerParam,
      "angle_control",          // サーバーノード名
      "angle_controller_param"  // パラメータ名
    };
  alps::ros2::communication::RosToCanParamBridge<can_map::VelocityControllerParam>
    ros_to_can_param_velocity_controller_param_{
      transceiver_,
      param_port_,
      *this,
      can_map::kIdVelocityControllerParam,
      "velocity_control",          // サーバーノード名
      "velocity_controller_param"  // パラメータ名
    };
  alps::ros2::communication::RosToCanParamBridge<can_map::AngleVelocityControllerParam>
    ros_to_can_param_angle_velocity_controller_param_{
      transceiver_,
      param_port_,
      *this,
      can_map::kIdAngleVelocityControllerParam,
      "angle_velocity_control",          // サーバーノード名
      "angle_velocity_controller_param"  // パラメータ名
    };
};

int main(int argc, char * argv[])
{
  alps::ros2::log::SetDefaultLogWriter();  // ログ出力関数の設定

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRosBridge>());
  rclcpp::shutdown();
  return 0;
}
