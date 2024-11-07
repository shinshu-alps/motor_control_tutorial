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
#include "motor_control_interfaces/msg/target_angle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alps::ros2::type
{
// TopicConverterの特殊化
template <>
struct TopicConverter<double, motor_control_interfaces::msg::TargetAngle>
{
  static std::optional<motor_control_interfaces::msg::TargetAngle> ToTopic(
    const double & original_data)
  {
    motor_control_interfaces::msg::TargetAngle ros_topic_data;
    ros_topic_data.target_angle = original_data;
    return ros_topic_data;
  }

  static std::optional<double> ToOriginal(
    const motor_control_interfaces::msg::TargetAngle & ros_topic_data)
  {
    return ros_topic_data.target_angle;
  }
};
}  // namespace alps::ros2::type

using namespace std::chrono_literals;

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
  alps::ros2::communication::RosToCanTopicBridge<double, motor_control_interfaces::msg::TargetAngle>
    ros_to_can_topic_target_angle_{transceiver_, *this, can_map::kIdTargetAngle, "target_angle"};

  // CANトピックブリッジ : CAN -> ROS
  alps::ros2::communication::CanToRosTopicBridge<
    can_map::AngleControllerCalcInfo,
    alps_interfaces::msg::MotorAngleControllerCalcInfo>
    can_to_ros_topic_bridge_double_{
      transceiver_, *this, can_map::kIdAngleControllerCalcInfo, "calc_info/angle_controller"};

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
};

int main(int argc, char * argv[])
{
  alps::ros2::log::SetDefaultLogWriter();  // ログ出力関数の設定

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRosBridge>());
  rclcpp::shutdown();
  return 0;
}
