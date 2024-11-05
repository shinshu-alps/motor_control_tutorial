#include <memory>

#include "alps_lin/communication/socket_can_io.hpp"
#include "alps_ros2/communication/can_param_bridge.hpp"
#include "alps_ros2/communication/can_topic_bridge.hpp"
#include "alps_ros2/log/log_writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

using String = etl::string<64>;

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
  alps::cmn::communication::CanParamPort<20> param_port_{transceiver_, 0x100};

  // CANトピックブリッジ : ROS -> CAN
  alps::ros2::communication::RosToCanTopicBridge<int32_t, std_msgs::msg::Int32>
    ros_to_can_topic_bridge_int32_{transceiver_, *this, 0x001, "int32_topic"};

  // CANトピックブリッジ : CAN -> ROS
  alps::ros2::communication::CanToRosTopicBridge<double, std_msgs::msg::Float64>
    can_to_ros_topic_bridge_double_{transceiver_, *this, 0x002, "float64_topic"};

  // CANパラメータブリッジ : CAN -> ROS
  alps::ros2::communication::RosToCanParamBridge<String> ros_to_can_param_bridge_string_{
    transceiver_, param_port_, *this, 0x010, "can_ros_bridge_ros_target", "string_param"};
};

int main(int argc, char * argv[])
{
  alps::ros2::log::SetDefaultLogWriter();  // ログ出力関数の設定

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRosBridge>());
  rclcpp::shutdown();
  return 0;
}
