#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/util/typed_param.hpp"
#include "motor_control/key_input_thread.hpp"
#include "motor_control_interfaces/msg/target_angle.hpp"

using namespace std::chrono_literals;

class AngleControl : public rclcpp::Node
{
public:
  AngleControl() : Node("angle_control")
  {
    msg_target_angle_.target_angle = 0.0;

    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&AngleControl::CbTimer, this));

    // パブリッシャー作成
    pub_target_angle_ = this->create_publisher<motor_control_interfaces::msg::TargetAngle>(
      "target_angle", rclcpp::QoS(10));

    // コールバックの登録
    param_angle_controller_.RegisterOnChangeCallback(
      [this](const alps::cmn::control::PidParam & value) {
        RCLCPP_INFO(
          this->get_logger(),
          "angle_controller_param: kp=%f, ki=%f, kd=%f, diff_lpf_time_const=%f",
          value.kp,
          value.ki,
          value.kd,
          value.diff_lpf_time_const.count());
      });

    RCLCPP_INFO(this->get_logger(), "Start AngleControl");
  }

  void CbTimer()
  {
    msg_target_angle_.target_angle = key_input_thread_.GetInputVal();
    pub_target_angle_->publish(msg_target_angle_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_input_;
  motor_control_interfaces::msg::TargetAngle msg_target_angle_;
  KeyInputThread key_input_thread_{*this, "Target Angle [deg]"};
  rclcpp::Publisher<motor_control_interfaces::msg::TargetAngle>::SharedPtr pub_target_angle_;
  alps::ros2::util::TypedParamServer<alps::cmn::control::PidParam> param_angle_controller_{
    *this, "angle_controller"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleControl>());
  rclcpp::shutdown();
  return 0;
}
