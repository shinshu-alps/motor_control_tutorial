#include "alps_cmn/util/angle.hpp"
#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/util/typed_param.hpp"
#include "motor_control/key_input_thread.hpp"
#include "motor_control_interfaces/msg/control_target.hpp"

using namespace std::chrono_literals;

class AngleControl : public rclcpp::Node
{
public:
  AngleControl() : Node("angle_control")
  {
    msg_target_angle_.target = 0.0;

    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&AngleControl::CbTimer, this));

    // パブリッシャー作成
    pub_target_angle_ =
      this->create_publisher<motor_control_interfaces::msg::ControlTarget>("target/angle", 1);

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
    param_target_angle_.RegisterOnChangeCallback(
      [this](const double & value) { CbTargetChanged(value); });

    RCLCPP_INFO(this->get_logger(), "Start AngleControl");
  }

private:
  void CbTimer()
  {
    pub_target_angle_->publish(msg_target_angle_);
  }

  void CbTargetChanged(const double & target_angle_deg)
  {
    double target_angle_rad = alps::cmn::util::DegToRad(target_angle_deg);
    RCLCPP_INFO(
      this->get_logger(),
      "target_angle: %f [deg] (= %f [rad])",
      target_angle_deg,
      target_angle_rad);
    msg_target_angle_.target = target_angle_rad;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_input_;
  motor_control_interfaces::msg::ControlTarget msg_target_angle_;
  rclcpp::Publisher<motor_control_interfaces::msg::ControlTarget>::SharedPtr pub_target_angle_;
  alps::ros2::util::TypedParamServer<double> param_target_angle_{*this, "target_angle_deg"};
  alps::ros2::util::TypedParamServer<alps::cmn::control::PidParam> param_angle_controller_{
    *this, "angle_controller_param"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleControl>());
  rclcpp::shutdown();
  return 0;
}
