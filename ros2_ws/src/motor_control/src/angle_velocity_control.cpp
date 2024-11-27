#include "alps_cmn/util/angle.hpp"
#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/util/typed_param.hpp"
#include "motor_control/key_input_thread.hpp"
#include "motor_control_interfaces/msg/control_target.hpp"

using namespace std::chrono_literals;

class AngleVelocityControl : public rclcpp::Node
{
public:
  AngleVelocityControl() : Node("angle_velocity_control")
  {
    msg_target_angle_velocity_.target = 0.0;

    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&AngleVelocityControl::CbTimer, this));

    // パブリッシャー作成
    pub_target_angle_velocity_ =
      this->create_publisher<motor_control_interfaces::msg::ControlTarget>(
        "target/angle_velocity", 1);

    // コールバックの登録
    param_velocity_controller_.RegisterOnChangeCallback(
      [this](const alps::cmn::control::MotorVelocityControllerParam & value) {
        RCLCPP_INFO(
          this->get_logger(),
          "velocity_controller_param: kp=%f, ki=%f, kd=%f, diff_lpf_time_const=%f, "
          "kff_velocity=%f, kff_acceleration=%f, target_acceleration_lpf_time_const=%f",
          value.pid_param.kp,
          value.pid_param.ki,
          value.pid_param.kd,
          value.pid_param.diff_lpf_time_const.count(),
          value.kff_velocity,
          value.kff_acceleration,
          value.target_acceleration_lpf_time_const.count());
      });
    param_angle_velocity_controller_.RegisterOnChangeCallback(
      [this](const alps::cmn::control::PidParam & value) {
        RCLCPP_INFO(
          this->get_logger(),
          "angle_velocity_controller_param: kp=%f, ki=%f, kd=%f, diff_lpf_time_const=%f",
          value.kp,
          value.ki,
          value.kd,
          value.diff_lpf_time_const.count());
      });
    param_target_velocity_.RegisterOnChangeCallback(
      [this](const double & value) { CbTargetChanged(value); });

    RCLCPP_INFO(this->get_logger(), "Start AngleVelocityControl");
  }

private:
  void CbTimer()
  {
    pub_target_angle_velocity_->publish(msg_target_angle_velocity_);
  }

  void CbTargetChanged(const double & target_angle_deg)
  {
    double target_angle_rad = alps::cmn::util::DegToRad(target_angle_deg);
    RCLCPP_INFO(
      this->get_logger(),
      "target_angle: %f [deg] (= %f [rad])",
      target_angle_deg,
      target_angle_rad);
    msg_target_angle_velocity_.target = target_angle_rad;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_input_;
  motor_control_interfaces::msg::ControlTarget msg_target_angle_velocity_;
  rclcpp::Publisher<motor_control_interfaces::msg::ControlTarget>::SharedPtr
    pub_target_angle_velocity_;
  alps::ros2::util::TypedParamServer<double> param_target_velocity_{
    *this, "target_velocity_deg_per_sec"};
  alps::ros2::util::TypedParamServer<alps::cmn::control::MotorVelocityControllerParam>
    param_velocity_controller_{*this, "velocity_controller_param"};
  alps::ros2::util::TypedParamServer<alps::cmn::control::PidParam> param_angle_velocity_controller_{
    *this, "angle_velocity_controller_param"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleVelocityControl>());
  rclcpp::shutdown();
  return 0;
}
