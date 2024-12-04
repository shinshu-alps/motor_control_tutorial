#include "alps_cmn/util/angle.hpp"
#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/util/typed_param.hpp"
#include "motor_control/ctrl_mode.hpp"
#include "motor_control_interfaces/msg/control_target.hpp"

using namespace std::chrono_literals;

class ControlCommander : public rclcpp::Node
{
public:
  ControlCommander() : Node("control_commander")
  {
    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&ControlCommander::CbTimer, this));

    // パブリッシャー作成
    pub_target_angle_ =
      this->create_publisher<motor_control_interfaces::msg::ControlTarget>("target/angle", 1);
    pub_target_velocity_ =
      this->create_publisher<motor_control_interfaces::msg::ControlTarget>("target/velocity", 1);
    pub_target_angle_velocity_ =
      this->create_publisher<motor_control_interfaces::msg::ControlTarget>(
        "target/angle_velocity", 1);

    // パラメータコールバックの登録
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

    param_target_angle_.RegisterOnChangeCallback(
      [this](const double & value) { CbTargetAngleChanged(value); });
    param_target_velocity_.RegisterOnChangeCallback(
      [this](const double & value) { CbTargetVelocityChanged(value); });

    // 目標値初期化
    msg_target_angle_.target = 0.0;
    msg_target_velocity_.target = 0.0;
    msg_target_angle_velocity_.target = 0.0;

    RCLCPP_INFO(this->get_logger(), "Start ControlCommander");
  }

private:
  void CbTimer()
  {
    auto ctrl_mode_flag = param_ctrl_mode_flag_.GetParam().value();
    int selected_mode_num = 0;
    if (ctrl_mode_flag.angle_mode) {
      now_mode_ = CtrlMode::kAngle;
      selected_mode_num++;
    }
    if (ctrl_mode_flag.velocity_mode) {
      now_mode_ = CtrlMode::kVelocity;
      selected_mode_num++;
    }
    if (ctrl_mode_flag.angle_velocity_mode) {
      now_mode_ = CtrlMode::kAngleVelocity;
      selected_mode_num++;
    }
    if (selected_mode_num != 1) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,  // 1s
        "Invalid ctrl_mode_flag. Please select only one mode.");
      now_mode_ = CtrlMode::kNone;
    }

    switch (now_mode_) {
      case CtrlMode::kAngle:
        pub_target_angle_->publish(msg_target_angle_);
        break;
      case CtrlMode::kVelocity:
        pub_target_velocity_->publish(msg_target_velocity_);
        break;
      case CtrlMode::kAngleVelocity:
        pub_target_angle_velocity_->publish(msg_target_angle_velocity_);
        break;
      default:
        break;
    }
  }

  void CbTargetAngleChanged(const double & target_angle_deg)
  {
    double target_angle_rad = alps::cmn::util::DegToRad(target_angle_deg);
    RCLCPP_INFO(
      this->get_logger(),
      "target_angle: %f [deg] (= %f [rad])",
      target_angle_deg,
      target_angle_rad);
    msg_target_angle_.target = target_angle_rad;
    msg_target_angle_velocity_.target = target_angle_rad;
  }

  void CbTargetVelocityChanged(const double & target_velocity_deg)
  {
    double target_velocity_rad = alps::cmn::util::DegToRad(target_velocity_deg);
    RCLCPP_INFO(
      this->get_logger(),
      "target_velocity: %f [deg/s] (= %f [rad/s])",
      target_velocity_deg,
      target_velocity_rad);
    msg_target_velocity_.target = target_velocity_rad;
  }

  rclcpp::TimerBase::SharedPtr timer_;

  CtrlMode now_mode_ = CtrlMode::kNone;

  motor_control_interfaces::msg::ControlTarget msg_target_angle_;
  motor_control_interfaces::msg::ControlTarget msg_target_velocity_;
  motor_control_interfaces::msg::ControlTarget msg_target_angle_velocity_;

  rclcpp::Publisher<motor_control_interfaces::msg::ControlTarget>::SharedPtr pub_target_angle_;
  rclcpp::Publisher<motor_control_interfaces::msg::ControlTarget>::SharedPtr pub_target_velocity_;
  rclcpp::Publisher<motor_control_interfaces::msg::ControlTarget>::SharedPtr
    pub_target_angle_velocity_;

  alps::ros2::util::TypedParamServer<alps::cmn::control::PidParam> param_angle_controller_{
    *this, "angle_controller_param"};
  alps::ros2::util::TypedParamServer<alps::cmn::control::MotorVelocityControllerParam>
    param_velocity_controller_{*this, "velocity_controller_param"};
  alps::ros2::util::TypedParamServer<alps::cmn::control::PidParam> param_angle_velocity_controller_{
    *this, "angle_velocity_controller_param"};

  alps::ros2::util::TypedParamServer<CtrlModeFlag> param_ctrl_mode_flag_{*this, "ctrl_mode_flag"};

  alps::ros2::util::TypedParamServer<double> param_target_angle_{*this, "target_angle_deg"};
  alps::ros2::util::TypedParamServer<double> param_target_velocity_{
    *this, "target_velocity_deg_per_sec"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlCommander>());
  rclcpp::shutdown();
  return 0;
}
