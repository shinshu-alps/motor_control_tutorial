#include "alps_cmn/control/motor_angle_controller.hpp"
#include "alps_cmn/control/motor_angle_velocity_controller.hpp"
#include "alps_cmn/control/motor_velocity_controller.hpp"
#include "alps_cmn/util/angle.hpp"
#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/type/custom_type_topic_conversion_rule.hpp"
#include "alps_ros2/type/topic_converter.hpp"
#include "alps_ros2/util/typed_param.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motor_control/control_target.hpp"
#include "motor_control/ctrl_mode.hpp"
#include "motor_control/virtual_motor.hpp"
#include "motor_control_interfaces/msg/angle.hpp"
#include "motor_control_interfaces/msg/control_target.hpp"

using namespace std::chrono_literals;

class SimMotor : public rclcpp::Node
{
public:
  SimMotor() : Node("sim_motor")
  {
    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&SimMotor::CbTimer, this));

    // パブリッシャー作成
    pub_angle_controller_calc_info_ =
      this->create_publisher<alps_interfaces::msg::MotorAngleControllerCalcInfo>(
        "calc_info/angle_controller", 1);
    pub_velocity_controller_calc_info_ =
      this->create_publisher<alps_interfaces::msg::MotorVelocityControllerCalcInfo>(
        "calc_info/velocity_controller", 1);
    pub_angle_velocity_controller_calc_info_ =
      this->create_publisher<alps_interfaces::msg::MotorAngleControllerCalcInfo>(
        "calc_info/angle_velocity_controller", 1);
    pub_now_angle_ = this->create_publisher<motor_control_interfaces::msg::Angle>("now_angle", 1);

    // サブスクライバー作成
    sub_target_angle_ = this->create_subscription<motor_control_interfaces::msg::ControlTarget>(
      "target/angle", 1, [this](const motor_control_interfaces::msg::ControlTarget::SharedPtr msg) {
        if (this->ctrl_mode_ != CtrlMode::kAngle) {
          this->ctrl_mode_ = CtrlMode::kAngle;
          this->angle_controller.Reset();
          RCLCPP_INFO(this->get_logger(), "Change to Angle Control Mode");
          this->motor_.ResetAngle();
        }
        this->angle_ctrl_target_.target = msg->target;
        angle_controller.SetTargetAngle(msg->target);
      });
    sub_target_velocity_ = this->create_subscription<motor_control_interfaces::msg::ControlTarget>(
      "target/velocity",
      1,
      [this](const motor_control_interfaces::msg::ControlTarget::SharedPtr msg) {
        if (this->ctrl_mode_ != CtrlMode::kVelocity) {
          this->ctrl_mode_ = CtrlMode::kVelocity;
          this->velocity_controller.Reset();
          RCLCPP_INFO(this->get_logger(), "Change to Velocity Control Mode");
        }
        this->velocity_ctrl_target_.target = msg->target;
        velocity_controller.SetTargetVelocity(msg->target);
      });
    sub_target_angle_velocity_ =
      this->create_subscription<motor_control_interfaces::msg::ControlTarget>(
        "target/angle_velocity",
        1,
        [this](const motor_control_interfaces::msg::ControlTarget::SharedPtr msg) {
          if (this->ctrl_mode_ != CtrlMode::kAngleVelocity) {
            this->ctrl_mode_ = CtrlMode::kAngleVelocity;
            this->angle_velocity_controller.Reset();
            RCLCPP_INFO(this->get_logger(), "Change to Angle-Velocity Control Mode");
            this->motor_.ResetAngle();
          }
          this->angle_velocity_ctrl_target_.target = msg->target;
          angle_velocity_controller.SetTargetAngle(msg->target);
        });

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
        angle_controller.SetParam(value);
        angle_controller.Reset();
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
        velocity_controller.SetParam(value);
        velocity_controller.Reset();
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
        angle_velocity_controller.SetParam(value);
        angle_velocity_controller.Reset();
      });

    RCLCPP_INFO(this->get_logger(), "Start sim_motor");
  }

private:
  void CbTimer()
  {
    // 制御
    switch (ctrl_mode_) {
      case CtrlMode::kNone:  // 待機
        motor_.Drive(0.0f);
        break;
      case CtrlMode::kAngle:  // 角度制御
        angle_controller.Control(angle_ctrl_target_.ff_value);
        pub_angle_controller_calc_info_->publish(
          alps::ros2::type::ToRosTopic<alps_interfaces::msg::MotorAngleControllerCalcInfo>(
            angle_controller.GetCalcInfo()));
        break;
      case CtrlMode::kVelocity:  // 速度制御
        velocity_controller.Control(velocity_ctrl_target_.ff_value);
        pub_velocity_controller_calc_info_->publish(
          alps::ros2::type::ToRosTopic<alps_interfaces::msg::MotorVelocityControllerCalcInfo>(
            velocity_controller.GetCalcInfo()));
        break;
      case CtrlMode::kAngleVelocity:  // 角度-速度制御
        angle_velocity_controller.Control(
          velocity_controller, angle_velocity_ctrl_target_.ff_value);
        pub_velocity_controller_calc_info_->publish(
          alps::ros2::type::ToRosTopic<alps_interfaces::msg::MotorVelocityControllerCalcInfo>(
            velocity_controller.GetCalcInfo()));
        pub_angle_velocity_controller_calc_info_->publish(
          alps::ros2::type::ToRosTopic<alps_interfaces::msg::MotorAngleControllerCalcInfo>(
            angle_velocity_controller.GetCalcInfo()));
        break;
    }

    // 現在の角度送信
    motor_control_interfaces::msg::Angle msg_now_angle;
    msg_now_angle.angle = motor_.GetAngle();
    pub_now_angle_->publish(msg_now_angle);
  }

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // パブリッシャー
  rclcpp::Publisher<alps_interfaces::msg::MotorAngleControllerCalcInfo>::SharedPtr
    pub_angle_controller_calc_info_;
  rclcpp::Publisher<alps_interfaces::msg::MotorVelocityControllerCalcInfo>::SharedPtr
    pub_velocity_controller_calc_info_;
  rclcpp::Publisher<alps_interfaces::msg::MotorAngleControllerCalcInfo>::SharedPtr
    pub_angle_velocity_controller_calc_info_;
  rclcpp::Publisher<motor_control_interfaces::msg::Angle>::SharedPtr pub_now_angle_;

  // サブスクライバー
  rclcpp::Subscription<motor_control_interfaces::msg::ControlTarget>::SharedPtr sub_target_angle_;
  rclcpp::Subscription<motor_control_interfaces::msg::ControlTarget>::SharedPtr
    sub_target_velocity_;
  rclcpp::Subscription<motor_control_interfaces::msg::ControlTarget>::SharedPtr
    sub_target_angle_velocity_;

  // パラメータ
  alps::ros2::util::TypedParamClient<alps::cmn::control::PidParam> param_angle_controller_{
    *this, "control_commander", "angle_controller_param"};
  alps::ros2::util::TypedParamClient<alps::cmn::control::MotorVelocityControllerParam>
    param_velocity_controller_{*this, "control_commander", "velocity_controller_param"};
  alps::ros2::util::TypedParamClient<alps::cmn::control::PidParam> param_angle_velocity_controller_{
    *this, "control_commander", "angle_velocity_controller_param"};

  // 仮想モーター
  VirtualMotor motor_{
    // 10.0f
    20.0f,   // float torque_constant =
    0.1f,    // float moment_of_inertia =
    1024,    // uint32_t resolution =
    0.05f,   // float abs_static_friction =
    0.001f,  // float abs_kinetic_friction =
    1.0f,    // float damper =
    0.0f,    // float initial_velocity =
    0.0f     // float initial_angle =
  };

  // モーター制御
  CtrlMode ctrl_mode_{CtrlMode::kNone};

  // 制御器
  using MotorAngleController = alps::cmn::control::
    MotorAngleController<std::chrono::high_resolution_clock, VirtualMotor, VirtualMotor>;
  using MotorVelocityController = alps::cmn::control::
    MotorVelocityController<std::chrono::high_resolution_clock, VirtualMotor, VirtualMotor>;
  using MotorAngleVelocityController =
    alps::cmn::control::MotorAngleVelocityController<std::chrono::high_resolution_clock>;
  MotorAngleController angle_controller{motor_, motor_, alps::cmn::control::PidParam{}};
  MotorVelocityController velocity_controller{
    motor_, motor_, alps::cmn::control::MotorVelocityControllerParam{}};
  MotorAngleVelocityController angle_velocity_controller{alps::cmn::control::PidParam{}};

  // 目標値
  ControlTarget angle_ctrl_target_;
  ControlTarget velocity_ctrl_target_;
  ControlTarget angle_velocity_ctrl_target_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimMotor>());
  rclcpp::shutdown();
  return 0;
}
