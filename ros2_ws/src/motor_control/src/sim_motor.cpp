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
#include "motor_control/key_input_thread.hpp"
#include "motor_control/virtual_motor.hpp"
#include "motor_control_interfaces/msg/target_angle.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class SimMotor : public rclcpp::Node
{
public:
  SimMotor() : Node("sim_motor")
  {
    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&SimMotor::CbTimer, this));

    // パブリッシャー作成
    pub_visualized_plant_ =
      this->create_publisher<visualization_msgs::msg::Marker>("sim_motor/visualized_plant", 1);
    pub_visualized_plant_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("sim_motor/visualized_plant_pose", 1);
    pub_angle_controller_calc_info_ =
      this->create_publisher<alps_interfaces::msg::MotorAngleControllerCalcInfo>(
        "sim_motor/angle_controller_calc_info", 1);

    // サブスクライバー作成
    sub_target_angle_ = this->create_subscription<motor_control_interfaces::msg::TargetAngle>(
      "target_angle", 1, [this](const motor_control_interfaces::msg::TargetAngle::SharedPtr msg) {
        if (this->ctrl_mode_ != CtrlMode::kAngle) {
          this->ctrl_mode_ = CtrlMode::kAngle;
          this->angle_controller.Reset();
          RCLCPP_INFO(this->get_logger(), "Change to Angle Control Mode");
        }
        this->angle_ctrl_target_.target = msg->target_angle;
        angle_controller.SetTargetAngle(msg->target_angle);
      });

    msg_visualized_plant_.header.frame_id = "map";
    msg_visualized_plant_.type = visualization_msgs::msg::Marker::CYLINDER;
    msg_visualized_plant_.scale.x = 1.0;
    msg_visualized_plant_.scale.y = 1.0;
    msg_visualized_plant_.scale.z = 0.5;
    msg_visualized_plant_.color.r = 1.0;
    msg_visualized_plant_.color.a = 1.0;
    msg_visualized_plant_pose_.header.frame_id = "map";
    msg_visualized_plant_pose_.pose.position.x = 0.0;
    msg_visualized_plant_pose_.pose.position.y = 0.0;
    msg_visualized_plant_pose_.pose.position.z = 0.25;

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
        angle_controller.SetParam(value);
        angle_controller.Reset();
      });
    if (param_angle_controller_.HasValidValue()) {
      angle_controller.SetParam(*param_angle_controller_.GetParam());
    }

    RCLCPP_INFO(this->get_logger(), "Start SimMotor");
  }

private:
  void CbTimer()
  {
    // 受信データ処理

    // 制御
    switch (ctrl_mode_) {
      case CtrlMode::kNone:  // 待機
        motor_.Drive(0.0f);
        break;
      case CtrlMode::kAngle:  // 角度制御
        angle_controller.Control(angle_ctrl_target_.ff_value);
        // pub_angle_controller_calc_info.Publish(angle_controller.GetCalcInfo());
        break;
      case CtrlMode::kVelocity:  // 速度制御
        velocity_controller.Control(velocity_ctrl_target_.ff_value);
        // pub_velocity_controller_calc_info.Publish(velocity_controller.GetCalcInfo());
        break;
      case CtrlMode::kAngleVelocity:  // 角度-速度制御
        angle_velocity_controller.Control(
          velocity_controller, angle_velocity_ctrl_target_.ff_value);
        // pub_angle_velocity_controller_calc_info.Publish(angle_velocity_controller.GetCalcInfo());
        break;
    }

    // 計算情報
    pub_angle_controller_calc_info_->publish(
      alps::ros2::type::ToRosTopic<
        alps_interfaces::msg::MotorAngleControllerCalcInfo,
        alps::cmn::control::MotorAngleControllerCalcInfo>(angle_controller.GetCalcInfo()));

    // 可視化
    tf2::Quaternion q;
    q.setRPY(0, 0, motor_.GetAngle());
    msg_visualized_plant_.pose.orientation = tf2::toMsg(q);
    msg_visualized_plant_.header.stamp = this->now();
    pub_visualized_plant_->publish(msg_visualized_plant_);
    msg_visualized_plant_pose_.pose.orientation = tf2::toMsg(q);
    msg_visualized_plant_pose_.header.stamp = this->now();
    pub_visualized_plant_pose_->publish(msg_visualized_plant_pose_);
  }

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // パブリッシャー
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_visualized_plant_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_visualized_plant_pose_;
  rclcpp::Publisher<alps_interfaces::msg::MotorAngleControllerCalcInfo>::SharedPtr
    pub_angle_controller_calc_info_;

  // サブスクライバー
  rclcpp::Subscription<motor_control_interfaces::msg::TargetAngle>::SharedPtr sub_target_angle_;

  // パラメータ
  alps::ros2::util::TypedParamClient<alps::cmn::control::PidParam> param_angle_controller_{
    *this, "angle_control", "angle_controller_param"};

  // 仮想モーター
  VirtualMotor motor_{10.0f};
  visualization_msgs::msg::Marker msg_visualized_plant_;
  geometry_msgs::msg::PoseStamped msg_visualized_plant_pose_;

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
