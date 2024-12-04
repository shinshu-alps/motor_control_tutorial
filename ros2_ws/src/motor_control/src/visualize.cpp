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
#include "motor_control_interfaces/msg/angle.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class SimMotor : public rclcpp::Node
{
public:
  SimMotor() : Node("visualize")
  {
    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&SimMotor::CbTimer, this));

    // パブリッシャー作成
    pub_visualized_plant_ =
      this->create_publisher<visualization_msgs::msg::Marker>("visualize/visualized_plant", 1);
    pub_visualized_plant_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("visualize/visualized_plant_pose", 1);

    // サブスクライバー作成
    sub_now_angle_ = this->create_subscription<motor_control_interfaces::msg::Angle>(
      "now_angle", 1, [this](const motor_control_interfaces::msg::Angle::SharedPtr msg) {
        this->now_angle_ = msg->angle;
      });

    // 可視化用メッセージ準備
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

    RCLCPP_INFO(this->get_logger(), "Start visualize");
  }

private:
  void CbTimer()
  {
    // 可視化
    tf2::Quaternion q;
    q.setRPY(0, 0, this->now_angle_);
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

  // サブスクライバー
  rclcpp::Subscription<motor_control_interfaces::msg::Angle>::SharedPtr sub_now_angle_;

  // 視覚化用メッセージ
  visualization_msgs::msg::Marker msg_visualized_plant_;
  geometry_msgs::msg::PoseStamped msg_visualized_plant_pose_;

  // 現在の角度
  float now_angle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimMotor>());
  rclcpp::shutdown();
  return 0;
}
