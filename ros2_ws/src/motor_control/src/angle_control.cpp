#include <iostream>
#include <mutex>

#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/util/typed_param.hpp"
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

    // 入力スレッドの開始
    std::thread input_thread(&AngleControl::InputThread, this);
    input_thread.detach();

    RCLCPP_INFO(this->get_logger(), "Start AngleControl");
  }

  void CbTimer()
  {
    std::lock_guard<std::mutex> lock(mtx_target_angle_);
    pub_target_angle_->publish(msg_target_angle_);
  }

  void InputThread()
  {
    rclcpp::sleep_for(1s);
    auto rate = rclcpp::Rate(100ms);
    while (rclcpp::ok()) {
      std::cout << "Enter Target Angle [deg]: ";
      double target_angle;
      if (!(std::cin >> target_angle)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid input");
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        continue;
      }

      {  // 目標値セット
        std::lock_guard<std::mutex> lock(mtx_target_angle_);
        msg_target_angle_.target_angle = target_angle;
      }

      rate.sleep();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_input_;
  motor_control_interfaces::msg::TargetAngle msg_target_angle_;
  std::mutex mtx_target_angle_;
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
