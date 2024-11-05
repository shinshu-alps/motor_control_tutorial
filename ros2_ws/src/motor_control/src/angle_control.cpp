#include "alps_ros2/type/custom_type_param_conversion_rule.hpp"
#include "alps_ros2/util/typed_param.hpp"

using namespace std::chrono_literals;

class TypedParamServer : public rclcpp::Node
{
public:
  TypedParamServer() : Node("typed_param_server")
  {
    // タイマー作成
    timer_ = this->create_wall_timer(20ms, std::bind(&TypedParamServer::TimerCb, this));

    // コールバックの登録
    param_angle_controller.RegisterOnChangeCallback(
      [this](const alps::cmn::control::PidParam & value) {
        RCLCPP_INFO(
          this->get_logger(),
          "angle_controller_param: kp=%f, ki=%f, kd=%f, diff_lpf_time_const=%f",
          value.kp,
          value.ki,
          value.kd,
          value.diff_lpf_time_const.count());
      });

    RCLCPP_INFO(this->get_logger(), "Start TypedParamServer");
  }

  void TimerCb() {}

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_angle_;
  alps::ros2::util::TypedParamServer<alps::cmn::control::PidParam> param_angle_controller{
    *this, "angle_controller"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TypedParamServer>());
  rclcpp::shutdown();
  return 0;
}
