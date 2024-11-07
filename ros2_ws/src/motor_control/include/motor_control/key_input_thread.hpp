#include <iostream>
#include <mutex>

#include "rclcpp/node.hpp"

class KeyInputThread
{
public:
  KeyInputThread(rclcpp::Node & node, std::string input_name, double initial_val = 0.0)
  : logger_{node.get_logger()}, input_name_{input_name}, input_val_{initial_val}
  {
    std::thread input_thread(&KeyInputThread::InputThread, this);
    input_thread.detach();
  }

  double GetInputVal()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return input_val_;
  }

private:
  void InputThread()
  {
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(logger_, "Start KeyInputThread");

    auto rate = rclcpp::WallRate(100ms);
    while (rclcpp::ok()) {
      std::cout << "Enter" << input_name_ << ": ";
      double input_val;
      if (!(std::cin >> input_val)) {
        RCLCPP_ERROR(logger_, "Invalid input");
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        continue;
      }

      {  // 値セット
        std::lock_guard<std::mutex> lock(mtx_);
        input_val_ = input_val;
      }

      rate.sleep();
    }
  }

  rclcpp::Logger logger_;
  std::string input_name_;
  std::mutex mtx_;
  double input_val_;
};