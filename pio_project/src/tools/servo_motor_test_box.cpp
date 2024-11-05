/**
 * @brief サーボモータ4個をボタンとポテンショメータで制御する装置のプログラム
 */

#ifdef TOOL_SERVO_MOTOR_TEST_BOX

#include <array>

#include "alps_mbed/actuator/servo_motor.hpp"
#include "alps_mbed/gpio/analog_in.hpp"
#include "alps_mbed/gpio/digital_in.hpp"
#include "alps_mbed/gpio/digital_out.hpp"

// サーボパラメータ設定
// 繋いだサーボモータに合わせて変更する
constexpr alps::cmn::actuator::ServoMotorParam kServoParam1{alps::cmn::actuator::kSg90};
constexpr alps::cmn::actuator::ServoMotorParam kServoParam2{alps::cmn::actuator::kSg90};
constexpr alps::cmn::actuator::ServoMotorParam kServoParam3{alps::cmn::actuator::kSg90};
constexpr alps::cmn::actuator::ServoMotorParam kServoParam4{alps::cmn::actuator::kSg90};

// ポテンショメータの最大値と最小値
constexpr float max_potentiometer_val1 = 0.9;
constexpr float min_potentiometer_val1 = 0.1;
constexpr float max_potentiometer_val2 = 0.9;
constexpr float min_potentiometer_val2 = 0.1;
constexpr float max_potentiometer_val3 = 0.9;
constexpr float min_potentiometer_val3 = 0.1;
constexpr float max_potentiometer_val4 = 0.9;
constexpr float min_potentiometer_val4 = 0.1;

// ピン配置
constexpr PinName kPotentiometer1 = A5;
constexpr PinName kPotentiometer2 = A4;
constexpr PinName kPotentiometer3 = A6;
constexpr PinName kPotentiometer4 = A0;
constexpr PinName kButton1 = D3;
constexpr PinName kButton2 = D12;
constexpr PinName kButton3 = D2;
constexpr PinName kButton4 = D1;
constexpr PinName kLed1 = D6;
constexpr PinName kLed2 = D8;
constexpr PinName kLed3 = D7;
constexpr PinName kLed4 = D11;
constexpr PinName kPwm1 = A2;
constexpr PinName kPwm2 = D10;
constexpr PinName kPwm3 = A3;
constexpr PinName kPwm4 = A1;

/// @brief ポテンショメータ，スイッチの入力からサーボモータを制御するクラス
class UnitController
{
public:
  UnitController(
    PinName potentiometer_pin,
    PinName button_pin,
    PinName led_pin,
    PinName servo_pwm_pin,
    const alps::cmn::actuator::ServoMotorParam & servo_param,
    float max_potentiometer_val,
    float min_potentiometer_val,
    std::function<void(bool, float, float)> output_log_func)
  : potentiometer_(potentiometer_pin),
    button_(button_pin),
    led_(led_pin),
    servo_pwm_(servo_pwm_pin),
    servo_param_(servo_param),
    servo_motor_(servo_pwm_, servo_param),
    max_potentiometer_val_{max_potentiometer_val},
    min_potentiometer_val_{min_potentiometer_val},
    output_log_func_{output_log_func}
  {
  }

  /// @brief 周期実行処理
  void Update()
  {
    bool button_now_status = button_.Read();
    float potentiometer_input = potentiometer_.Read();
    float scale = 1.0 / (max_potentiometer_val_ - min_potentiometer_val_);
    float normalized_val = (potentiometer_input - min_potentiometer_val_) * scale;
    float target_angle = normalized_val * servo_param_.max_angle;

    if ((button_now_status) && (!button_prev_status_)) {
      enable_ = !enable_;
    }
    button_prev_status_ = button_now_status;
    led_.Write(enable_);

    if (enable_) {
      servo_motor_.SetAngle(target_angle);
    } else {
      servo_motor_.SetFree();
    }
    output_log_func_(enable_, potentiometer_input, target_angle);
  }

private:
  alps::mbed::gpio::AnalogIn potentiometer_;
  alps::mbed::gpio::DigitalIn button_;
  alps::mbed::gpio::DigitalOut led_;
  alps::mbed::gpio::PwmOut servo_pwm_;
  alps::cmn::actuator::ServoMotorParam servo_param_;
  alps::mbed::actuator::ServoMotor servo_motor_;

  bool enable_{false};
  bool button_prev_status_{false};

  const float max_potentiometer_val_;
  const float min_potentiometer_val_;

  std::function<void(bool, float, float)> output_log_func_;
};

// ログ出力関数
void OutputLogData(int motor_number, bool enable, float potentiometer_input, float target_angle)
{
  printf(
    "|motor%d: enable=%d, potentio=%f, target=%f ",
    motor_number,
    enable,
    potentiometer_input,
    target_angle);
}

// ユニット別ログ出力関数の用意
auto output_log_func1 = [](bool enable, float potentiometer_input, float target_angle) {
  OutputLogData(1, enable, potentiometer_input, target_angle);
};
auto output_log_func2 = [](bool enable, float potentiometer_input, float target_angle) {
  OutputLogData(2, enable, potentiometer_input, target_angle);
};
auto output_log_func3 = [](bool enable, float potentiometer_input, float target_angle) {
  OutputLogData(3, enable, potentiometer_input, target_angle);
};
auto output_log_func4 = [](bool enable, float potentiometer_input, float target_angle) {
  OutputLogData(4, enable, potentiometer_input, target_angle);
  printf("\n");
};

// 初期化
std::array<UnitController, 4> unit_controllers{
  UnitController(
    kPotentiometer1,
    kButton1,
    kLed1,
    kPwm1,
    kServoParam1,
    max_potentiometer_val1,
    min_potentiometer_val1,
    output_log_func1),
  UnitController(
    kPotentiometer2,
    kButton2,
    kLed2,
    kPwm2,
    kServoParam2,
    max_potentiometer_val2,
    min_potentiometer_val2,
    output_log_func2),
  UnitController(
    kPotentiometer3,
    kButton3,
    kLed3,
    kPwm3,
    kServoParam3,
    max_potentiometer_val3,
    min_potentiometer_val3,
    output_log_func3),
  UnitController(
    kPotentiometer4,
    kButton4,
    kLed4,
    kPwm4,
    kServoParam4,
    max_potentiometer_val4,
    min_potentiometer_val4,
    output_log_func4),
};

int main()
{
  printf("start");

  while (true) {
    for (auto & unit_controller : unit_controllers) {
      unit_controller.Update();
    }

    ThisThread::sleep_for(20ms);
  }
}

#endif
