/**
 * @file robomas_motor.cpp
 * @brief ロボマスモーターを対象として実機制御
 */

#ifdef ROBOMAS_MOTOR

#include "alps_cmn/actuator/robomas_motor.hpp"

#include "alps_cmn/communication/can_data_frame.hpp"
#include "alps_cmn/communication/can_param.hpp"
#include "alps_cmn/control/pid.hpp"
#include "alps_mbed/communication/buffered_can_io.hpp"
#include "can_map.hpp"
#include "mbed.h"
#include "robomas_board.hpp"

enum class CtrlMode : uint8_t {
  kNone,
  kAngle,
  kVelocity,
  kAngleVelocity,
};

int main()
{
  // クロックの設定
  ::mbed::HighResClock::lock();

  // ロボマスモーター用CAN
  alps::mbed::communication::BufferedCanIo<> robomas_can{
    RobomasBoard::kCan2Rd, RobomasBoard::kCan2Td, 300us};
  alps::cmn::communication::CanTypedDataTransceiver<decltype(robomas_can)> robomas_transceiver{
    robomas_can};
  alps::cmn::actuator::robomas_motor::CanPort robomas_can_port{robomas_transceiver};

  // PC-MCU用CAN
  alps::mbed::communication::BufferedCanIo<300, 300> pc_mcu_can{
    RobomasBoard::kCan1Rd, RobomasBoard::kCan1Td};
  alps::cmn::communication::CanTypedDataTransceiver<decltype(pc_mcu_can), 100> pc_mcu_transceiver{
    pc_mcu_can};
  alps::cmn::communication::CanParamPort<30> can_param_port{
    pc_mcu_transceiver, can_map::kIdParamPort};

  // ロボマスモータ用CAN
  alps::mbed::communication::BufferedCanIo<> can(D10, D2);
  alps::cmn::communication::CanTypedDataTransceiver<decltype(can)> transceiver(can);

  // ロボマスモータ
  using M2006 = alps::cmn::actuator::robomas_motor::M2006<mbed::HighResClock>;
  using alps::cmn::actuator::robomas_motor::Id;
  alps::cmn::actuator::robomas_motor::CanPort can_port{transceiver};
  M2006 motor{can_port, Id::kMotor1};

  // 制御モード
  CtrlMode ctrl_mode = CtrlMode::kNone;

  // 制御器
  alps::cmn::control::MotorAngleController<::mbed::HighResClock, decltype(motor), decltype(motor)>
    angle_controller{motor, motor, alps::cmn::control::PidParam{}};
  alps::cmn::control::
    MotorVelocityController<::mbed::HighResClock, decltype(motor), decltype(motor)>
      velocity_controller{motor, motor, alps::cmn::control::MotorVelocityControllerParam{}};
  alps::cmn::control::MotorAngleVelocityController<::mbed::HighResClock> angle_velocity_controller{
    alps::cmn::control::PidParam{}};

  while (1) {
    // CAN受信データ処理（ロボマスモータの角度情報等を更新）
    transceiver.ReadUpdate();

    // 制御
    switch (ctrl_mode) {
      case CtrlMode::kNone:  // 待機
        motor.Drive(0.0f);
        break;
      case CtrlMode::kAngle:  // 角度制御
        angle_controller.Control();
        break;
      case CtrlMode::kVelocity:  // 速度制御
        velocity_controller.Control();
        break;
      case CtrlMode::kAngleVelocity:  // 角度-速度制御
        angle_velocity_controller.Control(velocity_controller);
        break;
    }

    // モーターの出力値をCAN通信で送信
    can_port.PublishMotorOutput();

    ThisThread::sleep_for(20ms);
  }
}

#endif
