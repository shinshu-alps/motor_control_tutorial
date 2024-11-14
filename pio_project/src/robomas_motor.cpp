/**
 * @file robomas_motor.cpp
 * @brief ロボマスモーターを対象として実機制御
 */

#ifdef ROBOMAS_MOTOR

#include "alps_cmn/actuator/robomas_motor.hpp"

#include "alps_cmn/communication/can_data_frame.hpp"
#include "alps_cmn/communication/can_param.hpp"
#include "alps_cmn/control/pid.hpp"
#include "alps_cmn/type/custom_type_serialize_rule.hpp"
#include "alps_mbed/communication/buffered_can_io.hpp"
#include "can_map.hpp"
#include "control_target.hpp"
#include "ctrl_mode.hpp"
#include "mbed.h"
#include "robomas_board.hpp"

template <class T>
using CanSubscriber = alps::cmn::communication::CanSubscriber<T>;
template <class T>
using CanPublisher = alps::cmn::communication::CanPublisher<T>;
template <class T>
using CanParamClient = alps::cmn::communication::CanParamClient<T>;

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

  // ロボマスモータ
  using M2006 = alps::cmn::actuator::robomas_motor::M2006<mbed::HighResClock>;
  using alps::cmn::actuator::robomas_motor::Id;
  alps::cmn::actuator::robomas_motor::CanPort can_port{robomas_transceiver};
  M2006 motor{can_port, Id::kMotor1};

  // 制御モード
  CtrlMode ctrl_mode = CtrlMode::kNone;

  // 制御器
  using MotorAngleController =
    alps::cmn::control::MotorAngleController<mbed::HighResClock, decltype(motor), decltype(motor)>;
  using MotorVelocityController = alps::cmn::control::
    MotorVelocityController<mbed::HighResClock, decltype(motor), decltype(motor)>;
  using MotorAngleVelocityController =
    alps::cmn::control::MotorAngleVelocityController<mbed::HighResClock>;
  // ※パラメータはここではセットしていない
  MotorAngleController angle_controller{motor, motor, alps::cmn::control::PidParam{}};
  MotorVelocityController velocity_controller{
    motor, motor, alps::cmn::control::MotorVelocityControllerParam{}};
  MotorAngleVelocityController angle_velocity_controller{alps::cmn::control::PidParam{}};

  // 目標値
  ControlTarget angle_ctrl_target;
  ControlTarget velocity_ctrl_target;
  ControlTarget angle_velocity_ctrl_target;

  // サブスクライバー
  auto cb_angle_ctrl_target = [&](const ControlTarget & target) {
    angle_ctrl_target = target;
    angle_controller.SetTargetAngle(target.target);
    ctrl_mode = CtrlMode::kAngle;
  };
  auto cb_velocity_ctrl_target = [&](const ControlTarget & target) {
    velocity_ctrl_target = target;
    velocity_controller.SetTargetVelocity(target.target);
    ctrl_mode = CtrlMode::kVelocity;
  };
  auto cb_angle_velocity_ctrl_target = [&](const ControlTarget & target) {
    angle_velocity_ctrl_target = target;
    angle_velocity_controller.SetTargetAngle(target.target);
    ctrl_mode = CtrlMode::kAngleVelocity;
  };
  CanSubscriber<ControlTarget> sub_angle_ctrl_target{
    pc_mcu_transceiver, can_map::kIdTargetAngle, cb_angle_ctrl_target};
  CanSubscriber<ControlTarget> sub_velocity_ctrl_target{
    pc_mcu_transceiver, can_map::kIdTargetVelocity, cb_velocity_ctrl_target};
  CanSubscriber<ControlTarget> sub_angle_velocity_ctrl_target{
    pc_mcu_transceiver, can_map::kIdTargetAngleVelocity, cb_angle_velocity_ctrl_target};

  // パブリッシャー
  CanPublisher<can_map::AngleControllerCalcInfo> pub_angle_controller_calc_info{
    pc_mcu_transceiver, can_map::kIdAngleControllerCalcInfo};
  CanPublisher<can_map::VelocityControllerCalcInfo> pub_velocity_controller_calc_info{
    pc_mcu_transceiver, can_map::kIdVelocityControllerCalcInfo};
  CanPublisher<can_map::AngleVelocityControllerCalcInfo> pub_angle_velocity_controller_calc_info{
    pc_mcu_transceiver, can_map::kIdAngleControllerCalcInfo};

  // パラメーター
  auto cb_angle_controller_param = [&](const can_map::AngleControllerParam & param) {
    angle_controller.SetParam(param);
  };
  auto cb_velocity_controller_param = [&](const can_map::VelocityControllerParam & param) {
    velocity_controller.SetParam(param);
  };
  auto cb_angle_velocity_controller_param =
    [&](const can_map::AngleVelocityControllerParam & param) {
      angle_velocity_controller.SetParam(param);
    };
  CanParamClient<can_map::AngleControllerParam> param_angle_controller{
    pc_mcu_transceiver,
    can_param_port,
    can_map::kIdAngleControllerParam,
    cb_angle_controller_param};
  CanParamClient<can_map::VelocityControllerParam> param_velocity_controller{
    pc_mcu_transceiver,
    can_param_port,
    can_map::kIdVelocityControllerParam,
    cb_velocity_controller_param};
  CanParamClient<can_map::AngleVelocityControllerParam> param_angle_velocity_controller{
    pc_mcu_transceiver,
    can_param_port,
    can_map::kIdAngleVelocityControllerParam,
    cb_angle_velocity_controller_param};

  while (true) {
    // CAN受信データ処理
    pc_mcu_transceiver.ReadUpdate();
    robomas_transceiver.ReadUpdate();
    can_param_port.Update();

    // Enable監視
    if (false
        // 本当はEnableの立ち上がりを監視してコントローラーにリセットをかけたい
    ) {
      angle_controller.Reset();
      velocity_controller.Reset();
      angle_velocity_controller.Reset();
    }

    // 制御
    switch (ctrl_mode) {
      case CtrlMode::kNone:  // 待機
        motor.Drive(0.0f);
        break;
      case CtrlMode::kAngle:  // 角度制御
        angle_controller.Control(angle_ctrl_target.ff_value);
        pub_angle_controller_calc_info.Publish(angle_controller.GetCalcInfo());
        break;
      case CtrlMode::kVelocity:  // 速度制御
        velocity_controller.Control(velocity_ctrl_target.ff_value);
        pub_velocity_controller_calc_info.Publish(velocity_controller.GetCalcInfo());
        break;
      case CtrlMode::kAngleVelocity:  // 角度-速度制御
        angle_velocity_controller.Control(velocity_controller, angle_velocity_ctrl_target.ff_value);
        pub_angle_velocity_controller_calc_info.Publish(angle_velocity_controller.GetCalcInfo());
        break;
    }

    // モーターの出力値をCAN通信で送信
    can_port.PublishMotorOutput();

    ThisThread::sleep_for(20ms);
  }
}

#endif
