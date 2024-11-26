#ifndef MOTOR_CONTROL_VIRTUAL_MOTOR_HPP
#define MOTOR_CONTROL_VIRTUAL_MOTOR_HPP

#include <cmath>
#include <cstdio>

#include "alps_cmn/actuator/i_motor.hpp"
#include "alps_cmn/control/integrator.hpp"
#include "alps_cmn/sensor/i_rotary_sensor.hpp"

/**
 * @brief 仮想的なモーター
 */
class VirtualMotor : public alps::cmn::actuator::IMotor<VirtualMotor>,
                     public alps::cmn::sensor::IRotarySensor<VirtualMotor>
{
public:
  /**
   * @brief コンストラクタ
   * 
   * @param torque_constant   モーターのトルク定数
   * @param moment_of_inertia 慣性モーメント
   * @param 
   * @param damper            ダンパー係数
   * @param initial_velocity  初期角速度
   * @param initial_angle     初期角度
   */
  VirtualMotor(
    float torque_constant = 1.0f,
    float moment_of_inertia = 1.0f,
    float abs_static_friction = 0.05f,
    float abs_kinetic_friction = 0.02f,
    float damper = 1.0f,
    float initial_velocity = 0.0f,
    float initial_angle = 0.0f)
  : torque_constant_{torque_constant},
    moment_of_inertia_{moment_of_inertia},
    abs_static_friction_{abs_static_friction},
    abs_kinetic_friction_{abs_kinetic_friction},
    damper_{damper}
  {
    velocity_integrator_.SetCurrentValue(initial_velocity);
    angle_integrator_.SetCurrentValue(initial_angle);
  }

  /**
   * @brief モーターを駆動する
   * 
   * @param output_ratio 駆動比
   */
  void Drive(float output_ratio)
  {
    drive_ratio_ = etl::clamp(output_ratio, -1.0f, 1.0f);
  }

  /**
   * @brief 簡単なモーターのモデルを用いてモーター駆動値を反映した角度を計算する
   * 
   * @return double 計算した角度
   */
  double GetAngle()
  {
    float pure_acceleration = drive_ratio_ * torque_constant_ / moment_of_inertia_;  // 純粋な加速度

    float friction{0.0f};
    if (std::abs(velocity_integrator_.GetCurrentValue()) < 0.01f) {
      // 静止摩擦
      // printf("static friction\n");
      if (std::abs(pure_acceleration) < abs_static_friction_) {
        pure_acceleration = 0.0f;
        velocity_integrator_.SetCurrentValue(0.0f);
      } else {
        friction = std::copysign(abs_static_friction_, velocity_integrator_.GetCurrentValue());
      }
    } else {
      // 動摩擦
      // printf("kinetic friction\n");
      friction = std::copysign(abs_kinetic_friction_, velocity_integrator_.GetCurrentValue());
    }
    float dumper_applied_acceleration =
      pure_acceleration -
      (damper_ * velocity_integrator_.GetCurrentValue());  // ダンパーを考慮した加速度
    float axis_acceleration = dumper_applied_acceleration - friction;  // 摩擦を考慮した加速度
    velocity_integrator_.Calculate(axis_acceleration);                 // 角速度を積分
    angle_integrator_.Calculate(velocity_integrator_.GetCurrentValue());  // 角度を積分

    return angle_integrator_.GetCurrentValue();
  }

private:
  /// @brief 現在のモーター駆動比
  float drive_ratio_{0.0f};
  /// @brief モーターのトルク定数
  float torque_constant_;
  /// @brief 慣性モーメント
  float moment_of_inertia_;
  /// @brief 絶対静止摩擦
  float abs_static_friction_;
  /// @brief 絶対動摩擦
  float abs_kinetic_friction_;

  /// @brief ダンパー係数
  float damper_;
  /// @brief 角速度の積分器
  alps::cmn::control::Integrator<std::chrono::high_resolution_clock> velocity_integrator_;
  /// @brief 角度の積分器
  alps::cmn::control::Integrator<std::chrono::high_resolution_clock> angle_integrator_;
};

#endif
