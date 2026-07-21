/**
 * @file VelocityPlanner.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 速度控制器基类
 * @version 0.1
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 定义一些速度规划器的固定操作和固定变量
 * @versioninfo :
 */
#pragma once

#include <cstdint>

class VelocityPlannerBase {

public:
  enum class planner_status : uint8_t {
    OK,
    NotInitialized,
    InvalidArgument,
    InvalidDeltaTime,
    NotConfigured,
  };

  struct constraints {
    float max_velocity;     // 最大速度
    float max_acceleration; // 最大加速度
    float max_deceleration; // 最大减速度
    float max_jerk;         // 最大加加速度
  };

  struct output {
    float velocity;
    float acceleration;
    bool reached;
  };

  virtual ~VelocityPlannerBase() = default;

  // 配置规划器
  virtual planner_status Configure(const constraints &limits) = 0;

  // 设置规划目标
  virtual planner_status SetTarget(float target_velocity) = 0;

  // 重规划设置
  virtual planner_status Replan(float new_target, float current_velocity) = 0;

  // 每个控制周期都要调用，单位为秒
  virtual planner_status Update(float dt, output &result) = 0;

  [[nodiscard]] virtual float GetTarget() const = 0;
  [[nodiscard]] virtual output GetOutput() const = 0;

protected:
  constraints limits_{};
  float target_velocity_{0.0f};
  output output_{};

  bool is_configured_{false};
  bool is_initialized_{false};
};
