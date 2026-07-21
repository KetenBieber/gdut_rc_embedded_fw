/**
 * @file SlopePlanner.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 斜坡规划器Slope Planner
 * @version 0.1
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * 需要明确规划器与控制器的职责边界，规划器让期望速度满足velocity、accelerate、jerk约束
 * 控制器负责让实际速度跟踪规划速度
 * 规划器不应该每周期都追着实际速度跑，但也不能完全无视实际速度
 *
 * 这个版本slope规划减速会考虑当前实际速度进行减速规划，而不是规划于target，消除了所谓的减速迟缓的现象
 * @versioninfo :
 */
#pragma once

#include "VelocityPlanner.hpp"
#include "math_tools.hpp"

class SlopePlanner : public VelocityPlannerBase {

public:
  using planner_status = VelocityPlannerBase::planner_status;
  using constraints = VelocityPlannerBase::constraints;
  using output = VelocityPlannerBase::output;

  SlopePlanner() = default;

  /**
   * @brief 配置规划器约束项
   *
   * @param limits
   * @return planner_status
   */
  planner_status Configure(const constraints &limits) override {
    if (limits.max_velocity <= 0.0f || limits.max_acceleration <= 0.0f ||
        limits.max_deceleration <= 0.0f) {
      return planner_status::InvalidArgument;
    }

    limits_ = limits;
    is_configured_ = true;
    is_initialized_ = false;

    target_velocity_ = Clamp<float>(target_velocity_, -limits_.max_velocity,
                                    limits_.max_velocity);

    return planner_status::OK;
  }

  planner_status SetTarget(float target_velocity) override {
    if (!is_configured_) {
      return planner_status::NotConfigured;
    }

    target_velocity_ = Clamp<float>(target_velocity, -limits_.max_velocity,
                                    limits_.max_velocity);
    // 小于阈值，表示到达
    output_.reached =
        Abs(target_velocity_ - output_.velocity) <= velocity_epsilon_;
    return planner_status::OK;
  }

  /**
   * @brief
   * 重规划，存入开始规划前的速度和目标速度，完成速度规划，再通过update拿到规划的output值
   *        注意这里传的不是每时每刻的实际速度，而是一次动作的初始速度
   *
   * @param new_target
   * @param current_velocity
   * @return planner_status
   */
  planner_status Replan(float new_target, float current_velocity) override {
    if (!is_configured_) {
      return planner_status::NotConfigured;
    }

    // 对输入的当前速度做限幅
    output_.velocity = current_velocity;
    output_.acceleration = 0.0f;
    output_.reached = false;
    is_initialized_ = true;

    // 设置目标值
    return SetTarget(new_target);
  }

  planner_status Update(float dt, output &result) override {
    if (!is_configured_) {
      return planner_status::NotConfigured;
    }

    if (!is_initialized_) {
      return planner_status::NotInitialized;
    }
    if (dt <= 0.0f) {
      return planner_status::InvalidDeltaTime;
    }

    const float error = target_velocity_ - output_.velocity;

    // 速度差达到阈值，直接覆写，完成规划
    if (Abs(error) <= velocity_epsilon_) {
      output_.velocity = target_velocity_;
      output_.acceleration = 0.0f;
      output_.reached = true;

      result = output_;
      return planner_status::OK;
    }

    // 判断是加速任务or减速任务
    float delta = 0.0f;

    // For a reversing target, brake exactly to zero first. Acceleration in the
    // opposite direction starts on the next update, so one step never crosses
    // zero using the deceleration limit.
    if (output_.velocity * target_velocity_ < 0.0f) {
      const float max_delta = limits_.max_deceleration * dt;

      if (Abs<float>(output_.velocity) <= max_delta) {
        delta = -output_.velocity;
      } else {
        const float direction = output_.velocity > 0.0f ? 1.0f : -1.0f;
        delta = -direction * max_delta;
      }
    } else {
      const bool accelerating =
          is_accelerating(output_.velocity, target_velocity_);
      const float acceleration_limit =
          accelerating ? limits_.max_acceleration : limits_.max_deceleration;
      const float max_delta = acceleration_limit * dt;
      delta = Clamp<float>(error, -max_delta, max_delta);
    }

    output_.velocity += delta;
    output_.acceleration = delta / dt;
    output_.reached =
        Abs(target_velocity_ - output_.velocity) <= velocity_epsilon_;

    if (output_.reached) {
      output_.velocity = target_velocity_;
    }

    result = output_;
    return planner_status::OK;
  }

  [[nodiscard]] float GetTarget() const override { return target_velocity_; }

  [[nodiscard]] output GetOutput() const override { return output_; }

  planner_status SetVelocityEpsilon(float velocity_epsilon) {
    if (velocity_epsilon < 0.0f) {
      return planner_status::InvalidArgument;
    }

    velocity_epsilon_ = velocity_epsilon;
    return planner_status::OK;
  }

private:
  static bool is_accelerating(float current, float target) {
    // 同向且目标绝对值更大，才算加速
    // 反向和减小绝对值都先按减速处理
    return current * target >= 0.0f && Abs<float>(target) > Abs<float>(current);
  }

private:
  // 速度差阈值，每个不同的joint都可单独设置
  float velocity_epsilon_{1.0e-4f};
};
