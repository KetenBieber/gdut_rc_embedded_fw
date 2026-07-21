/**
 * @file ChassisBase.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 底盘控制器基类
 * @version 0.1
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 底盘实际状态观测可以用一个观测器进行观测，观测器可以融合多方面数据
 *         底盘控制可以分为两种思路：速控思路 和 力控思路
 *         力控思路可以更好加入整车功率控制的环路，当然也方便对物理世界建模，以便做一些力补偿算法
 * @versioninfo :
 */
#pragma once

class ChassisBase {
public:
  // 平面底盘运动指令
  struct Velocity_Twist {
    float vx;
    float vy;
    float az;
  };

  explicit ChassisBase();

  ChassisBase(const ChassisBase &) = delete;
  ChassisBase &operator=(const ChassisBase &) = delete;

  // 初始化底盘工作
  virtual void Init() = 0;

  // 运动学逆解算
  virtual void InverseKinematicCal() = 0;

  // 底盘看门狗(如果有离线电机进行部分安全策略，或者上线日志提醒)
  virtual void ChassisWatchDog() = 0;

  Velocity_Twist get_current_cmd() const { return current_cmd_; }

  Velocity_Twist get_current_velocity() const { return current_velocity_; }

protected:
  Velocity_Twist current_cmd_{};      // 当前指令
  Velocity_Twist current_velocity_{}; // 底盘当前状态

  // 底盘是否全部在线
  bool is_online_{false};
};