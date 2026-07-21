/**
 * @file math_tools.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 一些数学工具
 * @version 0.1
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <cstdint>

template <typename T> T Abs(T val) { return val >= static_cast<T>(0) ? val : -val; }

template <typename T> T Clamp(T val, T min, T max) {
  if (val < min) {
    return min;
  }

  if (val > max) {
    return max;
  }

  return val;
}
