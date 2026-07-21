/**
 * @file PWMInterface.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
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

#include "stm32h7xx_hal.h"

#include <cstdint>

class PWMInterface {
public:
  enum class pwm_channel : uint8_t {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
  };

  enum class pwm_status : uint8_t {
    OK,
    InvalidArgument,
    InvalidChannel,
    UnsupportedChannel,
    HalError,
    HalBusy,
    HalTimeout,
  };

  explicit PWMInterface(TIM_HandleTypeDef &impl) : impl_(impl) {}

  PWMInterface(const PWMInterface &) = delete;
  PWMInterface &operator=(const PWMInterface &) = delete;

  pwm_status Start(pwm_channel channel) {
    uint32_t hal_channel = 0U;
    const pwm_status status = ValidateChannel(channel, hal_channel);
    if (status != pwm_status::OK) {
      return status;
    }

    return FromHalStatus(HAL_TIM_PWM_Start(&impl_, hal_channel));
  }

  pwm_status Stop(pwm_channel channel) {
    uint32_t hal_channel = 0U;
    const pwm_status status = ValidateChannel(channel, hal_channel);
    if (status != pwm_status::OK) {
      return status;
    }

    return FromHalStatus(HAL_TIM_PWM_Stop(&impl_, hal_channel));
  }

  pwm_status SetCompare(pwm_channel channel, uint32_t compare) {
    uint32_t hal_channel = 0U;
    const pwm_status status = ValidateChannel(channel, hal_channel);
    if (status != pwm_status::OK) {
      return status;
    }

    if ((static_cast<uint64_t>(compare) > get_period_ticks()) ||
        (compare > get_max_counter_value())) {
      return pwm_status::InvalidArgument;
    }

    __HAL_TIM_SET_COMPARE(&impl_, hal_channel, compare);
    return pwm_status::OK;
  }

  // Changes ARR for the whole timer. Existing CCR values are not rescaled.
  pwm_status SetPeriodTicks(uint32_t period_ticks) {
    if ((period_ticks == 0U) || ((static_cast<uint64_t>(period_ticks) - 1U) >
                                 get_max_counter_value())) {
      return pwm_status::InvalidArgument;
    }

    __HAL_TIM_SET_AUTORELOAD(&impl_, period_ticks - 1U);
    return pwm_status::OK;
  }

  // duty range: 0..1000. The result is rounded to the nearest timer tick.
  pwm_status SetDutyPermille(pwm_channel channel, uint16_t duty) {
    if (duty > 1000U) {
      return pwm_status::InvalidArgument;
    }

    const uint64_t compare = (get_period_ticks() * duty + 500U) / 1000U;

    // At full ARR range, ARR + 1 cannot be represented by CCR. Reject this
    // special 100% case instead of allowing it to wrap to zero in hardware.
    if (compare > get_max_counter_value()) {
      return pwm_status::InvalidArgument;
    }

    return SetCompare(channel, static_cast<uint32_t>(compare));
  }

  // duty range: 0..100.
  pwm_status SetDutyPercent(pwm_channel channel, uint8_t duty) {
    if (duty > 100U) {
      return pwm_status::InvalidArgument;
    }

    return SetDutyPermille(channel, static_cast<uint16_t>(duty) * 10U);
  }

  [[nodiscard]] uint32_t get_current_ccr(pwm_channel channel) const {
    uint32_t hal_channel = 0U;
    if (ValidateChannel(channel, hal_channel) != pwm_status::OK) {
      return 0U;
    }

    return __HAL_TIM_GET_COMPARE(&impl_, hal_channel);
  }

  // uint64_t is required to represent 2^32 ticks when ARR == UINT32_MAX.
  [[nodiscard]] uint64_t get_period_ticks() const {
    return static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(&impl_)) + 1U;
  }

  // Forces an update event and may truncate the current PWM period.
  pwm_status ApplyUpdate() {
    return FromHalStatus(HAL_TIM_GenerateEvent(&impl_, TIM_EVENTSOURCE_UPDATE));
  }

private:
  [[nodiscard]] static bool ToHalChannel(pwm_channel channel,
                                         uint32_t &hal_channel) {
    switch (channel) {
    case pwm_channel::CHANNEL1:
      hal_channel = TIM_CHANNEL_1;
      return true;
    case pwm_channel::CHANNEL2:
      hal_channel = TIM_CHANNEL_2;
      return true;
    case pwm_channel::CHANNEL3:
      hal_channel = TIM_CHANNEL_3;
      return true;
    case pwm_channel::CHANNEL4:
      hal_channel = TIM_CHANNEL_4;
      return true;
    case pwm_channel::CHANNEL5:
      hal_channel = TIM_CHANNEL_5;
      return true;
    case pwm_channel::CHANNEL6:
      hal_channel = TIM_CHANNEL_6;
      return true;
    }

    return false;
  }

  [[nodiscard]] pwm_status ValidateChannel(pwm_channel channel,
                                           uint32_t &hal_channel) const {
    if (!ToHalChannel(channel, hal_channel)) {
      return pwm_status::InvalidChannel;
    }

    if (!IS_TIM_CCX_INSTANCE(impl_.Instance, hal_channel)) {
      return pwm_status::UnsupportedChannel;
    }

    return pwm_status::OK;
  }

  [[nodiscard]] uint32_t get_max_counter_value() const {
    return IS_TIM_32B_COUNTER_INSTANCE(impl_.Instance) ? UINT32_MAX
                                                       : UINT16_MAX;
  }

  [[nodiscard]] static pwm_status FromHalStatus(HAL_StatusTypeDef status) {
    switch (status) {
    case HAL_OK:
      return pwm_status::OK;
    case HAL_BUSY:
      return pwm_status::HalBusy;
    case HAL_TIMEOUT:
      return pwm_status::HalTimeout;
    case HAL_ERROR:
    default:
      return pwm_status::HalError;
    }
  }

  TIM_HandleTypeDef &impl_;
};
