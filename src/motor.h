/*
 * This file is part of the ct-Bot motor controller firmware.
 * Copyright (c) 2021-2022 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    motor.h
 * @brief   Motor driver
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#pragma once

#include "rotary_encoder.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Motor driver
 */
class Motor {
protected:
    static constexpr uint32_t PWM_FREQUENCY { 20'000 }; /**< Pwm frequency in Hz */

    int32_t pwm_;
    const uint8_t pwm_pin_;
    const uint8_t dir_pin_;
    const bool invert_dir_;

public:
    static constexpr uint16_t PWM_MAX { 6'250 };

    /**
     * @brief Construct a new Motor object
     * @param[in] pin_pwm: Pin number of the pwm signal, only used for initialization
     * @param[in] pin_dir: Pin number of the direction signal
     * @param[in] invert: Invert motor direction setting; set to true, if wheel turning direction should be inverted
     */
    Motor(const uint8_t pin_pwm, const uint8_t pin_dir, const bool invert);

    /**
     * @brief Set a new pwm value
     * @param[in] pwm: New pwm valueto set; [- PWM_MAX; PWM_MAX]
     */
    void set_pwm(const int32_t pwm);

    /**
     * @brief Set a new pwm duty cycle relative to max speed
     * @param[in] pwm_rel: New pwm duty cycle as ratio of max speed; [-1; +1]
     */
    void set_duty(const float pwm_rel) {
        set_pwm(static_cast<int32_t>(pwm_rel * PWM_MAX));
    }

    /**
     * @return Current pwm value set; [- PWM_MAX; PWM_MAX]
     */
    auto get_pwm() const {
        return pwm_;
    }
};

} // namespace ctbot
