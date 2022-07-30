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
 * @file    motor.cpp
 * @brief   Motor driver
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "motor.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <cmath>
#include <cstdio>


namespace ctbot {

Motor::Motor(const uint8_t pin_pwm, const uint8_t pin_dir, const bool invert) : pwm_ {}, pwm_pin_ { pin_pwm }, dir_pin_ { pin_dir }, invert_dir_ { invert } {
    gpio_init(dir_pin_);
    gpio_disable_pulls(dir_pin_);
    gpio_set_dir(dir_pin_, true);
    gpio_put(dir_pin_, false);

    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);
    const auto slice_num { pwm_gpio_to_slice_num(pwm_pin_) };
    pwm_set_clkdiv(slice_num, 1.f);
    pwm_set_wrap(slice_num, PWM_MAX - 1);
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(pwm_pin_, 0);
    set_pwm(0L);
}

void Motor::set_pwm(int32_t new_pwm) {
    pwm_ = std::abs(new_pwm) > PWM_MAX ? (new_pwm < 0 ? -PWM_MAX : PWM_MAX) : new_pwm;

    if (invert_dir_) {
        new_pwm = -new_pwm;
    }
    gpio_put(dir_pin_, new_pwm >= 0);

    pwm_set_gpio_level(pwm_pin_, static_cast<uint16_t>(std::abs(pwm_)));
}

} // namespace ctbot
