/*
 * This file is part of the ct-Bot motor controller firmware.
 * Copyright (c) 2021-2022 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 *
 * PIO code based on https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/Rotary_encoder
 */

/**
 * @file    rotary_encoder.h
 * @brief   Rotary encoder for ct-Bot v2
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#pragma once

#include "hardware/irq.h"

#include <cstdint>


namespace ctbot {

class RotaryEncoder {
private:
    static constexpr bool DEBUG_ { false };

public:
    RotaryEncoder(const uint32_t pin_a, const bool direction);

    void reset();

    auto get_counts() const {
        return direction_ ? data_.counts : -data_.counts;
    }

    void set_counts(const int32_t counts) {
        data_.counts = direction_ ? counts : -counts;
    }

    auto get_dt() const {
        return data_.dt;
    }

    void reset_time();

    int32_t get_rpm();

    int32_t get_rpm_time();

protected:
    static constexpr uint8_t PIO_ID_ { 0 };
    static constexpr uint32_t COUNTS_PER_ROTATION_ { 12 };

    struct data_t {
        int32_t counts;
        bool dir;
        uint32_t time;
        uint32_t dt;
    };

    static inline uint8_t next_id_ {};
    static inline data_t* p_data_[2] {};
    static inline uint32_t pio_offset_ {};

    static void irq_handler();

    static inline void mask_interrupt() {
        irq_set_mask_enabled(PIO_ID_ ? (1U << PIO1_IRQ_0) : (1U << PIO0_IRQ_0), false);
    }

    static inline void unmask_interrupt() {
        irq_set_mask_enabled(PIO_ID_ ? (1U << PIO1_IRQ_0) : (1U << PIO0_IRQ_0), true);
    }

    const bool direction_;
    data_t data_;
    uint32_t last_rpm_us_;
    int32_t last_rot_;
};

} // namespace ctbot
