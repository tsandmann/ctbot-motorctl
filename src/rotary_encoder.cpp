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
 * @file    rotary_encoder.cpp
 * @brief   Rotary encoder for ct-Bot v2
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "rotary_encoder.h"
#include "rotary_encoder.pio.h"

#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include <cstdio>


namespace ctbot {

RotaryEncoder::RotaryEncoder(const uint32_t pin_a, const bool direction) : direction_ { direction }, data_ { 0, true, 0, 0 }, last_rpm_us_ {}, last_rot_ {} {
    static_assert(PIO_ID_ < 2, "invalid PIO_ID_");

    if (next_id_ > 1) {
        if (DEBUG_) {
            printf("RotaryEncoder::RotaryEncoder(): no more ressources available.\n");
        }
        return;
    }

    if (DEBUG_) {
        printf("RotaryEncoder::RotaryEncoder(): id=%u\n", next_id_);
    }

    p_data_[next_id_] = &this->data_;

    PIO const pio { PIO_ID_ ? pio1 : pio0 };
    const auto sm { pio_claim_unused_sm(pio, false) };
    if (DEBUG_) {
        printf("RotaryEncoder::RotaryEncoder(): pio %u sm=%d\n", PIO_ID_, sm);
    }
    if (sm < 0) {
        if (DEBUG_) {
            printf("RotaryEncoder::RotaryEncoder(): unable to claim pio sm.\n");
        }
        return;
    }

    if (next_id_ == 0) {
        if (!pio_can_add_program(pio, &rotary_encoder_program)) {
            if (DEBUG_) {
                printf("RotaryEncoder::RotaryEncoder(): unable to load pio program.\n");
            }
            pio_sm_unclaim(pio, sm);
            return;
        }

        /* load the pio program into the pio memory */
        pio_offset_ = pio_add_program(pio, &rotary_encoder_program);
        if (DEBUG_) {
            printf("RotaryEncoder::RotaryEncoder(): pio %u offset=%u\n", PIO_ID_, pio_offset_);
        }

        const uint irq_num { static_cast<uint>(PIO_ID_ ? PIO1_IRQ_0 : PIO0_IRQ_0) };
        irq_set_exclusive_handler(irq_num, irq_handler);
        irq_set_priority(irq_num, 0);
        irq_set_enabled(irq_num, true);
        pio->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS | PIO_IRQ0_INTE_SM2_BITS | PIO_IRQ0_INTE_SM3_BITS;
    }
    ++next_id_;

    rotary_encoder_init(pio, sm, pio_offset_, pin_a);
}

void RotaryEncoder::reset() {
    mask_interrupt();
    data_.counts = 0;
    data_.dir = true;
    data_.time = 0;
    data_.dt = 0;
    unmask_interrupt();

    last_rpm_us_ = 0;
    last_rot_ = 0;
}

void RotaryEncoder::reset_time() {
    mask_interrupt();
    data_.time = time_us_32();
    data_.dt = 0;
    unmask_interrupt();
}

int32_t RotaryEncoder::get_rpm() {
    const auto now_us { time_us_32() };
    const auto rot { data_.counts };
    if (rot == last_rot_) {
        last_rpm_us_ = now_us;
        return 0;
    }

    const auto rot_diff { rot - last_rot_ };
    last_rot_ = rot;

    const auto dt_us { now_us - last_rpm_us_ };
    last_rpm_us_ = now_us;

    const auto rpm_factor { (1'000'000.f * 60.f / static_cast<float>(COUNTS_PER_ROTATION_)) / static_cast<float>(dt_us) };
    if (DEBUG_) {
        printf("diff=%ld\tdt=%lu us\n", rot_diff, data_.dt);
    }
    const auto rpm { static_cast<int32_t>(rot_diff * rpm_factor) };

    if (direction_) {
        return rpm;
    } else {
        return -rpm;
    }
}

int32_t RotaryEncoder::get_rpm_time() {
    if (!data_.dt) {
        return 0;
    }
    if (time_us_32() - data_.time > 300'000UL) {
        reset_time();
        return 0;
    }

    mask_interrupt();
    const auto dt { data_.dt };
    const auto dir { data_.dir };
    unmask_interrupt();

    const auto rpm { (1'000'000.f * 60.f) / static_cast<float>(dt) };
    const auto ret { static_cast<int32_t>(rpm) };
    if (direction_) {
        return dir ? ret : -ret;
    } else {
        return dir ? -ret : ret;
    }
}

void RotaryEncoder::irq_handler() {
    static uint8_t calls[2] {};
    const auto now { time_us_32() };

    const PIO p_pio { PIO_ID_ ? pio1 : pio0 };
    const auto irq_status { p_pio->irq };

    if (irq_status & 0x5) {
        /* SM 0 */
        switch (irq_status & 0x5) {
            case 1:
                /* clockwise */
                --p_data_[0]->counts;
                p_data_[0]->dir = false;
                break;

            case 4:
                /* anticlockwise */
                ++p_data_[0]->counts;
                p_data_[0]->dir = true;
                break;
        }

        if (++calls[0] == COUNTS_PER_ROTATION_) {
            p_data_[0]->dt = now - p_data_[0]->time;
            p_data_[0]->time = now;
            calls[0] = 0;
        }
    }

    if (irq_status & 0xa) {
        /* SM 1 */
        switch (irq_status & 0xa) {
            case 2:
                /* clockwise */
                --p_data_[1]->counts;
                p_data_[1]->dir = false;
                break;

            case 8:
                /* anticlockwise */
                ++p_data_[1]->counts;
                p_data_[1]->dir = true;
                break;
        }

        if (++calls[1] == COUNTS_PER_ROTATION_) {
            p_data_[1]->dt = now - p_data_[1]->time;
            p_data_[1]->time = now;
            calls[1] = 0;
        }
    }

    p_pio->irq = 0xf; // clear interrupt
}

} // namespace ctbot
