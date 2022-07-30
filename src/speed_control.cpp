/*
 * This file is part of the ct-Bot motor controller firmware.
 * Copyright (c) 2018-2022 Timo Sandmann
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
 * @file    speed_control.h
 * @brief   Motor speed controller
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "speed_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pid_v1.h"
#include "crc32.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"

#include <cstring>
#include <cstdio>


namespace ctbot {

std::array<SpeedControl*, SpeedControl::MAX_INSTANCES_> SpeedControl::controller_list_ {};
uint8_t SpeedControl::registered_instances_ {};
uint8_t SpeedControl::adc_pin_ {};
float SpeedControl::motor_current_ma_ {};
uint16_t SpeedControl::adc_offset_ {};

SpeedControl::SpeedControl(RotaryEncoder& enc, Motor& motor, uint8_t adc_pin)
    : direction_ { true }, setpoint_ {}, input_ {}, output_ {}, kp_ { INITIAL_KP_ }, ki_ { INITIAL_KI_ }, kd_ { INITIAL_KD_ }, enc_rpm_ {}, enc_counts_ {},
      p_pid_controller_ { new Pid { input_, output_, setpoint_, kp_, ki_, kd_, true } }, encoder_ { enc }, motor_ { motor } {
    if (!p_pid_controller_) {
        return;
    }

    reset();

    for (auto& p_ctrl : controller_list_) {
        if (!p_ctrl) {
            p_ctrl = this;
            ++registered_instances_;
            break;
        }
    }

    if (adc_pin && !adc_pin_) {
        adc_pin_ = adc_pin;
        adc_init();
        adc_set_temp_sensor_enabled(false);
        adc_gpio_init(adc_pin_);
        adc_select_input(adc_pin_ - 26);
    }

    if (registered_instances_ == 1) {
        xTaskCreateAffinitySet(SpeedControl::controller, "sctrl", TASK_STACK_SIZE_ / sizeof(StackType_t), nullptr, TASK_PRIORITY_, 1 << TASK_CORE_ID_, nullptr);
    }
}

SpeedControl::~SpeedControl() {
    for (auto& p_ctrl : controller_list_) {
        if (p_ctrl == this) {
            p_ctrl = nullptr;
        }
    }

    delete p_pid_controller_;
}

void SpeedControl::reset() {
    motor_.set_pwm(0);

    encoder_.reset();

    enc_rpm_ = 0;
    enc_counts_ = 0;

    if (!p_pid_controller_) {
        return;
    }

    p_pid_controller_->set_sample_time(SAMPLE_TIME_MS_);
    p_pid_controller_->set_output_limits(0, MAX_SPEED * GB_RATIO);
    p_pid_controller_->set_mode(Pid::Modes::AUTOMATIC);
}

void SpeedControl::run(const uint32_t time_ms) {
    enc_rpm_ = encoder_.get_rpm_time();
    enc_counts_ = encoder_.get_counts();
    if (DEBUG_) {
        // printf("SpeedControl::run(%u): enc_rpm=%d enc_counts=%d\n", time_ms, enc_rpm_, enc_counts_);
    }
    input_ = std::abs(enc_rpm_);

    if (!p_pid_controller_ || !p_pid_controller_->compute(time_ms)) {
        return;
    }

    int32_t pwm;
    if (setpoint_ == 0.f) {
        pwm = 0;
    } else {
        pwm = static_cast<int32_t>(output_ / (static_cast<float>(MAX_SPEED * GB_RATIO) / static_cast<float>(Motor::PWM_MAX)));
    }

    if (motor_.get_pwm() != (direction_ ? pwm : -pwm)) {
        motor_.set_pwm(direction_ ? pwm : -pwm);
        if (DEBUG_) {
            printf("SpeedControl::run(%u): set pwm to %d\n", time_ms, direction_ ? pwm : -pwm);
        }
    }
}

void SpeedControl::set_parameters(const float kp, const float ki, const float kd) {
    if (!p_pid_controller_) {
        return;
    }

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    p_pid_controller_->set_tunings(kp_, ki_, kd_);
}

void SpeedControl::controller(void*) {
    char* speed_buffer = new char[sizeof(SpeedControl::SpeedData)];
    configASSERT(speed_buffer);
    uint32_t last_enc_sent_ms {};

    uart_init(uart0, 1'000'000);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_set_translate_crlf(uart0, false);
#endif
    gpio_set_function(0, GPIO_FUNC_UART); // TX
    gpio_set_function(1, GPIO_FUNC_UART); // RX

    {
        adc_select_input(adc_pin_ - 26);
        adc_fifo_setup(true, false, 0, 0, 0);
        adc_set_clkdiv(48'000'000UL / 1'500UL - 1UL); // 1'500 samples / s
        vTaskDelay(pdMS_TO_TICKS(50));
        adc_fifo_drain();
        adc_run(true);
        float adc_sum { static_cast<float>(adc_fifo_get_blocking()) };
        for (size_t i {}; i < 100; ++i) {
            adc_sum = (CURRENT_MEASUREMENT_ALPHA_ * adc_fifo_get_blocking()) + (1 - CURRENT_MEASUREMENT_ALPHA_) * adc_sum;
        }
        adc_offset_ = static_cast<uint16_t>(adc_sum - 10.f);
    }

    if (DEBUG_) {
        printf("SpeedControl::controller(): adc_offset=%u\n", adc_offset_);
    }
    motor_current_ma_ = 0;

    char* p_speed_buffer { speed_buffer };
    float last_speed[2] {};

    while (true) {
        TickType_t last_wake_time { xTaskGetTickCount() };
        const auto time_ms { time_us_32() / 1'000UL };

        while (uart_is_readable(uart0)) {
            while (uart_is_readable(uart0)) {
                *p_speed_buffer = uart_getc(uart0);
                ++p_speed_buffer;

                if (p_speed_buffer - speed_buffer >= sizeof(SpeedControl::SpeedData)) {
                    break;
                }
            }

            if (speed_buffer[0] != 0xaa) {
                std::memmove(&speed_buffer[0], &speed_buffer[1], sizeof(SpeedControl::SpeedData) - 1);
                --p_speed_buffer;
            } else if (p_speed_buffer - speed_buffer >= sizeof(SpeedControl::SpeedData)) {
                p_speed_buffer = speed_buffer;
                auto p_data { reinterpret_cast<const SpeedControl::SpeedData*>(speed_buffer) };
                switch (p_data->cmd) {
                    case 0:
                        if (SpeedControl::set_speed(*p_data)) {
                            if (p_data->speed[0] != last_speed[0] || p_data->speed[1] != last_speed[1]) {
                                if (DEBUG_) {
                                    printf("speed set to %f\t%f\n", p_data->speed[0], p_data->speed[1]);
                                }
                                last_speed[0] = p_data->speed[0];
                                last_speed[1] = p_data->speed[1];
                            }
                            controller_list_[0]->set_speed(p_data->speed[0]);
                            controller_list_[1]->set_speed(p_data->speed[1]);
                        } else {
                            if (DEBUG_) {
                                printf("ERROR: invalid speed data received:\n");
                                for (size_t i {}; i < sizeof(SpeedData); ++i) {
                                    printf("0x%x ", speed_buffer[i]);
                                }
                                printf("\n");
                            }
                        }
                        break;

                    case 1:
                        if (DEBUG_) {
                            printf("reset cmd received\n");
                        }
                        // FIXME: validate crc
                        controller_list_[0]->reset();
                        controller_list_[1]->reset();
                        break;

                    default:
                        if (DEBUG_) {
                            printf("ERROR: invalid command received: 0x%x\n", p_data->cmd);
                            for (size_t i {}; i < sizeof(SpeedData); ++i) {
                                printf("0x%x ", speed_buffer[i]);
                            }
                            printf("\n");
                        }
                }
            }
        }

        for (auto p_ctrl : controller_list_) {
            if (p_ctrl) {
                p_ctrl->run(time_ms);
            }
        }

        if (adc_pin_) {
            size_t i {};
            const auto n { adc_fifo_get_level() };
            for (; i < n; ++i) {
                int32_t adc_result { adc_fifo_get() - (ADC_SUBTRACT_OFFSET_ ? adc_offset_ : 0) };
                if (ADC_SUBTRACT_OFFSET_ && adc_result < 0) {
                    if (DEBUG_) {
                        printf("adc_result[%u]=%d\n", i, adc_result);
                    }
                    adc_result = 0;
                }
                motor_current_ma_ =
                    (CURRENT_MEASUREMENT_ALPHA_ * adc_result * CURRENT_CONVERSION_FACTOR_) + (1.f - CURRENT_MEASUREMENT_ALPHA_) * motor_current_ma_;
            }
        }

        if (time_ms - last_enc_sent_ms >= DATA_PUBLISH_TIME_MS_) {
            last_enc_sent_ms = time_ms;

            SpeedControl::EncData enc_data {};
            const auto n { SpeedControl::get_enc_data(enc_data) };
            if (n) {
                uart_write_blocking(uart0, reinterpret_cast<const uint8_t*>(&enc_data), sizeof(enc_data));
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

size_t SpeedControl::get_enc_data(EncData& data) {
    static int32_t last_counts[2];
    static uint16_t last_mcurrent_;
    bool new_data {};
    size_t i {};
    for (auto p_ctrl : controller_list_) {
        if (i >= MAX_INSTANCES_) {
            break;
        }
        if (p_ctrl && last_counts[i] != p_ctrl->enc_counts_) {
            data.rpm[i] = static_cast<int16_t>(p_ctrl->enc_rpm_);
            data.counts[i] = p_ctrl->enc_counts_;
            last_counts[i] = data.counts[i];
            new_data = true;
        }
        ++i;
    }

    data.motor_current = get_motor_current();

    if (!new_data && last_mcurrent_ == data.motor_current) {
        return 0;
    }

    CRC32 crc;
    auto ptr { reinterpret_cast<const uint8_t*>(&data) };
    data.crc = CRC32::calculate(ptr, sizeof(EncData) - sizeof(data.crc));

    return i;
}

bool SpeedControl::set_speed(const SpeedData& data) {
    auto ptr { reinterpret_cast<const uint8_t*>(&data) };
    const auto checksum { CRC32::calculate(ptr, sizeof(data) - sizeof(data.crc)) };
    if (checksum != data.crc) {
        for (auto p_ctrl : controller_list_) {
            if (p_ctrl) {
                p_ctrl->set_speed(0.f);
            }
        }

        printf("speed=%f\t%f\tchecksum=%lu\tcrc=%lu\n", data.speed[0], data.speed[1], checksum, data.crc);
        return false;
    }

    for (auto p_ctrl : controller_list_) {
        size_t i {};
        if (p_ctrl) {
            p_ctrl->set_speed(data.speed[i]);
        }
        ++i;
    }

    return true;
}

} // namespace ctbot
