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

#pragma once

#include "rotary_encoder.h"
#include "motor.h"

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <array>


class Pid;

namespace ctbot {

/**
 * @brief Motor speed controller
 *
 * @startuml{SpeedControl.png}
 *  !include speed_control.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class SpeedControl {
    static constexpr uint8_t DEBUG_LEVEL_ { 0 }; // 0: off, 1: info, 2: debug, 3: noisy

protected:
    static constexpr size_t MAX_INSTANCES_ { 2 };
    static constexpr float INITIAL_KP_ { 0.5f };
    static constexpr float INITIAL_KI_ { 8.f };
    static constexpr float INITIAL_KD_ { 0.f };
    static constexpr uint32_t SAMPLE_TIME_MS_ { 50 }; /**< Sample time of PID controller in ms */
    static constexpr uint32_t DATA_PUBLISH_TIME_MS_ { 10 }; /**< Time between data published over uart in ms */
    static constexpr float CURRENT_MEASUREMENT_ALPHA_ { 0.01f };
    static constexpr float OPAMP_AMPLIFICATION_ { 50.f }; // V/V
    static constexpr float SHUNT_MOHM_ { 100.f }; // mOhm
    static constexpr float ADC_VREF_MV_ { 3'300.f }; // mV
    static constexpr uint32_t ADC_RES_BIT_ { 12 }; // bit
    static constexpr bool ADC_SUBTRACT_OFFSET_ { false };
    static constexpr float CURRENT_CONVERSION_FACTOR_ = ADC_VREF_MV_ / ((1 << ADC_RES_BIT_) - 1.f) / (OPAMP_AMPLIFICATION_ * (SHUNT_MOHM_ / 1'000.f)); // mA
    static constexpr uint8_t TASK_CORE_ID_ { 0 };
    static constexpr uint8_t TASK_PRIORITY_ { 4 };
    static constexpr uint32_t TASK_STACK_SIZE_ { 1024 }; /**< Stack size of controller task in byte */

    static std::array<SpeedControl*, MAX_INSTANCES_> controller_list_; /**< List of all SpeedControl instances created */
    static uint8_t registered_instances_;
    static uint8_t adc_pin_;
    static float motor_current_ma_;
    static uint16_t adc_offset_;

    bool direction_;
    float setpoint_, input_, output_;
    float kp_, ki_, kd_;
    bool enabled_;
    int32_t enc_rpm_;
    int32_t enc_counts_;
    Pid* p_pid_controller_;
    RotaryEncoder& encoder_;
    Motor& motor_;

    /**
     * @brief Perform the PID controller update step
     * @note Is called periodically by the speed controller task every TASK_PERIOD_MS ms
     */
    void run(const uint32_t time_ms);

public:
    static constexpr uint32_t MAX_SPEED { 200 }; /**< Maximum possible speed in rpm */
    static constexpr uint32_t GB_RATIO { 150 };

    struct EncData {
        const uint8_t start;
        int16_t rpm[MAX_INSTANCES_];
        int32_t counts[MAX_INSTANCES_];
        uint16_t motor_current;
        uint32_t crc;

        constexpr EncData() : start { 0xaa }, rpm {}, counts {}, motor_current {}, crc {} {}
    } __attribute__((__packed__));

    struct SpeedData {
        const uint8_t start;
        uint8_t cmd;
        float speed[MAX_INSTANCES_];
        uint32_t crc;

        constexpr SpeedData() : start { 0xaa }, cmd {}, speed {}, crc {} {}
    } __attribute__((__packed__));

    /**
     * @brief Speed controller task implementation
     * @note Calls run() on all SpeedControl instances
     */
    static void controller(void*);

    static size_t get_enc_data(EncData& data);

    static bool set_speed(const SpeedData& data);

    static uint16_t get_motor_current() {
        return static_cast<uint16_t>(motor_current_ma_);
    }

    /**
     * @brief Construct a new SpeedControl object
     * @param[in] enc: Reference to encoder to use for controller input (speed)
     * @param[in] motor: Reference to motor driver to use for controller output (PWM duty cycle)
     */
    SpeedControl(RotaryEncoder& enc, Motor& motor, uint8_t adc_pin = 0);

    /**
     * @brief Destroy the Speed Control object
     */
    ~SpeedControl();

    void reset();

    /**
     * @brief Set a new desired speed
     * @param[in] speed: Speed to set as percentage value; [-100; +100]
     */
    void set_speed(const float speed) {
        auto abs_speed { std::fabs(speed) };
        if (abs_speed > 100.f) {
            abs_speed = 100.f;
        }
        setpoint_ = abs_speed * (GB_RATIO * MAX_SPEED / 100.f); // abs_speed is in %
        direction_ = speed >= 0.f;
    }

    /**
     * @return Current speed set
     */
    auto get_speed() const {
        return setpoint_ / (direction_ ? GB_RATIO * MAX_SPEED / 100.f : GB_RATIO * MAX_SPEED / -100.f); // convert speed to %
    }

    /**
     * @return Current motor speed as reported by related encoder
     */
    auto get_enc_rpm() const {
        return enc_rpm_;
    }

    /**
     * @return Current Kp parameter setting
     */
    auto get_kp() const {
        return kp_;
    }

    /**
     * @return Current Ki parameter setting
     */
    auto get_ki() const {
        return ki_;
    }

    /**
     * @return Current Kd parameter setting
     */
    auto get_kd() const {
        return kd_;
    }

    /**
     * @brief Set new parameters for underlying PID controller
     * @param[in] kp: Proportional tuning parameter
     * @param[in] ki: Integral tuning parameter
     * @param[in] kd: Derivative tuning parameter
     */
    void set_parameters(const float kp, const float ki, const float kd);

    void set_enable(bool value) {
        enabled_ = value;
    }

protected:
    static bool check_crc(const SpeedData& data);
};

} // namespace ctbot
