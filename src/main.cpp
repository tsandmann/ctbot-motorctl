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
 */

/**
 * @file    main.cpp
 * @brief   Motor controller with rotary encoder for ct-Bot v2
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "rotary_encoder.h"
#include "speed_control.h"
#include "uart_tcp.h"
#include "freertos_runtime_stats.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "pico/stdlib.h"

#include <cstdint>
#include <cstdio>


static constexpr bool DEBUG { true };
static constexpr uint32_t DEBUG_CYCLE_TIME_MS { 500 };

static constexpr uint8_t INIT_CORE_ID { 0 };

static constexpr uint8_t UART1_RX_PIN { 9 };
static constexpr uint8_t UART1_TX_PIN { 8 };
static constexpr uint8_t ENC_L_PIN_A { 16 };
static constexpr uint8_t ENC_L_PIN_B { 17 };
static constexpr uint8_t ENC_R_PIN_A { 18 };
static constexpr uint8_t ENC_R_PIN_B { 19 };
static constexpr uint8_t MOTOR_L_PIN_PWM { 6 };
static constexpr uint8_t MOTOR_L_PIN_DIR { 7 };
static constexpr uint8_t MOTOR_R_PIN_PWM { 14 };
static constexpr uint8_t MOTOR_R_PIN_DIR { 15 };
static constexpr uint8_t ADC_IN_PIN { 28 };
static constexpr uint32_t MOTOR_CURRENT_TRESHOLD { 4 }; // mA

using ctbot::Motor;
using ctbot::RotaryEncoder;
using ctbot::SpeedControl;


static RotaryEncoder* encoders[2];
static Motor* motors[2];
static SpeedControl* speedcontrols[2];

static UartTcp* server;


static void setup_task(void*) {
    vTaskCoreAffinitySet(xTimerGetTimerDaemonTaskHandle(), 1 << INIT_CORE_ID);

    server = new UartTcp { uart1, UART1_RX_PIN, UART1_TX_PIN, 1'000'000 };
    configASSERT(server);

    while (!server->init()) {
        if ((DEBUG)) {
            printf("server->init() failed, retrying in 300 s ...\n");
        }
        vTaskDelay(pdMS_TO_TICKS(300'000UL));
    }

    vTaskCoreAffinitySet(xTaskGetHandle("tcpip_thread"), 1 << 1);

    encoders[0] = new RotaryEncoder { ENC_L_PIN_A, false };
    encoders[1] = new RotaryEncoder { ENC_R_PIN_A, true };
    configASSERT(encoders[0] && encoders[1]);

    motors[0] = new Motor { MOTOR_L_PIN_PWM, MOTOR_L_PIN_DIR, false };
    motors[1] = new Motor { MOTOR_R_PIN_PWM, MOTOR_R_PIN_DIR, true };
    configASSERT(motors[0] && motors[1]);

    speedcontrols[0] = new SpeedControl { *encoders[0], *motors[0], ADC_IN_PIN };
    speedcontrols[1] = new SpeedControl { *encoders[1], *motors[1], ADC_IN_PIN };
    configASSERT(speedcontrols[0] && speedcontrols[1]);

    vTaskPrioritySet(nullptr, 1);

    FreeRTOSRuntimeStats* p_stats {};
    if (DEBUG) {
        p_stats = new FreeRTOSRuntimeStats;
        configASSERT(p_stats);
    }

    while (DEBUG) {
        if (speedcontrols[0]->get_enc_rpm() || speedcontrols[1]->get_enc_rpm()) {
            const auto rpm_0 { speedcontrols[0]->get_enc_rpm() };
            const auto rpm_gb_0 { rpm_0 / static_cast<int32_t>(SpeedControl::GB_RATIO / 10) };
            printf("0: rpm motor=%6ld\trpm gb_10=%5ld\t%4ld mm/s\n", rpm_0, rpm_gb_0, rpm_gb_0 * 3 / 10);
            // printf("   dt=%lu\n", encoders[0]->get_dt());
            // const auto rpm_0_enc { encoders[0]->get_rpm() };
            // printf("   rpm motor=%ld\trpm gb_10=%ld\n", rpm_0_enc, rpm_0_enc / static_cast<int32_t>(SpeedControl::GB_RATIO / 10));

            const auto rpm_1 { speedcontrols[1]->get_enc_rpm() };
            const auto rpm_gb_1 { rpm_1 / static_cast<int32_t>(SpeedControl::GB_RATIO / 10) };
            printf("1: rpm motor=%6ld\trpm gb_10=%5ld\t%4ld mm/s\n", rpm_1, rpm_gb_1, rpm_gb_1 * 3 / 10);
            // printf("   dt=%lu\n", encoders[1]->get_dt());
            // const auto rpm_1_enc { encoders[1]->get_rpm() };
            // printf("   rpm motor=%ld\trpm gb_10=%ld\n", rpm_1_enc, rpm_1_enc / static_cast<int32_t>(SpeedControl::GB_RATIO / 10));
        }

        if (ADC_IN_PIN) {
            const auto current { SpeedControl::get_motor_current() };
            if (current >= MOTOR_CURRENT_TRESHOLD) {
                printf("Motor current: %u mA\n", current);
            }
        }

        p_stats->print();

        vTaskDelay(pdMS_TO_TICKS(DEBUG_CYCLE_TIME_MS));
    }

    vTaskDelete(nullptr);
}

int main() {
    stdio_init_all();

    if (DEBUG) {
        busy_wait_ms(2'000);
        printf("\n\nRunning FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ ".\n\n");
    }

    xTaskCreateAffinitySet(setup_task, "setup", 512, nullptr, 2, 1 << INIT_CORE_ID, nullptr);

    if (DEBUG) {
        printf("main(): starting scheduler...\n\n");
    }

    vTaskStartScheduler();

    configASSERT(false);
    return 0;
}
