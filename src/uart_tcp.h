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
 * @file    uart_tcp.h
 * @brief   Uart to TCP server for LwIP and FreeRTOS
 * @author  Timo Sandmann
 * @date    30.07.2022
 */


#pragma once

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include <cstdint>
#include <array>
#include <atomic>


class TcpServer;
class NtpClient;

class UartTcp {
    static constexpr bool DEBUG_ { false };

    static constexpr uint8_t MAX_INSTANCES_ { 2 };
    static constexpr size_t UART_RX_BUFFER_SIZE_ { 8192 };
    static constexpr size_t TCP_RX_BUFFER_SIZE_ { 32 };
    static constexpr size_t TCP_TX_BUFFER_SIZE_ { 256 };
    static constexpr uint8_t TASK_PRIORITY_ { 2 };
    static constexpr size_t STACK_SIZE_ { 512 };

    uart_inst_t* p_uart_;
    std::atomic<bool> running_;
    TaskHandle_t p_worker_;

protected:
    static constexpr uint8_t WIFI_CORE_ID_ { 1 };

    static inline std::array<UartTcp*, MAX_INSTANCES_> p_instances_;

    std::array<char, 32> uart_in_buffer_;
    StreamBufferHandle_t uart_rx_stream_;
    std::array<uint8_t, TCP_RX_BUFFER_SIZE_>* p_tcp_rx_buffer_;
    std::array<char, TCP_TX_BUFFER_SIZE_>* p_tcp_tx_buffer_;
    std::array<char, 20> time_buffer_;
    TcpServer* p_tcp_server_;
    NtpClient* p_ntp_;
    size_t min_rx_buffer_free_;

    void uart_isr();
    void run();

public:
    UartTcp(uart_inst_t* p_uart, uint8_t rx_pin, uint8_t tx_pin, size_t baudrate);
    ~UartTcp();

    bool init();

    void stop() {
        running_ = false;
    }
};
