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
 * @file    uart_tcp.cpp
 * @brief   Uart to TCP server for LwIP and FreeRTOS
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "uart_tcp.h"
#include "tcp_server.h"
#include "ntp_client.h"
#include "wifi_config.h"

#include "pico/cyw43_arch.h"

#include <type_traits>
#include <charconv>
#include <limits>
#include <cstring>


UartTcp::UartTcp(uart_inst_t* p_uart, uint8_t rx_pin, uint8_t tx_pin, size_t baudrate)
    : p_uart_ { p_uart }, running_ {}, p_worker_ {}, p_tcp_server_ {}, p_ntp_ {}, min_rx_buffer_free_ { std::numeric_limits<size_t>::max() } {
    const auto uart_index { uart_get_index(p_uart_) };
    configASSERT(uart_index < p_instances_.size());
    p_instances_[uart_index] = this;

    uart_rx_stream_ = xStreamBufferCreate(UART_RX_BUFFER_SIZE_, 1);
    configASSERT(uart_rx_stream_);

    p_tcp_rx_buffer_ = new std::array<uint8_t, std::tuple_size<std::remove_reference<decltype(*p_tcp_rx_buffer_)>::type>::value>;
    configASSERT(p_tcp_rx_buffer_);

    p_tcp_tx_buffer_ = new std::array<char, std::tuple_size<std::remove_reference<decltype(*p_tcp_tx_buffer_)>::type>::value>;
    configASSERT(p_tcp_tx_buffer_);

    uart_init(p_uart_, baudrate);
    uart_set_hw_flow(p_uart_, false, false);
    uart_set_format(p_uart_, 8, 1, UART_PARITY_NONE);
#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_set_translate_crlf(p_uart_, false);
#endif
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_fifo_enabled(p_uart_, true);
    irq_set_exclusive_handler(
        uart_index == 0 ? UART0_IRQ : UART1_IRQ, uart_index == 0 ? []() { p_instances_[0]->uart_isr(); } : []() { p_instances_[1]->uart_isr(); });
    uart_set_irq_enables(p_uart_, true, false);
    irq_set_priority(uart_index == 0 ? UART0_IRQ : UART1_IRQ, 1 << 6); // 0, 64, 128, 192
    irq_set_enabled(uart_index == 0 ? UART0_IRQ : UART1_IRQ, true);

    while (uart_is_readable(p_uart_)) {
        uart_getc(p_uart_);
    }

    p_tcp_server_ = new TcpServer;
    configASSERT(p_tcp_server_);

    p_ntp_ = new NtpClient;
    configASSERT(p_ntp_);
}

UartTcp::~UartTcp() {
    running_ = false;
    volatile TaskHandle_t* p_task { &p_worker_ };
    while (*p_task) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    delete p_ntp_;
    delete p_tcp_server_;
    delete p_tcp_tx_buffer_;
    delete p_tcp_rx_buffer_;
}

bool UartTcp::init() {
    if (!p_tcp_server_->init(wifi_ssid, wifi_pass, WIFI_CORE_ID_)) {
        if constexpr (DEBUG_) {
            printf("UartTcp::init(): p_tcp_server_->init() failed, abort.\n");
        }
        return false;
    }

    if (!p_tcp_server_->open(23)) {
        if constexpr (DEBUG_) {
            printf("UartTcp::init(): p_tcp_server_->open() failed, abort.\n");
        }
        return false;
    }

    if (!p_ntp_->init()) {
        return false;
    }

    xTaskCreateAffinitySet(
        [](void* arg) {
            auto p_this { reinterpret_cast<UartTcp*>(arg) };
            p_this->run();
        },
        "UartTcp", STACK_SIZE_, this, TASK_PRIORITY_, 1 << WIFI_CORE_ID_, &p_worker_);

    configASSERT(p_worker_);

    return true;
}

void UartTcp::run() {
    running_ = true;

    uint32_t last_debug_time {};
    bool last_led_state {};
    while (running_) {
        if (auto client = p_tcp_server_->available()) {
            auto rx_len { p_tcp_server_->available(client) };
            if (rx_len > p_tcp_rx_buffer_->size()) {
                rx_len = p_tcp_rx_buffer_->size();
            }

            const auto received { p_tcp_server_->read(client, p_tcp_rx_buffer_->data(), rx_len) };
            if constexpr (DEBUG_) {
                p_tcp_rx_buffer_->data()[received] = 0;
                printf("%s", p_tcp_rx_buffer_->data());
            }

            auto p_recv_buffer = p_tcp_rx_buffer_->data();
            /* discard telnet commands during first second */
            while (*p_recv_buffer == 0xff && rx_len >= 3) {
                if (time_us_32() / 1'000UL < p_tcp_server_->get_connect_time(client) + 1'000UL) {
                    p_recv_buffer += 3;
                    rx_len -= 3;
                } else {
                    break;
                }
            }

            uart_write_blocking(p_uart_, p_recv_buffer, rx_len);
        }

        if constexpr (DEBUG_) {
            const auto tmp { min_rx_buffer_free_ };
            min_rx_buffer_free_ = std::min(min_rx_buffer_free_, xStreamBufferSpacesAvailable(uart_rx_stream_));
            if (tmp != min_rx_buffer_free_) {
                printf("UartTcp::run(): RX stream free: %u\n", min_rx_buffer_free_);
            }
        }

        const auto uart_available { xStreamBufferBytesAvailable(uart_rx_stream_) };
        if (uart_available) {
            if (p_tcp_server_->connected()) {
                p_tcp_server_->flush();
            }
            auto tx_len { xStreamBufferReceive(uart_rx_stream_, p_tcp_tx_buffer_->data(), std::min(uart_available, p_tcp_tx_buffer_->size()), 0) };
            auto p_tx_buffer = p_tcp_tx_buffer_->data();

            if (tx_len >= 2 && std::strncmp(p_tx_buffer, "\eC", 2) == 0) {
                p_tx_buffer += 2;
                tx_len -= 2;

                time_t time;
                if (p_ntp_->request(&time)) {
                    auto [p_end, ec] = std::to_chars(time_buffer_.data(), time_buffer_.data() + time_buffer_.size(), time);
                    if (ec == std::errc()) {
                        uart_write_blocking(uart1, reinterpret_cast<const uint8_t*>("set time "), 9);
                        uart_write_blocking(uart1, reinterpret_cast<const uint8_t*>(time_buffer_.data()), p_end - time_buffer_.data());
                        uart_write_blocking(uart1, reinterpret_cast<const uint8_t*>("\r\n"), 2);
                    }

                    if constexpr (DEBUG_) {
                        tm* utc { std::gmtime(&time) };
                        printf("UartTcp::run(): Got NTP response: %02d/%02d/%04d %02d:%02d:%02d UTC\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1'900,
                            utc->tm_hour, utc->tm_min, utc->tm_sec);
                    }
                } else if constexpr (DEBUG_) {
                    printf("UartTcp::run(): NTP request failed.\n");
                }
            }

            if (tx_len && p_tcp_server_->connected()) {
                p_tcp_server_->write(p_tx_buffer, tx_len, false);
            }
        }

        if (!p_tcp_server_->available() && !xStreamBufferBytesAvailable(uart_rx_stream_)) {
            const auto now { time_us_32() };
            if (now > last_debug_time + 500'000) {
                last_debug_time = now;

                last_led_state = !last_led_state;
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, last_led_state);
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    p_worker_ = nullptr;
    vTaskDelete(nullptr);
}

void UartTcp::uart_isr() {
    BaseType_t reschedule { pdFALSE };
    size_t i {};
    while (uart_is_readable(p_uart_) && i < uart_in_buffer_.size()) {
        uart_in_buffer_[i++] = uart_getc(p_uart_);
    }
    if (i) {
        xStreamBufferSendFromISR(uart_rx_stream_, uart_in_buffer_.data(), i, &reschedule);
        portYIELD_FROM_ISR(reschedule);
    }
}
