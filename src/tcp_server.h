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
 * @file    tcp_server.h
 * @brief   Simple TCP server for LwIP and FreeRTOS
 * @author  Timo Sandmann
 * @date    30.07.2022
 */


#pragma once

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <atomic>
#include <map>


class TcpServer {
    static constexpr bool DEBUG_ { false };
    static constexpr uint8_t MAX_CLIENTS_ { 2 };
    static constexpr bool NO_DELAY_ { false };
    static constexpr uint32_t TCP_WRITE_TIMEOUT_US_ { 200'000 };

    static inline TaskHandle_t init_task_ {};
    static inline bool cyw43_arch_init_ {};
    static inline std::atomic<bool> wifi_connected_ {};


    static err_t cb_accept(void* arg, struct tcp_pcb* client_pcb, err_t err);
    static err_t cb_sent(void* arg, struct tcp_pcb* tpcb, u16_t len);
    static err_t cb_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);
    static err_t cb_poll(void* arg, struct tcp_pcb* tpcb);
    static void cb_err(void* arg, err_t err);
    static void init_task(void*);

protected:
    static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS_ { 20'000 };
    static constexpr size_t STACK_SIZE_ { 2048 }; // byte
    static constexpr size_t RX_BUF_SIZE_ { 2048 };
    static constexpr uint32_t POLL_TIME_S_ { 5 };

    struct WifiData {
        std::string ssid_;
        std::string password_;

        WifiData(const std::string_view ssid, const std::string_view password) : ssid_ { ssid }, password_ { password } {}
    };

    struct TcpClient {
        tcp_pcb* p_pcb_;
        pbuf* rx_buf_;
        size_t rx_buf_offset_;
        uint16_t to_send_;
        uint32_t connect_time_;

        TcpClient(tcp_pcb* pcb, size_t rx_buf_size);
        ~TcpClient();

        void rx_buf_consume(size_t size);
    };

    tcp_pcb* p_server_pcb_;
    std::map<tcp_pcb*, TcpClient*> clients_;

public:
    static bool init(const char* ssid, const char* password) {
        return init(ssid, password, 1);
    }
    static bool init(const char* ssid, const char* password, uint8_t core_id) {
        return init(ssid, password, core_id, WIFI_CONNECT_TIMEOUT_MS_);
    }
    static bool init(const char* ssid, const char* password, uint8_t core_id, uint32_t connect_timeout_ms);

    // TcpServer(const char* ssid, const char* password) : TcpServer { ssid, password, 1, WIFI_CONNECT_TIMEOUT_MS_ } {};
    // TcpServer(const char* ssid, const char* password, uint8_t core_id) : TcpServer { ssid, password, core_id, WIFI_CONNECT_TIMEOUT_MS_ } {};
    TcpServer();
    ~TcpServer();

    bool open(uint16_t port);
    bool close();

    uint16_t available(tcp_pcb* client) const;
    tcp_pcb* available() const;

    bool connected() const {
        return !clients_.empty();
    }

    uint16_t read(tcp_pcb* client_pcb, void* p_data, uint16_t len);
    uint16_t write(tcp_pcb* client_pcb, const void* p_data, uint16_t len, bool copy = true);
    uint16_t write(const void* p_data, uint16_t len, bool copy = true);
    bool flush() const;

    uint32_t get_connect_time(tcp_pcb* client_pcb) const;
};
