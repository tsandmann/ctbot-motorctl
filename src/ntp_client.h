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
 * @file    ntp_client.h
 * @brief   Simple NTP client for LwIP and FreeRTOS
 * @author  Timo Sandmann
 * @date    30.07.2022
 */


#pragma once

#include "pico/stdlib.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>
#include <ctime>


class NtpClient {
    static constexpr bool DEBUG_ { false };

    static constexpr size_t NTP_MSG_LEN_ { 48 }; // byte
    static constexpr uint16_t NTP_PORT_ { 123 };
    static constexpr uint32_t NTP_DELTA_ { 2'208'988'800UL }; // seconds between 1 Jan 1900 and 1 Jan 1970
    static constexpr uint32_t NTP_TIMEOUT_ { 100 }; // ms

    static void recv_cb(void* arg, struct udp_pcb* pcb, struct pbuf* p, const ip_addr_t* addr, u16_t port);

    ip_addr_t ntp_server_;
    ip_addr_t* p_ntp_server_;
    udp_pcb* ntp_pcb_;
    TaskHandle_t caller_;
    time_t* last_time_;

public:
    NtpClient(ip_addr_t* p_ntp_server = nullptr);
    ~NtpClient();

    bool init();

    bool request(time_t* time);

    void test();
};
