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


#include "ntp_client.h"

#include "pico/cyw43_arch.h"

#include <cstring>


NtpClient::NtpClient(ip_addr_t* p_ntp_server) : p_ntp_server_ { p_ntp_server }, ntp_pcb_ {}, caller_ {}, last_time_ {} {}

NtpClient::~NtpClient() {}

bool NtpClient::init() {
    if (!p_ntp_server_) {
        p_ntp_server_ = &ntp_server_;
        *p_ntp_server_ = *netif_ip4_gw(netif_list);
    }

    ntp_pcb_ = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!ntp_pcb_) {
        printf("NtpClient::init(): Failed to create pcb\n");
        return false;
    }

    udp_recv(ntp_pcb_, recv_cb, this);

    return true;
}

void NtpClient::recv_cb(void* arg, struct udp_pcb* pcb, struct pbuf* p, const ip_addr_t* addr, u16_t port) {
    auto p_this { reinterpret_cast<NtpClient*>(arg) };

    const uint8_t mode { static_cast<uint8_t>(pbuf_get_at(p, 0) & 0x7) };
    const uint8_t stratum { pbuf_get_at(p, 1) };

    /* check the result */
    if (ip_addr_cmp(addr, p_this->p_ntp_server_) && port == NTP_PORT_ && p->tot_len == NTP_MSG_LEN_ && mode == 0x4 && stratum != 0) {
        uint8_t seconds_buf[4] = { 0 };
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        const uint32_t seconds_since_1900 { seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3] };
        const uint32_t seconds_since_1970 { seconds_since_1900 - NTP_DELTA_ };

        *p_this->last_time_ = seconds_since_1970;
    } else {
        printf("NtpClient::recv_cb(): Invalid NTP response\n");
        *p_this->last_time_ = 0;
    }
    pbuf_free(p);

    if (p_this->caller_) {
        xTaskNotifyGive(p_this->caller_);
    }
}

bool NtpClient::request(time_t* time) {
    last_time_ = time;
    caller_ = xTaskGetCurrentTaskHandle();

    cyw43_arch_lwip_begin();
    pbuf* p { pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN_, PBUF_RAM) };
    uint8_t* req { static_cast<uint8_t*>(p->payload) };
    std::memset(req, 0, NTP_MSG_LEN_);
    req[0] = 0x1b;
    udp_sendto(ntp_pcb_, p, p_ntp_server_, NTP_PORT_);
    pbuf_free(p);
    cyw43_arch_lwip_end();

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(NTP_TIMEOUT_)) != 1) {
        return false;
    }

    if (*last_time_ == 0) {
        return false;
    }

    return true;
}

void NtpClient::test() {
    if (!init()) {
        return;
    }

    time_t time;

    while (true) {
        if (request(&time)) {
            tm* utc { std::gmtime(&time) };
            printf("NtpClient::test(): Got NTP response: %02d/%02d/%04d %02d:%02d:%02d\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1'900, utc->tm_hour,
                utc->tm_min, utc->tm_sec);
        } else {
            printf("NtpClient::test(): NTP request failed.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(30'000));
    }
}
