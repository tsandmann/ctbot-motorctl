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
 * @file    tcp_server.cpp
 * @brief   Simple TCP server for LwIP and FreeRTOS
 * @author  Timo Sandmann
 * @date    30.07.2022
 */


#include "tcp_server.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include <cstdio>
#include <cstring>


TcpServer::TcpClient::TcpClient(tcp_pcb* pcb, size_t rx_buf_size)
    : p_pcb_ { pcb }, rx_buf_ {}, rx_buf_offset_ {}, to_send_ {}, connect_time_ { time_us_32() / 1'000UL } {}

TcpServer::TcpClient::~TcpClient() {}

void TcpServer::TcpClient::rx_buf_consume(size_t size) {
    ptrdiff_t left { rx_buf_->len - rx_buf_offset_ - size };
    if (left > 0) {
        rx_buf_offset_ += size;
    } else if (!rx_buf_->next) {
        pbuf_free(rx_buf_);
        rx_buf_ = nullptr;
        rx_buf_offset_ = 0;
    } else {
        auto head { rx_buf_ };
        rx_buf_ = rx_buf_->next;
        rx_buf_offset_ = 0;
        pbuf_ref(rx_buf_);
        pbuf_free(head);
    }

    if (p_pcb_) {
        tcp_recved(p_pcb_, size);
    }
}


TcpServer::TcpServer() : p_server_pcb_ {} {}

TcpServer::~TcpServer() {}

bool TcpServer::init(const char* ssid, const char* password, uint8_t core_id, uint32_t connect_timeout_ms) {
    if (!wifi_connected_) {
        auto p_data { new WifiData { ssid, password } };
        xTaskCreateAffinitySet(TcpServer::init_task, "wifi_init", STACK_SIZE_ / sizeof(StackType_t), p_data, 2, 1 << core_id, &init_task_);

        const auto timeout_us { connect_timeout_ms * 1'000UL };
        const auto start { time_us_32() };
        while (!wifi_connected_ && time_us_32() < start + timeout_us) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!wifi_connected_) {
            vTaskDelete(init_task_);
            init_task_ = nullptr;
            return false;
        }
    }

    return true;
}

void TcpServer::init_task(void* arg) {
    auto p_data { reinterpret_cast<WifiData*>(arg) };
    const auto data { *p_data };
    delete p_data;
    p_data = nullptr;

    if constexpr (DEBUG_) {
        printf("TcpServer::init_task() started on core %u.\n", portGET_CORE_ID());
    }
    if (!cyw43_arch_init_) {
        if (cyw43_arch_init_with_country(CYW43_COUNTRY_GERMANY)) {
            printf("Failed to init cyw43.\n");
            vTaskDelete(nullptr);
            return; // never reached
        }

        cyw43_arch_init_ = true;

        if constexpr (DEBUG_) {
            printf("TcpServer::init_task() cyw43_arch_init_with_country() done.\n");
        }
    }

    while (!wifi_connected_) {
        printf("Connecting to WiFi...\n");

        cyw43_arch_enable_sta_mode();
        if (!cyw43_arch_wifi_connect_timeout_ms(data.ssid_.c_str(), data.password_.c_str(), CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT_MS_ / 10UL)) {
            printf("Wifi connected.\n");
            wifi_connected_ = true;
        } else {
            printf("Wifi failed to connect.\n");
            // cyw43_arch_deinit();
            vTaskDelay(pdMS_TO_TICKS(1'000));
        }
    }

    ::vTaskDelete(nullptr);
}

err_t TcpServer::cb_accept(void* arg, struct tcp_pcb* client_pcb, err_t err) {
    if constexpr (DEBUG_) {
        printf("TcpServer::cb_accept()\n");
    }

    configASSERT(arg);
    auto p_this { reinterpret_cast<TcpServer*>(arg) };

    cyw43_arch_lwip_check();
    if (p_this->clients_.size() == TcpServer::MAX_CLIENTS_) {
        printf("TcpServer::cb_accept(): Maximum number of clients reached.\n");
        tcp_close(client_pcb);
        return ERR_ABRT;
    }

    if (err != ERR_OK || client_pcb == nullptr) {
        printf("TcpServer::cb_accept(): failure in accept\n");
        p_this->close();
        return ERR_VAL;
    }
    printf("TcpServer::cb_accept(): client connected\n");

    auto new_client { new TcpClient { client_pcb, RX_BUF_SIZE_ } };
    configASSERT(new_client);

    tcp_arg(client_pcb, p_this);
    tcp_sent(client_pcb, TcpServer::cb_sent);
    tcp_recv(client_pcb, TcpServer::cb_recv);
    tcp_poll(client_pcb, TcpServer::cb_poll, TcpServer::POLL_TIME_S_ * 2);
    tcp_err(client_pcb, TcpServer::cb_err);
    if (NO_DELAY_) {
        tcp_nagle_disable(client_pcb);
    }
    p_this->clients_[client_pcb] = new_client;

    /* telnet: suppress local echo */
    new_client->to_send_ = 39;
    tcp_write(client_pcb, "\xff\xfb\x01", 3, TCP_WRITE_FLAG_COPY);
    tcp_write(client_pcb, "\xff\xfe\x01", 3, TCP_WRITE_FLAG_COPY);
    tcp_write(client_pcb, "\xff\xfe\x22", 3, TCP_WRITE_FLAG_COPY);

    tcp_write(client_pcb, "telnet-2-serial ready.\r\n", 24, TCP_WRITE_FLAG_COPY);

    return ERR_OK;
}

err_t TcpServer::cb_sent(void* arg, tcp_pcb* client_pcb, u16_t len) {
    configASSERT(arg);
    auto p_this { reinterpret_cast<TcpServer*>(arg) };

    cyw43_arch_lwip_check();
    auto& to_send { p_this->clients_.at(client_pcb)->to_send_ };

    if constexpr (DEBUG_) {
        printf("TcpServer::cb_sent(): len=%u to_send_=%u\n", len, to_send);
    }
    to_send -= len;

    if (DEBUG_ && to_send == 0) {
        printf("TcpServer::cb_sent(): all data sent.\n");
    }

    return ERR_OK;
}

err_t TcpServer::cb_recv(void* arg, tcp_pcb* client_pcb, struct pbuf* p, err_t err) {
    if constexpr (DEBUG_) {
        printf("TcpServer::cb_recv()\n");
    }

    configASSERT(arg);
    auto p_this { reinterpret_cast<TcpServer*>(arg) };

    cyw43_arch_lwip_check();
    if (!p) {
        err_t res { ERR_CLSD };
        if (client_pcb) {
            tcp_arg(client_pcb, nullptr);
            tcp_poll(client_pcb, nullptr, 0);
            tcp_sent(client_pcb, nullptr);
            tcp_recv(client_pcb, nullptr);
            tcp_err(client_pcb, nullptr);

            res = tcp_close(client_pcb);
            if (res != ERR_OK) {
                printf("TcpServer::close(): failed %d, calling abort\n", res);
                tcp_abort(client_pcb);
                res = ERR_ABRT;
            }
            p_this->clients_.erase(client_pcb);
        }
        return res;
    }

    auto p_client { p_this->clients_.at(client_pcb) };
    if (p_client->rx_buf_) {
        if constexpr (DEBUG_) {
            printf("TcpServer::cb_recv(): %u/%u err %d\n", p_client->rx_buf_->tot_len, p->tot_len, err);
        }
        pbuf_cat(p_client->rx_buf_, p);
    } else {
        if constexpr (DEBUG_) {
            printf("TcpServer::cb_recv(): %u err %d\n", p->tot_len, err);
        }
        p_client->rx_buf_ = p;
        p_client->rx_buf_offset_ = 0;
    }

    return ERR_OK;
}

err_t TcpServer::cb_poll(void*, tcp_pcb*) {
    if constexpr (DEBUG_) {
        printf("TcpServer::cb_poll()\n");
    }

    return ERR_OK;
}

void TcpServer::cb_err(void* arg, err_t err) {
    printf("TcpServer::cb_err()");
    if (err != ERR_ABRT) {
        printf(": err=%d\n", err);
    } else {
        printf("\n");
    }
}

bool TcpServer::open(uint16_t port) {
    if (!wifi_connected_) {
        return false;
    }

    printf("TcpServer::open(): Starting server at %s port %u on core %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), port, portGET_CORE_ID());

    cyw43_arch_lwip_begin();
    tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("TcpServer::open(): failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, nullptr, port);
    if (err) {
        printf("TcpServer::open(): failed to bind to port %u\n");
        return false;
    }

    p_server_pcb_ = tcp_listen_with_backlog(pcb, 1);
    if (!p_server_pcb_) {
        printf("TcpServer::open(): failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(p_server_pcb_, this);
    tcp_accept(p_server_pcb_, TcpServer::cb_accept);
    if (NO_DELAY_) {
        tcp_nagle_disable(p_server_pcb_);
    }
    cyw43_arch_lwip_end();

    return true;
}

bool TcpServer::close() {
    if constexpr (DEBUG_) {
        printf("TcpServer::close()\n");
    }

    if (!wifi_connected_) {
        return false;
    }

    err_t err { ERR_OK };

    cyw43_arch_lwip_begin();
    for (auto client : clients_) {
        auto client_pcb { client.first };
        tcp_arg(client_pcb, nullptr);
        tcp_poll(client_pcb, nullptr, 0);
        tcp_sent(client_pcb, nullptr);
        tcp_recv(client_pcb, nullptr);
        tcp_err(client_pcb, nullptr);

        err = tcp_close(client_pcb);
        if (err != ERR_OK) {
            printf("TcpServer::close(): failed %d, calling abort\n", err);
            tcp_abort(client_pcb);
            err = ERR_ABRT;
        }
    }

    clients_.clear();
    cyw43_arch_lwip_end();

    if (p_server_pcb_) {
        cyw43_arch_lwip_begin();
        tcp_arg(p_server_pcb_, nullptr);
        tcp_close(p_server_pcb_);
        cyw43_arch_lwip_end();
        p_server_pcb_ = nullptr;
    }

    return err;
}

uint16_t TcpServer::read(tcp_pcb* client_pcb, void* p_data, uint16_t len) {
    if constexpr (DEBUG_) {
        printf("TcpServer::read(%u)\n", len);
    }

    cyw43_arch_lwip_begin();
    if (clients_.empty()) {
        cyw43_arch_lwip_end();
        if constexpr (DEBUG_) {
            printf("TcpServer::read(): no client connected.\n");
        }
        return 0;
    }

    auto p_client { clients_.at(client_pcb) };
    if (!p_client->rx_buf_) {
        cyw43_arch_lwip_end();
        return 0;
    }

    size_t max_size { p_client->rx_buf_->tot_len - p_client->rx_buf_offset_ };
    if (len > max_size) {
        len = max_size;
    }

    if constexpr (DEBUG_) {
        printf("TcpServer::read(%u): %u, %u\n", len, p_client->rx_buf_->tot_len, p_client->rx_buf_offset_);
    }

    size_t size_read {};
    auto p_dest { reinterpret_cast<char*>(p_data) };
    while (len) {
        size_t buf_size = p_client->rx_buf_->len - p_client->rx_buf_offset_;
        size_t copy_size = (len < buf_size) ? len : buf_size;
        std::memcpy(p_dest, reinterpret_cast<char*>(p_client->rx_buf_->payload) + p_client->rx_buf_offset_, copy_size);
        p_dest += copy_size;
        p_client->rx_buf_consume(copy_size);
        len -= copy_size;
        size_read += copy_size;
    }

    cyw43_arch_lwip_end();

    if constexpr (DEBUG_) {
        printf("TcpServer::read(): done: %u\n", size_read);
    }
    return size_read;
}

uint16_t TcpServer::write(tcp_pcb* client_pcb, const void* p_data, uint16_t len, bool copy) {
    if constexpr (DEBUG_) {
        printf("TcpServer::write(%u)\n", len);
    }

    cyw43_arch_lwip_begin();
    if (clients_.empty()) {
        cyw43_arch_lwip_end();
        if (DEBUG_) {
            printf("TcpServer::write(): no client connected.\n");
        }
        return 0;
    }

    err_t err;
    const uint32_t start { time_us_32() };
    do {
        err = tcp_write(client_pcb, p_data, len, copy ? TCP_WRITE_FLAG_COPY : 0);

        if (err == ERR_MEM) {
            cyw43_arch_lwip_end();
            vTaskDelay(pdMS_TO_TICKS(2));
            cyw43_arch_lwip_begin();
        }
    } while (err == ERR_MEM && time_us_32() < start + TCP_WRITE_TIMEOUT_US_);

    if (err != ERR_OK) {
        cyw43_arch_lwip_end();
        printf("TcpServer::write(): Failed to write data %d\n", err);
        return 0;
    }

    clients_.at(client_pcb)->to_send_ += len;
    cyw43_arch_lwip_end();
    return len;
}

uint16_t TcpServer::write(const void* p_data, uint16_t len, bool copy) {
    if (!wifi_connected_) {
        return false;
    }

    for (auto client : clients_) {
        if (write(client.first, p_data, len, copy) != len) {
            return 0;
        }
    }

    return len;
}

bool TcpServer::flush() const {
    if constexpr (DEBUG_) {
        printf("TcpServer::flush()\n");
    }

    if (!wifi_connected_) {
        return false;
    }

    cyw43_arch_lwip_begin();
    if (clients_.empty()) {
        cyw43_arch_lwip_end();
        if constexpr (DEBUG_) {
            printf("TcpServer::flush(): no client connected.\n");
        }
        return 0;
    }

    err_t err { ERR_OK };
    for (auto client : clients_) {
        const auto res { tcp_output(client.first) };
        if (res != ERR_OK) {
            err = res;
        }
    }

    cyw43_arch_lwip_end();
    return err == ERR_OK;
}

uint16_t TcpServer::available(tcp_pcb* client) const {
    cyw43_arch_lwip_begin();
    if (!clients_.contains(client)) {
        cyw43_arch_lwip_end();
        return 0;
    }

    auto p_client { clients_.at(client) };
    if (!p_client->rx_buf_) {
        cyw43_arch_lwip_end();
        return 0;
    }

    const auto res { p_client->rx_buf_->tot_len - p_client->rx_buf_offset_ };
    cyw43_arch_lwip_end();
    return res;
}

tcp_pcb* TcpServer::available() const {
    if (!wifi_connected_) {
        return nullptr;
    }

    cyw43_arch_lwip_begin(); // TODO: distinct mutex for clients_?
    for (auto client : clients_) {
        if (client.second->rx_buf_) {
            const auto res { client.first };
            cyw43_arch_lwip_end();
            return res;
        }
    }

    cyw43_arch_lwip_end();
    return nullptr;
}

uint32_t TcpServer::get_connect_time(tcp_pcb* client_pcb) const {
    cyw43_arch_lwip_begin();
    auto client { clients_.at(client_pcb) };
    const auto res { client->connect_time_ };
    cyw43_arch_lwip_end();
    return res;
}
