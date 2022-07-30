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
 * @file    freertos_runtime_stats.cpp
 * @brief   Generate and print runtime stats on FreeRTOS
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "freertos_runtime_stats.h"

#include <algorithm>
#include <cstring>
#include <cstdio>


FreeRTOSRuntimeStats::FreeRTOSRuntimeStats() : last_total_runtime_ {} {}

bool FreeRTOSRuntimeStats::generate() {
    size_t num_tasks { uxTaskGetNumberOfTasks() };
    if (task_data_.size() != num_tasks) {
        task_data_.resize(num_tasks);
    }
    current_runtimes_.clear();

    uint32_t total_runtime;
    num_tasks = uxTaskGetSystemState(task_data_.data(), num_tasks, &total_runtime);
    if (!num_tasks) {
        return false;
    }

    for (size_t i {}; i < num_tasks; ++i) {
        current_runtimes_.push_back(std::make_pair(task_data_[i].xHandle,
            (task_data_[i].ulRunTimeCounter - last_runtimes_[task_data_[i].xHandle]) * TOTAL_PERCENT_ / (total_runtime - last_total_runtime_)));
        last_runtimes_[task_data_[i].xHandle] = task_data_[i].ulRunTimeCounter;
    }
    last_total_runtime_ = total_runtime;

    auto cmp = [](std::pair<TaskHandle_t, float> const& a, std::pair<void*, float> const& b) { return a.second >= b.second; };
    std::sort(current_runtimes_.begin(), current_runtimes_.end(), cmp);

    return true;
}

bool FreeRTOSRuntimeStats::print() {
    if (!generate()) {
        return false;
    }

    for (auto& e : current_runtimes_) {
        const auto name { pcTaskGetName(e.first) };
        const char* tabs { std::strlen(name) > 7 ? "\t" : "\t\t" };
        const char* space { e.second >= 10.f ? "" : " " };
        const uint8_t affinity { static_cast<uint8_t>(vTaskCoreAffinityGet(e.first) & 0x3) };
        const uint32_t stack_free { uxTaskGetStackHighWaterMark(e.first) * sizeof(StackType_t) };

        printf("%s%s%s%.2f %% (0x%x)\t%u byte\n", name, tabs, space, e.second, affinity, stack_free);
    }

    if (current_runtimes_.size()) {
        puts("");
    }

    return true;
}
