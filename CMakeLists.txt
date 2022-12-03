# This file is part of the ct-Bot motor controller firmware.
# Copyright (c) 2021-2022 Timo Sandmann
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library. If not, see <http://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 3.13)

include(custom_options.cmake OPTIONAL)

set(PICO_BOARD pico_w CACHE INTERNAL "")

include(pico_sdk_import.cmake)

project(ctbot_motorctl C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

pico_sdk_init()

add_executable(motorctl
    src/freertos_runtime_stats.cpp
    src/main.cpp
    src/motor.cpp
    src/ntp_client.cpp
    src/rotary_encoder.cpp
    src/speed_control.cpp
    src/support.c
    src/tcp_server.cpp
    src/uart_tcp.cpp
    lib/crc32/src/crc32.cpp
    lib/pid/src/pid_v1.cpp
)

pico_generate_pio_header(motorctl ${CMAKE_CURRENT_LIST_DIR}/src/rotary_encoder.pio)

target_include_directories(motorctl PUBLIC
    ./src
    ./lib/crc32/src
    ./lib/pid/src
)
target_compile_options(motorctl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-flto>)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
pico_enable_stdio_usb(motorctl 1)

pico_add_extra_outputs(motorctl)

add_subdirectory(FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040 FreeRTOS-Kernel)

target_link_libraries(motorctl FreeRTOS-Kernel pico_cyw43_arch_lwip_sys_freertos pico_stdlib hardware_pwm hardware_adc)

if (STTY_UPLOAD_COMMAND)
    install(CODE "execute_process(COMMAND ${STTY_UPLOAD_COMMAND} COMMAND ${CMAKE_COMMAND} -E sleep 2)")
endif (STTY_UPLOAD_COMMAND)

if (UPLOAD_PATH)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/motorctl.uf2" DESTINATION "${UPLOAD_PATH}/")
endif (UPLOAD_PATH)
