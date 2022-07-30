//
// Copyright (c) 2013 Christopher Baker <https://christopherbaker.net>
//
// SPDX-License-Identifier:	MIT
//


#pragma once

#include <cstdint>
#include <cstddef>


/// \brief A class for calculating the CRC32 checksum from arbitrary data.
/// \sa http://forum.arduino.cc/index.php?topic=91179.0
class CRC32 {
public:
    /// \brief Initialize an empty CRC32 checksum.
    CRC32() : _state { ~0UL } {}

    /// \brief Reset the checksum claculation.
    void reset() {
        _state = ~0UL;
    }

    /// \brief Update the current checksum caclulation with the given data.
    /// \param data The data to add to the checksum.
    void update(const uint8_t& data);

    /// \brief Update the current checksum caclulation with the given data.
    /// \tparam Type The data type to read.
    /// \param data The data to ad  d to the checksum.
    template <typename T>
    void update(const T& data) {
        update(&data, 1);
    }

    /// \brief Update the current checksum caclulation with the given data.
    /// \tparam Type The data type to read.
    /// \param p_data The array to add to the checksum.
    /// \param size Size of the array to add.
    template <typename T>
    void update(const T* p_data, size_t size) {
        const size_t n_bytes { size * sizeof(T) };
        const uint8_t* ptr { reinterpret_cast<const uint8_t*>(p_data) };

        for (size_t i {}; i < n_bytes; ++i) {
            update(ptr[i]);
        }
    }

    /// \returns the caclulated checksum.
    uint32_t finalize() const {
        return ~_state;
    }

    /// \brief Calculate the checksum of an arbitrary data array.
    /// \tparam Type The data type to read.
    /// \param data A pointer to the data to add to the checksum.
    /// \param size The size of the data to add to the checksum.
    /// \returns the calculated checksum.
    template <typename T>
    static uint32_t calculate(const T* data, size_t size) {
        CRC32 crc;
        crc.update(data, size);
        return crc.finalize();
    }

private:
    uint32_t _state;
};
