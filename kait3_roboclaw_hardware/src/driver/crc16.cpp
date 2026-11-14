#include "kait3_roboclaw_hardware/driver/crc16.hpp"

namespace kait3_roboclaw_hardware::driver {
    void crc16_update(std::uint16_t &crc, std::uint8_t value) noexcept {
        std::uint8_t index = static_cast<std::uint8_t>((crc >> 8) ^ value);
        crc = (crc << 8) ^ CRC16_LOOKUP_TABLE[index];
    }

    void crc16_update_many(std::uint16_t &crc, const std::uint8_t *buffer, std::size_t length) noexcept {
        for (std::size_t i = 0; i < length; ++i) {
            crc16_update(crc, buffer[i]);
        }
    }

    std::uint16_t crc16_calculate(const std::uint8_t *buffer, std::size_t length) noexcept {
        std::uint16_t crc = 0;
        for (std::size_t i = 0; i < length; ++i) {
            crc16_update(crc, buffer[i]);
        }
        return crc;
    }
} //kait3_roboclaw_hardware::driver
