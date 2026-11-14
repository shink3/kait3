#ifndef KAIT3_ROBOCLAW_HARDWARE_UTIL_BIT_CONV_HPP_
#define KAIT3_ROBOCLAW_HARDWARE_UTIL_BIT_CONV_HPP_

#include <climits>
#include <cstdint>
#include <type_traits>
#include <cstddef>

namespace kait3_roboclaw_hardware::util {

    template<typename T>
    std::size_t write_integer_big_endian(std::uint8_t *buffer, const T& value) noexcept {
        static_assert(std::is_integral_v<T>, "T must be integer type");
        static_assert(sizeof(T) <= 8, "T must be 64 bits or less");
        static_assert(CHAR_BIT == 8, "8 bit byte required");

        std::uint8_t *p = buffer;

        using unsigind_type = std::make_unsigned_t<T>;
        unsigind_type u = static_cast<unsigind_type>(value);
        
        for (std::size_t i = sizeof(T); i > 0; --i) {
            *p++ = static_cast<std::uint8_t>((u >> (i - 1) * 8) & 0xFF);
        }
        return sizeof(T);
    }

    template<typename T>
    std::size_t read_integer_big_endian(const std::uint8_t *buffer, T& value) noexcept {
        static_assert(std::is_integral_v<T>, "T must be integer type");
        static_assert(sizeof(T) <= 8, "T must be 64 bits or less");
        static_assert(CHAR_BIT == 8, "8 bit bytes required");

        const std::uint8_t *p = buffer;

        using unsigned_type = std::make_unsigned_t<T>;
        unsigned_type u = 0;
        
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            u = static_cast<unsigned_type>((u << 8) | *p++);
        }
        value = u;
        return sizeof(T);
    }
} //namespace kait3_roboclaw_hardware::util
#endif //KAIT3_ROBOCLAW_HARDWARE_UTIL_BIT_CONV_HPP_

