#ifndef KAIT3_ROBOCLAW_HARDWARE_DRIVER_EXCEPTIONS_HPP_
#define KAIT3_ROBOCLAW_HARDWARE_DRIVER_EXCEPTIONS_HPP_

#include <chrono>
#include <cstdint>
#include <stdexcept>

namespace kait3_roboclaw_hardware::driver {

    class CRCMismatchException : public std::runtime_error {
    public:
        explicit CRCMismatchException(std::uint16_t expected, std::uint16_t actual);

    private:
        static std::string make_error_message(std::uint16_t expected, std::uint16_t actual);
    };

    class SerialException : public std::runtime_error {
    public:
        SerialException(const char *message);
    };

    class SerialTimeoutException : public std::runtime_error {
    public:
        using duration = std::chrono::steady_clock::duration;

        enum class Operation {
            read,
            write,
        };

        explicit SerialTimeoutException(Operation op, const duration& timeout);

    private:
        static std::string make_error_message(Operation op, const duration& timeout);
    };

    class InvalidResponseException : public std::runtime_error {
    public:
        InvalidResponseException(const char *message);
    };

} //namespace kait3_roboclaw_hardware::driver

#endif //KAIT3_ROBOCLAW_HARDWARE_DRIVER_EXCEPTIONS_HPP_
