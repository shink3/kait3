#include "kait3_roboclaw_hardware/driver/exceptions.hpp"
#include <chrono>
#include <string>

namespace kait3_roboclaw_hardware::driver {

    CRCMismatchException::CRCMismatchException(std::uint16_t expected, std::uint16_t actual)
    : std::runtime_error(make_error_message(expected, actual)) {}

    std::string CRCMismatchException::make_error_message(std::uint16_t expected, std::uint16_t actual) {
       return "CRC mismatch. expected=" + std::to_string(expected) + ", but got " + std::to_string(actual);
    }

    SerialException::SerialException(const char *message)
    : std::runtime_error(message) {};

    SerialTimeoutException::SerialTimeoutException(
        SerialTimeoutException::Operation op, 
        const SerialTimeoutException::duration& timeout
    ) : std::runtime_error(make_error_message(op, timeout)) {}

    std::string SerialTimeoutException::make_error_message(
        SerialTimeoutException::Operation op,
        const SerialTimeoutException::duration& timeout
    ) {
        return std::string("serial") + (op == Operation::read ? "read" : "write") + 
                "timeout after " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count()) +
                "ms";
    }

    InvalidResponseException::InvalidResponseException(const char *message)
    : std::runtime_error(message) {}


} //namespace kait3_roboclaw_hardware::driver
