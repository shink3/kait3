#include "kait3_roboclaw_hardware/driver/roboclaw.hpp"
#include "kait3_roboclaw_hardware/driver/command.hpp"
#include <chrono>
#include <cmath>
#include <cerrno>
#include <exception>
#include <algorithm>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace kait3_roboclaw_hardware::driver {

    namespace {

        inline constexpr speed_t to_termios_baud_rate(std::uint32_t baud_rate) {
            switch (baud_rate) {
                case 2400:   return B2400;
                case 4800:   return B4800;
                case 9600:   return B9600;
                case 19200:  return B19200;
                case 38400:  return B38400;
                case 57600:  return B57600;
                case 115200: return B115200;
                case 230400: return B230400;
                case 460800: return B460800; 
                default:
                    __builtin_unreachable();
            }
        }

        int wait_fd(int fd, short events, std::chrono::steady_clock::time_point deadline) {
            using clock = std::chrono::steady_clock;

            for (;;) {
                auto now = clock::now();

                if (now >= deadline) return 0;

                auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);

                int timeout_ms = static_cast<int>(std::min<std::int64_t>(remaining.count() + 1, std::numeric_limits<int>::max()));

                struct pollfd pfd;
                pfd.fd = fd;
                pfd.events = events;
                
                int result = ::poll(&pfd, 1, timeout_ms);

                if (result < 0) {
                    if (errno == EINTR) 
                        continue;
                    throw std::system_error(errno, std::system_category(), "poll");
                } else if (result == 0) {
                    return 0;
                } else {
                    if (pfd.revents & POLLNVAL) 
                        throw std::system_error(EBADF, std::system_category(), "poll");
                    
                    if (pfd.revents & (POLLHUP | POLLERR)) 
                        throw SerialException("serial port communication failed");

                    if (!(pfd.revents & events)) continue;

                    return 1;
                }
            }
        }  

        inline int wait_write_fd(int fd, std::chrono::steady_clock::time_point deadline) {
            return wait_fd(fd, POLLOUT, deadline);
        }

        inline int wait_read_fd(int fd, std::chrono::steady_clock::time_point deadline) {
            return wait_fd(fd, POLLIN, deadline);
        }

       inline std::uint32_t to_fixed_16_16(const std::string& arg_name, double value) {
            constexpr double scale = 65536.0;
            constexpr double uint32_limit = static_cast<double>(std::numeric_limits<std::uint32_t>::max()) + 1.0;
             
            double scaled = value * scale;

            if (!std::isfinite(value) ||
                value < 0.0 ||
                !std::isfinite(scaled) ||
                scaled >= uint32_limit) {
                throw std::out_of_range(arg_name + " is out of range of 16.16 fixed-point, but got " + std::to_string(scaled));
            }

            return static_cast<std::uint32_t>(scaled);
        }

        inline double from_fixed_16_16(std::uint32_t value) {
            constexpr double scale = 65536.0;
            return static_cast<double>(value) / scale;
        }

        inline void validate_volts(double volts, const std::string param_name = "volts") {
            if (volts < 6.0 || 34.0 < volts) {
                throw std::out_of_range( param_name  + " must be in range of 6.0V to 34.0V, but got" + std::to_string(volts) + "V");
            }
        }
    }

    RoboClaw::RoboClaw(
        const std::string& path,
        std::uint32_t baud_rate,
        std::uint8_t address,
        const duration& read_timeout,
        const duration& write_timeout,
        bool exclusive_access,
        bool unacknowledged,
        std::uint32_t max_attempts
    ) : fd_(-1),
        device_path_(path),
        tx_buffer_(64),
        rx_buffer_(64),
        exclusive_access_(exclusive_access),
        unacknowledged_(unacknowledged),
        max_attempts_(max_attempts)
    {
        set_baud_rate(baud_rate);
        set_address(address);
        set_read_timeout(read_timeout);
        set_write_timeout(write_timeout);
    }

   RoboClaw::~RoboClaw() {
        close();
    }

    constexpr bool RoboClaw::is_valid_baud_rate(std::uint32_t baud_rate) noexcept {
        switch (baud_rate) {
            case 2400:
            case 9600:
            case 19200:
            case 38400:
            case 57600:
            case 115200:
            case 230400:
            case 460800:
                return true;
            default:
                return false;
        }
    }

    constexpr bool RoboClaw::is_valid_address(std::uint8_t address) noexcept {
        constexpr std::uint8_t min_address = 0x80;
        constexpr std::uint8_t max_address = 0x87;

        return min_address <= address && address <= max_address;
    }

    constexpr std::uint16_t RoboClaw::command_group(RoboClawCommand command) noexcept {
        return static_cast<std::uint16_t>(command) & ROBOCLAW_CMD_GROUP_MASK;
    }

    constexpr std::uint8_t RoboClaw::command_value(RoboClawCommand command) noexcept {
        return static_cast<std::uint8_t>(
            static_cast<std::uint16_t>(command) & ROBOCLAW_CMD_VALUE_MASK
        );
    }

    void RoboClaw::write_all(const std::uint8_t *source, std::size_t count) {
        using clock = std::chrono::steady_clock;
        using operation = SerialTimeoutException::Operation;

        assert(is_open());
        assert(source != nullptr || count == 0);

        auto deadline = clock::now() + write_timeout_;

        std::size_t total_written = 0;
        while (total_written < count) {
            int ret = wait_write_fd(fd_, deadline);

            if (ret == 0) { 
                throw SerialTimeoutException(operation::write, write_timeout_);
            } else {
                ssize_t num_written = ::write(fd_, source + total_written, count - total_written);

                if (num_written < 0) {
                    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                        continue;
                    throw std::system_error(errno, std::system_category(), "write");
                } else if (num_written == 0) {
                    throw SerialException("write retuned zero");
                } else {
                    total_written += static_cast<std::size_t>(num_written);
                }
            }
        }
    }

    void RoboClaw::read_exactly(std::uint8_t *destination, std::size_t offset, std::size_t count) {
        using clock = std::chrono::steady_clock;
        using operation = SerialTimeoutException::Operation;

        assert(is_open());
        assert(destination != nullptr || count == 0);

        auto deadline = clock::now() + read_timeout_;

        std::size_t total_read = 0;
        while (total_read < count) {
            int ret = wait_read_fd(fd_, deadline);
            
            if (ret == 0) {
                throw SerialTimeoutException(operation::read, read_timeout_);
            } else {
                ssize_t num_read = ::read(fd_, destination + offset + total_read, count - total_read);

                if (num_read < 0) {
                    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                        continue;
                    throw std::system_error(errno, std::system_category(), "read");
                } else if (num_read == 0) {
                    continue;
                } else {
                    total_read += static_cast<std::size_t>(num_read);
                }
            }
        }
    }

    void RoboClaw::clear_input_buffer() {
         if (::tcflush(fd_, TCIFLUSH) == -1) 
             throw std::system_error(errno, std::system_category(), "tcflush(TCIFLUSH)");
    }

    void RoboClaw::send_query_request(RoboClawCommand command) {
        assert(command_group(command) == ROBOCLAW_CMD_GROUP_QUERY_FLAG);
        
        PacketBufferWriter writer(tx_buffer_);
        writer.write_header(address_, command_value(command));
        writer.flush();
        
        write_all(tx_buffer_.data(), writer.num_written());
    }   

    void RoboClaw::recv_control_response() {
        if (unacknowledged_) return;

        std::uint8_t ack;
        read_exactly(&ack, 0, 1);
        
        if (ack != 0xFF)
            throw InvalidResponseException((std::string("ack is invalid, got ") + std::to_string(ack)).c_str());
    }

    void RoboClaw::open() {
        if (is_open()) 
            throw std::logic_error("serial port is already open");

        int fd = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); 

        if (fd == -1) 
            throw std::system_error(errno, std::system_category(), "open");

        try {
            if (exclusive_access_) {
                if (::ioctl(fd, TIOCEXCL) == -1)  
                    throw std::system_error(errno, std::system_category(), "ioctl(TIOCEXCL)");
            }

            struct termios tty{};
            if (::tcgetattr(fd, &tty) == -1) 
                throw std::system_error(errno, std::system_category(), "tcgetattr");

            ::cfmakeraw(&tty);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;

            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 0;

            speed_t speed = to_termios_baud_rate(baud_rate_);

            if (::cfsetspeed(&tty, speed) == -1) 
                throw std::system_error(errno, std::system_category(), "cfsetspeed");

            if (::tcsetattr(fd, TCSANOW, &tty) == -1)
                throw std::system_error(errno, std::system_category(), "tcsetattr");
        } catch(...) {
            ::close(fd);
            throw;
        }
        fd_ = fd;
    }

    void RoboClaw::close() {
        if (fd_ == -1) return;

        ::close(fd_);
        fd_ = -1;
    }

    bool RoboClaw::is_open() const noexcept {
        return fd_ >= 0;
    }
   
    const std::string& RoboClaw::device_path() const noexcept {
        return device_path_;
    }

    std::uint32_t RoboClaw::baud_rate() const noexcept {
        return baud_rate_;
    }

    void RoboClaw::set_baud_rate(std::uint32_t baud_rate) {
        if (!is_valid_baud_rate(baud_rate)) 
            throw std::invalid_argument("unsupported baud rate");
        if (is_open()) 
            throw std::logic_error("cannot change baud rate while serial is open");

        baud_rate_ = baud_rate;
    }

    std::uint8_t RoboClaw::address() const noexcept {
        return address_;
    }

    void RoboClaw::set_address(std::uint8_t address) {
        if (!is_valid_address(address)) 
            throw std::invalid_argument("unsupported address");

        if (is_open())
            throw std::logic_error("cannot change address while serial is open");

        address_ = address;
    }

    const RoboClaw::duration& RoboClaw::read_timeout() const noexcept {
        return read_timeout_;
    }

    void RoboClaw::set_read_timeout(const RoboClaw::duration& timeout) {
        if (timeout <= RoboClaw::duration::zero()) 
            throw std::invalid_argument("read timeout must be positive");

        if (is_open()) 
            throw std::logic_error("cannot change read_timeout while serial is open");
    
        read_timeout_ = timeout;
    }

    const RoboClaw::duration& RoboClaw::write_timeout() const noexcept {
        return write_timeout_;
    }
    
    void RoboClaw::set_write_timeout(const RoboClaw::duration& timeout) {
        if (timeout <= RoboClaw::duration::zero()) 
            throw std::invalid_argument("write timeout must be positive");

        if (is_open()) 
            throw std::logic_error("cannot change write_timeout while serial is open");

        write_timeout_ = timeout;
    }

    bool RoboClaw::exclusive_access() const noexcept {
        return exclusive_access_;
    }

    void RoboClaw::set_exclusive_access(bool exclusive_access) {
        if (is_open())
            throw std::logic_error("cannot change exclusive_access while serial is open");

        exclusive_access_ = exclusive_access;
    }

    bool RoboClaw::unacknowledged() const noexcept {
        return unacknowledged_; 
    }

    void RoboClaw::set_unacknoledged(bool unacknowledged) {
        if (is_open()) 
            throw std::logic_error("cannot change unacknoledged while serial is open");

        unacknowledged_ = unacknowledged;
    }

    std::uint32_t RoboClaw::max_attempts() const noexcept {
        return max_attempts_;
    }

    void RoboClaw::set_max_attempts(std::uint32_t attempts) {
        if (is_open())
            throw std::logic_error("cannot change max_attmpts while serial is open");

        max_attempts_ = attempts;
    }

    void RoboClaw::execute_compatibility_motor_command(
        RoboClawCommand command, 
        std::uint8_t speed 
    ) {
        if (speed > 127) {
            throw std::out_of_range(
                "speed must be in range of  0 to 127 , but got " + std::to_string(speed) 
            );
        }
        execute_control_command<std::uint8_t>(command, speed);
    }

    void RoboClaw::forward_m1(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_forward_m1, speed);
    }

    void RoboClaw::backward_m1(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_backward_m1, speed);
    }

    void RoboClaw::set_main_voltage_minimum(double volts) {
        validate_volts(volts);
        std::uint8_t value = static_cast<std::uint8_t>((volts - 6) * 5);
        execute_control_command<std::uint8_t>(RoboClawCommand::set_main_voltage_min, value); 
    }

    void RoboClaw::set_main_voltage_maximum(double volts) {
        validate_volts(volts);
        std::uint8_t value = static_cast<std::uint8_t>(std::lround(volts * 5.12));
        execute_control_command<std::uint8_t>(RoboClawCommand::set_main_voltage_max, value);
    }

    void RoboClaw::forward_m2(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_forward_m2, speed);
    }

    void RoboClaw::backward_m2(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_backward_m2, speed);
    }

    void RoboClaw::drive_m1_7bit(std::uint8_t speed) {
       execute_compatibility_motor_command(RoboClawCommand::drive_m1_7bit, speed); 
    }

    void RoboClaw::drive_m2_7bit(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_m2_7bit, speed);
    }

    void RoboClaw::forward_mixed_mode(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_forward_mixed_mode, speed);
    }

    void RoboClaw::backward_mixed_mode(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_backward_mixed_mode, speed);
    }

    void RoboClaw::turn_right_mixed_mode(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::turn_right_mixed_mode, speed);
    }

    void RoboClaw::turn_left_mixed_mode(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::turn_left_mixed_mode, speed);
    }

    void RoboClaw::drive_7bit(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::drive_7bit, speed);
    }

    void RoboClaw::turn_7bit(std::uint8_t speed) {
        execute_compatibility_motor_command(RoboClawCommand::turn_7bit, speed);
    }
    

    void RoboClaw::set_serial_timeout(double sec) {
        if (sec < 0.0 || 25.5 < sec) {
            throw std::out_of_range(" sec must be in range of 0.0s to 25.5s, but got " + std::to_string(sec) + "s");
        }

        std::uint8_t value = static_cast<std::uint8_t>(std::lround(sec * 10.0));
        execute_control_command<std::uint8_t>(RoboClawCommand::set_serial_timeout, value); 
    }

    double RoboClaw::get_serial_timeout() {
        auto [value] = execute_query_command<std::uint8_t>(RoboClawCommand::get_serial_timeout);
        
        return static_cast<double>(value) / 10.0;
    }


    std::string RoboClaw::get_firmware_version() {
        std::uint32_t attempts = max_attempts_;

        std::exception_ptr ep = nullptr;

        while (attempts--) {
            try {
                clear_input_buffer();
                send_query_request(RoboClawCommand::get_firmware_version);      
                constexpr std::size_t max_version_string_size_bytes = 48;
                constexpr std::size_t crc_size_bytes = 2;

                std::uint8_t buffer[max_version_string_size_bytes];

                std::size_t num_read = 0;
                bool terminated = false;

                while (num_read < max_version_string_size_bytes) {
                    read_exactly(&buffer[num_read++], 0, 1);
        
                    if (num_read >= 2 &&
                        buffer[num_read - 2] == '\n' &&
                        buffer[num_read - 1] == '\0'
                    ) {
                        terminated = true;
                        break;
                    }
                }

                if (!terminated)
                    throw InvalidResponseException("terminator not found");

                std::uint8_t crc_buffer[crc_size_bytes];
                read_exactly(crc_buffer, 0, crc_size_bytes);

                std::uint16_t expected_crc = 0;

                crc16_update(expected_crc, address_);
                crc16_update(expected_crc, command_value(RoboClawCommand::get_firmware_version));
                crc16_update_many(expected_crc, buffer, num_read);

                std::uint16_t actual_crc = static_cast<std::uint16_t>(
                    static_cast<std::uint16_t>(crc_buffer[0] << 8) |
                    static_cast<std::uint16_t>(crc_buffer[1])
                );

                if (expected_crc != actual_crc) 
                    throw CRCMismatchException(expected_crc, actual_crc);

                return std::string(
                    reinterpret_cast<const char *>(buffer),
                    num_read - 2
                );
            } catch (const SerialTimeoutException&)  {
                ep = std::current_exception();
                break;;
            } catch (const CRCMismatchException&) {
                ep = std::current_exception();
                continue;
            } catch (const SerialException&) {
                ep = std::current_exception();
                break;
            } catch (const std::system_error&) {
                ep = std::current_exception();
                break;
            } catch (const std::exception&) {
                ep = std::current_exception();
                break;
            }
        }

        if (ep != nullptr) std::rethrow_exception(ep);

        __builtin_unreachable();
    }

    double RoboClaw::get_main_battery_voltage() {
        auto [value] = execute_query_command<std::uint16_t>(RoboClawCommand::get_main_battery_voltage);

        return value / 10.0;
    }

    double RoboClaw::get_logic_battery_voltage() {
        auto [value] = execute_query_command<std::uint16_t>(RoboClawCommand::get_logic_battery_voltage);

        return value / 10.0;
    }

    void RoboClaw::set_logic_voltage_level_minimum(double volts) {
        validate_volts(volts);
        std::uint8_t value = static_cast<std::uint8_t>((volts - 6) * 5);
        execute_control_command<std::uint8_t>(RoboClawCommand::set_min_logic_battery_voltage_level, value);
    }

    void RoboClaw::set_logic_voltage_level_maximum(double volts) {
        validate_volts(volts);
        std::uint8_t value = static_cast<std::uint8_t>(std::lround(volts * 5.12));
        execute_control_command<std::uint8_t>(RoboClawCommand::set_max_logic_battery_voltage_level, value);
    }

    std::tuple<std::int16_t, std::int16_t> RoboClaw::get_motor_pwms() {
        return execute_query_command<std::int16_t, std::int16_t>(RoboClawCommand::get_motor_pwms);
    }

    std::tuple<double, double> RoboClaw::get_motor_currents() {
        auto [m1, m2] = execute_query_command<std::uint16_t, std::uint16_t>(RoboClawCommand::get_motor_currents);

        return { m1 / 100.0, m2 / 100.0 };
    }
    
    void RoboClaw::set_main_battery_voltages(double min_volts, double max_volts) {
        if (min_volts < 6.0 || min_volts > 34.0) {
            throw std::out_of_range(
                "min_volts expected [6.0, 34.0], but got " + std::to_string(min_volts)
            );
        }

        if (max_volts < 6.0 || max_volts > 34.0) {
            throw  std::out_of_range(
                "max_volts expected [6.0, 34.0], but got " + std::to_string(max_volts)
            );
        }

        if (min_volts > max_volts) {
            throw std::invalid_argument(
                "min_volts must not exceed max_volts"   
            );
        }

        execute_control_command<std::uint16_t, std::uint16_t>(
            RoboClawCommand::set_main_battery_voltages,
            static_cast<std::uint16_t>(std::lround(min_volts * 10.0)),
            static_cast<std::uint16_t>(std::lround(max_volts * 10.0))
        );
    }

    void RoboClaw::set_logic_battery_voltages(double min_volts, double max_volts) {
        validate_volts(min_volts, "min_volts");
        validate_volts(max_volts, "max_volts");

        if (min_volts > max_volts) 
            throw std::invalid_argument("min_volts must not exceed max_volts");
        
        
        execute_control_command<std::uint16_t, std::uint16_t>(
            RoboClawCommand::set_logic_battery_voltages,
            static_cast<std::uint16_t>(std::lround(min_volts * 10.0)),
            static_cast<std::uint16_t>(std::lround(max_volts * 10.0))
        );
    }

    std::tuple<double, double> RoboClaw::get_main_battery_voltages() {
        auto [min_volts, max_volts] = execute_query_command<std::uint16_t, std::uint16_t>(
            RoboClawCommand::get_main_battery_voltages
        );

        return { min_volts / 10.0, max_volts / 10.0 }; 
    }

    std::tuple<double, double> RoboClaw::get_logic_battery_voltages() {
        auto [min_volts, max_volts] = execute_query_command<std::uint16_t, std::uint16_t>(
            RoboClawCommand::get_logic_battery_voltages
        );

        return { min_volts / 10.0, max_volts / 10.0 };
    }

    void RoboClaw::set_s3_s4_s5_modes(std::uint8_t s3_mode, std::uint8_t s4_mode, std::uint8_t s5_mode) {
        execute_control_command<std::uint8_t, std::uint8_t, std::uint8_t>(
            RoboClawCommand::set_s3_s4_s5_modes,
            s3_mode,
            s4_mode,
            s5_mode
        );
    } 

    std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> RoboClaw::get_s3_s4_s5_modes() {
        return execute_query_command<std::uint8_t, std::uint8_t, std::uint8_t>(RoboClawCommand::get_s3_s4_s5_modes);
    }

    void RoboClaw::set_deadband(double reverse_pct, double forward_pct) {
        if (reverse_pct < 0.0 || reverse_pct > 25.0) {
            throw std::out_of_range(
                "reverse_pct must be in range of 0.0% to 25.0%, but got " + std::to_string(reverse_pct) + "%"
            );
        }

        if (forward_pct < 0.0 || forward_pct > 25.0) {
            throw std::out_of_range(
                "forward_pct must be in range of 0.0% to 25.0%, but got " + std::to_string(forward_pct) + "%"
            );
        }

        execute_control_command<std::uint8_t, std::uint8_t>(
            RoboClawCommand::set_deadband_rc_analog_controls,
            static_cast<std::uint8_t>(reverse_pct * 10.0),
            static_cast<std::uint8_t>(forward_pct * 10.0)
        );
    }

    std::tuple<double, double> RoboClaw::get_deadband() {
        auto [rev, fwd] = execute_query_command<std::uint8_t, std::uint8_t>(
            RoboClawCommand::get_deadband_rc_analog_controls
        );

        return { rev / 10.0, fwd / 10.0 };
    }

    void RoboClaw::restore_defaults() {
        execute_control_command(RoboClawCommand::restore_defaults);
    }

    std::tuple<std::uint32_t, std::uint32_t> RoboClaw::get_default_duty_accels() {
        return execute_query_command<std::uint32_t, std::uint32_t>(RoboClawCommand::get_default_duty_accels);
    }

    double RoboClaw::get_temperature() {
        auto [raw] = execute_query_command<std::uint16_t>(RoboClawCommand::get_temperature);
        return raw / 10.0; 
    }

    double RoboClaw::get_temperature2() {
        auto [raw] = execute_query_command<std::uint16_t>(RoboClawCommand::get_temperature2);
        return raw / 10.0;
    }

    
    std::uint16_t RoboClaw::get_unit_status() {
        auto [status] = execute_query_command<std::uint16_t>(RoboClawCommand::get_status);
        return status;
    }

    std::tuple<std::uint8_t, std::uint8_t> RoboClaw::get_encoder_modes() {
        return execute_query_command<std::uint16_t, std::uint16_t>(RoboClawCommand::get_encoder_modes);
    }

    void RoboClaw::set_m1_encoder_mode(std::uint8_t flag) {
        execute_control_command<std::uint8_t>(RoboClawCommand::set_m1_encoder_mode, flag);
    }

    void RoboClaw::set_m2_encoder_mode(std::uint8_t flag) {
        execute_control_command<std::uint8_t>(RoboClawCommand::set_m2_encoder_mode, flag);
    }

    void RoboClaw::write_settings_to_eeprom() {
        execute_control_command(RoboClawCommand::write_settings_to_eeprom);
    }

    std::tuple<std::uint8_t, std::uint8_t> RoboClaw::read_settings_from_eeprom() {
        return execute_query_command<std::uint8_t, std::uint8_t>(
            RoboClawCommand::read_settings_from_eeprom
        );
    }

    void RoboClaw::set_standard_config_settings(std::uint16_t config) {
        execute_control_command<std::uint16_t>(RoboClawCommand::set_standard_config_settings, config);
    }

    std::uint16_t RoboClaw::get_standard_config_settings() {
        auto [config] = execute_query_command<std::uint16_t>(
            RoboClawCommand::get_standard_config_settings
        );

        return config;
    }

    void RoboClaw::set_m1_max_current_limit(double max_current) {    
        if (max_current < 0.0 || max_current > 120.0) 
            throw std::out_of_range("max_current must be in range of 0.0A to 120.0A, but got " + std::to_string(max_current) + "A");
        
        std::uint32_t value = static_cast<std::uint32_t>(std::lround(max_current * 100.0));
        execute_control_command<std::uint32_t, std::uint32_t>(
            RoboClawCommand::set_m1_max_current_limit,
            value,
            0U
        );
    }

    void RoboClaw::set_m2_max_current_limit(double max_current) {
        if (max_current < 0.0 || max_current > 120.0) 
            throw std::out_of_range("max_current must be in range of 0.0A to 120.0A, but got " + std::to_string(max_current) + "A");

        std::uint32_t value = static_cast<std::uint32_t>(std::lround(max_current * 100.0));
        execute_control_command<std::uint32_t, std::uint32_t>(
            RoboClawCommand::set_m2_max_current_limit,
            value,
            0U
        );
    }

    double RoboClaw::get_m1_max_current_limit() {
        auto [max_current, _] = execute_query_command<std::uint32_t, std::uint32_t>(RoboClawCommand::get_m1_max_current_limit);
        
        return static_cast<double>(max_current) / 100.0;
    }

    double RoboClaw::get_m2_max_current_limit() {
        auto [max_current, _] = execute_query_command<std::uint32_t, std::uint32_t>(RoboClawCommand::get_m2_max_current_limit);

        return static_cast<double>(max_current) / 100.0;
    }

    void RoboClaw::set_pwm_mode(std::uint8_t mode) {
        execute_control_command<std::uint8_t>(RoboClawCommand::set_pwm_mode, mode);
    }

    std::uint8_t RoboClaw::get_pwm_mode() {
        auto [mode] =  execute_query_command<std::uint8_t>(RoboClawCommand::get_pwm_mode);
        return mode;
    }

    std::tuple<std::uint32_t, std::uint8_t> RoboClaw::get_encoder_count_m1() {
        return execute_query_command<std::uint32_t, std::uint8_t>(RoboClawCommand::get_encoder_count_m1); 
    }

    std::tuple<std::uint32_t, std::uint8_t> RoboClaw::get_encoder_count_m2() {
        return execute_query_command<std::uint32_t, std::uint8_t>(RoboClawCommand::get_encoder_count_m2);
    }
    
    std::tuple<std::uint32_t, std::uint8_t> RoboClaw::get_speed_m1() {
        return execute_query_command<std::uint32_t, std::uint8_t>(RoboClawCommand::get_speed_m1);
    }

    std::tuple<std::uint32_t, std::uint8_t> RoboClaw::get_speed_m2() {
        return execute_query_command<std::uint32_t, std::uint8_t>(RoboClawCommand::get_speed_m2);
    }

    void RoboClaw::reset_encoder_counts() {
        execute_control_command(RoboClawCommand::reset_encoders_counts);
    }

    void RoboClaw::set_encoder1_register_count(std::uint32_t value) {
        execute_control_command<std::uint32_t>(RoboClawCommand::set_encoder1_count, value);
    }

    void RoboClaw::set_encoder2_register_count(std::uint32_t value) {
        execute_control_command<std::uint32_t>(RoboClawCommand::set_encoder2_count, value);
    }

    std::tuple<std::uint32_t, std::uint8_t> RoboClaw::get_raw_speed_m1() {
        return execute_query_command<std::uint32_t, std::uint8_t>(RoboClawCommand::get_raw_speed_m1);
    }

    std::tuple<std::uint32_t, std::uint8_t> RoboClaw::get_raw_speed_m2() {
        return execute_query_command<std::uint32_t, std::uint8_t>(RoboClawCommand::get_raw_speed_m2);
    }

    std::tuple<std::uint32_t, std::uint32_t> RoboClaw::get_encoder_counts() {
        return execute_query_command<std::uint32_t, std::uint32_t>(RoboClawCommand::get_encoder_counts);
    }

    std::tuple<std::uint32_t, std::uint32_t> RoboClaw::get_instantaneous_speeds() {
        return execute_query_command<std::uint32_t, std::uint32_t>(RoboClawCommand::get_instantaneous_speeds);
    }

    void RoboClaw::set_velocity_pid_m1(double kp, double ki, double kd, std::uint32_t qpps) {
        std::uint32_t p = to_fixed_16_16("kp", kp);
        std::uint32_t i = to_fixed_16_16("ki", ki);
        std::uint32_t d = to_fixed_16_16("kd", kd);

        execute_control_command<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>(
            RoboClawCommand::set_velocity_pid_m1,
            d,
            p,
            i, 
            qpps
        );
    }

    void RoboClaw::set_velocity_pid_m2(double kp, double ki, double kd, std::uint32_t qpps) {
        std::uint32_t p = to_fixed_16_16("kp", kp);
        std::uint32_t i = to_fixed_16_16("ki", ki);
        std::uint32_t d = to_fixed_16_16("kd", kd);

        execute_control_command<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>(
            RoboClawCommand::set_velocity_pid_m2,
            d,
            p,
            i,
            qpps
        );
    }

    void RoboClaw::drive_m1_signed_duty_cycle(std::int16_t duty) {
        execute_control_command<std::uint16_t>(RoboClawCommand::drive_m1_signed_duty, duty);
    }

    void RoboClaw::drive_m2_signed_duty_cycle(std::int16_t duty) {
        execute_control_command<std::uint16_t>(RoboClawCommand::drive_m2_signed_duty, duty);
    }

    void RoboClaw::drive_m1_m2_signed_duty_cycle(std::int16_t m1_duty, std::int16_t m2_duty) {
        execute_control_command<std::int16_t, std::int16_t>(
            RoboClawCommand::drive_m1_m2_signed_duty,
            m1_duty,
            m2_duty
        );
    }

    void RoboClaw::drive_m1_signed_speed(std::int32_t speed) {
        execute_control_command<std::int32_t>(RoboClawCommand::drive_m1_signed_speed, speed);
    }

    void RoboClaw::drive_m2_signed_speed(std::int32_t speed) {
        execute_control_command<std::int32_t>(RoboClawCommand::drive_m2_signed_speed, speed);
    }

    void RoboClaw::drive_m1_m2_signed_speed(std::int32_t m1_speed, std::int32_t m2_speed) {
        execute_control_command<std::int32_t, std::int32_t>(
            RoboClawCommand::drive_m1_m2_signed_speed,
            m1_speed,
            m2_speed
        );
    }

    void RoboClaw::drive_m1_signed_speed_accel(std::int32_t speed, std::uint32_t accel) {
        execute_control_command<std::uint32_t, std::int32_t>(
            RoboClawCommand::drive_m1_signed_speed_accel,
            accel,
            speed
        );
    }

    void RoboClaw::drive_m2_signed_speed_accel(std::int32_t speed, std::uint32_t accel) {
        execute_control_command<std::uint32_t, std::int32_t>(
            RoboClawCommand::drive_m2_signed_speed_accel,
            accel,
            speed
        );
    }

    void RoboClaw::drive_m1_m2_signed_speed_accel(std::int32_t m1_speed, std::int32_t m2_speed, std::uint32_t accel) {
        execute_control_command<std::uint32_t, std::int32_t, std::int32_t>(
            RoboClawCommand::drive_m1_m2_signed_speed_accel,
            accel,
            m1_speed,
            m2_speed
        );
    }

    void RoboClaw::drive_m1_m2_signed_speed_accel(
        std::int32_t m1_speed,
        std::uint32_t m1_accel,
        std::int32_t m2_speed,
        std::uint32_t m2_accel
    ) {
        execute_control_command<std::uint32_t, std::int32_t, std::uint32_t, std::int32_t>(
            RoboClawCommand::drive_m1_m2_individual_signed_speed_accel,
            m1_accel,
            m1_speed,
            m2_accel,
            m2_speed
        );
    }

    void RoboClaw::drive_m1_signed_duty_accel(std::int16_t duty, std::uint32_t accel) {
        execute_control_command<std::int16_t, std::uint32_t>(
            RoboClawCommand::drive_m1_signed_duty_accel,
            duty,
            accel
        );
    }

    void RoboClaw::drive_m2_signed_duty_accel(std::int16_t duty, std::uint32_t accel) {
        execute_control_command<std::int16_t, std::uint32_t>(
            RoboClawCommand::drive_m2_signed_duty_accel,
            duty,
            accel
        );
    }

    void RoboClaw::drive_m1_m2_signed_duty_accel(
        std::int16_t m1_duty, 
        std::uint32_t m1_accel,
        std::int16_t m2_duty,
        std::uint32_t m2_accel
    ) {
        execute_control_command<std::int16_t, std::uint32_t, std::int16_t, std::uint32_t>(
            RoboClawCommand::drive_m1_m2_signed_duty_accel,
            m1_duty,
            m1_accel,
            m2_duty,
            m2_accel
        );
    }

    std::tuple<double, double, double, std::uint32_t> RoboClaw::get_velocity_pid_m1() {
        auto [p, i, d, qpps] = execute_query_command<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>(
            RoboClawCommand::get_velocity_pid_m1
        );

        double kp = from_fixed_16_16(p);
        double ki = from_fixed_16_16(i);
        double kd = from_fixed_16_16(d);

        return { kp, ki, kd, qpps };
    }

    std::tuple<double, double, double, std::uint32_t> RoboClaw::get_velocity_pid_m2() {
        auto [p, i, d, qpps] = execute_query_command<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>(
            RoboClawCommand::get_velocity_pid_m2
        );

        double kp = from_fixed_16_16(p);
        double ki = from_fixed_16_16(i);
        double kd = from_fixed_16_16(d);

        return { kp, ki, kd, qpps };
    }
        

    void RoboClaw::set_default_duty_accel_m1(std::uint32_t accel) {
        execute_control_command<std::uint32_t>(
            RoboClawCommand::set_default_duty_accel_m1,
            accel
        );
    }

    void RoboClaw::set_default_duty_accel_m2(std::uint32_t accel) {
        execute_control_command<std::uint32_t>(
            RoboClawCommand::set_default_duty_accel_m2,
            accel
        );
    }
    
} //namespace kait3_roboclaw_hardware

