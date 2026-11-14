#ifndef KAIT3_ROBOCLAW_HARDWARE_DRIVER_ROBOCLAW_HPP_
#define KAIT3_ROBOCLAW_HARDWARE_DRIVER_ROBOCLAW_HPP_

#include "kait3_roboclaw_hardware/driver/command.hpp"
#include "kait3_roboclaw_hardware/driver/packet.hpp"
#include "kait3_roboclaw_hardware/driver/crc16.hpp"
#include "kait3_roboclaw_hardware/driver/exceptions.hpp"
#include <exception>
#include <stdexcept>
#include <string>
#include <chrono>
#include <cassert>
#include <system_error>
#include <termios.h>

namespace kait3_roboclaw_hardware::driver {

    class RoboClaw {
    public:
        using duration = std::chrono::steady_clock::duration;

        static constexpr std::uint32_t default_baud_rate        = 115200;
        static constexpr std::uint8_t  default_address          = 0x80;
        static constexpr duration      default_read_timeout     = std::chrono::seconds(1);
        static constexpr duration      default_write_timeout    = std::chrono::seconds(1);
        static constexpr std::uint32_t default_max_attempts     = 1;
   
    public:
        RoboClaw(
            const std::string& path,
            std::uint32_t baud_rate = default_baud_rate,
            std::uint8_t address = default_address,
            const duration& read_timeout = default_read_timeout,
            const duration& write_timeout = default_write_timeout,
            bool exclusive_access = false,
            bool unacknowledged = false,
            std::uint32_t max_attempts = default_max_attempts
        );

        RoboClaw(const RoboClaw&) = delete;

        RoboClaw& operator=(const RoboClaw&) = delete;

        RoboClaw(RoboClaw&&) noexcept = delete;

        RoboClaw& operator=(RoboClaw&&) noexcept = delete;

        ~RoboClaw();

    private:
        static constexpr bool is_valid_baud_rate(std::uint32_t buad_rate) noexcept;
        
        static constexpr bool is_valid_address(std::uint8_t address) noexcept;

        static constexpr std::uint16_t command_group(RoboClawCommand command) noexcept;

        static constexpr std::uint8_t command_value(RoboClawCommand command) noexcept;

        void write_all(const std::uint8_t *source, std::size_t count);

        void read_exactly(std::uint8_t *destination, std::size_t offset, std::size_t count);

        void clear_input_buffer();

        template<typename... Args>
        void send_control_request(RoboClawCommand command, Args... args); 

        void send_query_request(RoboClawCommand command);

        void recv_control_response();

        template<typename... Args>
        std::tuple<Args...> recv_query_response(RoboClawCommand command);
 
    public:
        void open();

        void close();

        bool is_open() const noexcept;

        const std::string& device_path() const noexcept;

        std::uint32_t baud_rate() const noexcept;  

        void set_baud_rate(std::uint32_t baud_rate);

        std::uint8_t address() const noexcept;

        void set_address(std::uint8_t address);

        const duration& read_timeout() const noexcept;  

        void set_read_timeout(const duration& timeout);

        const duration& write_timeout() const noexcept;

        void set_write_timeout(const duration& timeout);

        bool exclusive_access() const noexcept;

        void set_exclusive_access(bool exclusice_access);

        bool unacknowledged() const noexcept;

        void set_unacknoledged(bool unacknowledged);

        std::uint32_t max_attempts() const noexcept;

        void set_max_attempts(std::uint32_t attempts);

        template<typename... Args>
        void execute_control_command(RoboClawCommand command,  Args... args);

        template<typename... Args>
        std::tuple<Args...> execute_query_command(RoboClawCommand command);

private:
        void execute_compatibility_motor_command(
            RoboClawCommand command, 
            std::uint8_t speed
        );
public:
        
        void forward_m1(std::uint8_t speed);

        void backward_m1(std::uint8_t speed);

        void set_main_voltage_minimum(double volts);

        void set_main_voltage_maximum(double volts);

        void forward_m2(std::uint8_t speed);

        void backward_m2(std::uint8_t speed);

        void drive_m1_7bit(std::uint8_t speed);

        void drive_m2_7bit(std::uint8_t speed);

        void forward_mixed_mode(std::uint8_t speed);

        void backward_mixed_mode(std::uint8_t speed);

        void turn_right_mixed_mode(std::uint8_t speed);

        void turn_left_mixed_mode(std::uint8_t speed);

        void drive_7bit(std::uint8_t speed);

        void turn_7bit(std::uint8_t speed);
        
        void set_serial_timeout(double sec);

        double get_serial_timeout();

        std::string get_firmware_version();
        
        double get_main_battery_voltage();

        double get_logic_battery_voltage();
       
        void set_logic_voltage_level_minimum(double volts);

        void set_logic_voltage_level_maximum(double volts);

        std::tuple<std::int16_t, std::int16_t> get_motor_pwms();

        std::tuple<double, double> get_motor_currents();
        
        void set_main_battery_voltages(double min_volts, double max_volts);

        void set_logic_battery_voltages(double min_volts, double max_volts);

        std::tuple<double, double> get_main_battery_voltages();

        std::tuple<double, double> get_logic_battery_voltages();

       void set_s3_s4_s5_modes(std::uint8_t s3_mode, std::uint8_t s4_mode, std::uint8_t s5_mode);

       std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> get_s3_s4_s5_modes();
        
        void set_deadband(double reverse_pct, double forward_pct);

        std::tuple<double, double> get_deadband();

        void restore_defaults();

        std::tuple<std::uint32_t, std::uint32_t> get_default_duty_accels();
        
        double get_temperature();

        double get_temperature2();

        std::uint16_t get_unit_status(); 
        
        std::tuple<std::uint8_t, std::uint8_t> get_encoder_modes();

        void set_m1_encoder_mode(std::uint8_t flags);
        
        void set_m2_encoder_mode(std::uint8_t flags);
        
        void write_settings_to_eeprom();

        std::tuple<std::uint8_t, std::uint8_t> read_settings_from_eeprom();

        void set_standard_config_settings(std::uint16_t config);

        std::uint16_t get_standard_config_settings();

        void set_m1_max_current_limit(double max_current);

        void set_m2_max_current_limit(double max_current);

        double get_m1_max_current_limit();

        double get_m2_max_current_limit();

        void set_pwm_mode(std::uint8_t mode);

        std::uint8_t get_pwm_mode();
        
        std::tuple<std::uint32_t, std::uint8_t> get_encoder_count_m1();

        std::tuple<std::uint32_t, std::uint8_t> get_encoder_count_m2();

        std::tuple<std::uint32_t, std::uint8_t> get_speed_m1();

        std::tuple<std::uint32_t, std::uint8_t> get_speed_m2();

        void reset_encoder_counts();

        void set_encoder1_register_count(std::uint32_t value);

        void set_encoder2_register_count(std::uint32_t value);

        std::tuple<std::uint32_t, std::uint8_t> get_raw_speed_m1();

        std::tuple<std::uint32_t, std::uint8_t> get_raw_speed_m2();
    
        std::tuple<std::uint32_t, std::uint32_t> get_encoder_counts();

        std::tuple<std::uint32_t, std::uint32_t> get_instantaneous_speeds();

        void set_velocity_pid_m1(double kp, double ki, double kd, std::uint32_t qpps);

        void set_velocity_pid_m2(double kp, double ki, double kd, std::uint32_t qpps);

        void drive_m1_signed_duty_cycle(std::int16_t duty);

        void drive_m2_signed_duty_cycle(std::int16_t duty);

        void drive_m1_m2_signed_duty_cycle(std::int16_t m1_duty, std::int16_t m2_duty);

        void drive_m1_signed_speed(std::int32_t speed);

        void drive_m2_signed_speed(std::int32_t speed);

        void drive_m1_m2_signed_speed(std::int32_t m1_speed, std::int32_t m2_speed);

        void drive_m1_signed_speed_accel(std::int32_t speed, std::uint32_t accel);

        void drive_m2_signed_speed_accel(std::int32_t speed, std::uint32_t accel);

        void drive_m1_m2_signed_speed_accel(std::int32_t m1_speed, std::int32_t m2_speed, std::uint32_t accel);

        void drive_m1_m2_signed_speed_accel(
            std::int32_t m1_speed,
            std::uint32_t m1_accel, 
            std::int32_t m2_speed, 
            std::uint32_t m2_accel
        );
        
        void drive_m1_signed_duty_accel(
            std::int16_t duty,
            std::uint32_t accel
        );

        void drive_m2_signed_duty_accel(
            std::int16_t duty,
            std::uint32_t accel
        );

        void drive_m1_m2_signed_duty_accel(
            std::int16_t m1_duty,
            std::uint32_t m1_accel,
            std::int16_t m2_duty,
            std::uint32_t m2_accel
        );

        std::tuple<double, double, double, std::uint32_t> get_velocity_pid_m1();

        std::tuple<double, double, double, std::uint32_t> get_velocity_pid_m2();
       
        void set_default_duty_accel_m1(std::uint32_t accel);

        void set_default_duty_accel_m2(std::uint32_t accel);

    private:
        int fd_;
        std::string device_path_;
        std::uint32_t baud_rate_;
        std::uint8_t address_;
        duration read_timeout_;
        duration write_timeout_;
        std::vector<std::uint8_t> tx_buffer_;
        std::vector<std::uint8_t> rx_buffer_;
        bool exclusive_access_;
        bool unacknowledged_;
        std::uint32_t max_attempts_;
    };


    template<typename... Args>
    void RoboClaw::send_control_request(RoboClawCommand command, Args... args) {
        static_assert(
            (std::is_integral_v<Args> && ...), 
            "RoboClaw command arguments must be integer types"
        );
    
        assert(command_group(command) == ROBOCLAW_CMD_GROUP_CTL_FLAG);
        
        PacketBufferWriter writer(tx_buffer_);
        writer.write_header(address_, command_value(command));

        if constexpr (sizeof...(Args) > 0) {
            writer.write_many(args...);
        
            std::uint16_t crc = 0;
            crc16_update_many(crc, tx_buffer_.data(), writer.num_written());
            writer.write(crc);
        }

        writer.flush();
        write_all(tx_buffer_.data(), writer.num_written());
    }

    template<typename... Args>
    std::tuple<Args...> RoboClaw::recv_query_response(RoboClawCommand command) {
        static_assert(
            (std::is_integral_v<Args> && ...), 
            "RoboClaw response type must be integer type"
        );
        static_assert(
            sizeof...(Args) > 0, 
            "Query response must contain at least one element"
        );

        assert(command_group(command) == ROBOCLAW_CMD_GROUP_QUERY_FLAG);

        static constexpr std::size_t data_size_bytes = (sizeof(Args) + ...);
        static constexpr std::size_t crc_size_bytes = sizeof(std::uint16_t);
        static constexpr std::size_t response_size_bytes = data_size_bytes + crc_size_bytes;

        assert(rx_buffer_.size() >= response_size_bytes);
    
        read_exactly(rx_buffer_.data(), 0, response_size_bytes);

        PacketBufferReader reader(rx_buffer_);
        const auto data = reader.read_many<Args...>();
        const uint16_t actual_crc = reader.read<std::uint16_t>();

        std::uint16_t expected_crc = 0;
        crc16_update(expected_crc, address_);
        crc16_update(expected_crc, command_value(command));
        crc16_update_many(expected_crc, rx_buffer_.data(), data_size_bytes);

        if (expected_crc != actual_crc) 
            throw CRCMismatchException(expected_crc, actual_crc);

        return data;
    }

    template<typename... Args>
    void RoboClaw::execute_control_command(RoboClawCommand command, Args... args) {
        if (command_group(command) != ROBOCLAW_CMD_GROUP_CTL_FLAG) 
            throw std::invalid_argument("execute_control_command requires control command (e.g. SET / DRIVE)");
        std::uint32_t attempts = max_attempts_;
        std::exception_ptr ep = nullptr;
        
        while (attempts--) {
            try {
                clear_input_buffer();
                send_control_request(command, args...);
                recv_control_response();
                return;
            } catch (const SerialTimeoutException&) {
                ep = std::current_exception();
                break;
            } catch (const InvalidResponseException&) {
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

    template<typename...Args>
    std::tuple<Args...> RoboClaw::execute_query_command(RoboClawCommand command) {
        if (command_group(command) != ROBOCLAW_CMD_GROUP_QUERY_FLAG)
            throw  std::invalid_argument("execute_query_command requires query command (e.g GET)");
        
        std::uint32_t attempts = max_attempts_;
        std::exception_ptr ep = nullptr;

        while (attempts--) {
            try {
                clear_input_buffer();
                send_query_request(command);
                return recv_query_response<Args...>(command);
            } catch (const SerialTimeoutException&) {
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

} //namespace kait3_roboclaw_hardware::driver

#endif //KAIT3_ROBOCLAW_HARDWARE_DRIVER_ROBOCLAW_HPP_

