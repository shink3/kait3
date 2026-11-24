#ifndef KAIT3_ROBOCLAW_HARDWARE_ROBOCLAW_HARDWARE_HPP_
#define KAIT3_ROBOCLAW_HARDWARE_ROBOCLAW_HARDWARE_HPP_

#include "kait3_roboclaw_hardware/driver/roboclaw.hpp"
#include <hardware_interface/hardware_component_interface.hpp>
#include <string>
#include <memory>
#include <type_traits>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <array>
#include <optional>

namespace kait3_roboclaw_hardware {


    class RoboClawHardware : public hardware_interface::SystemInterface {
    private:
    	struct VelocityPidSettings {
		    double kp;
		    double ki;
		    double kd;
		    std::uint32_t qpps;
		};

		struct DriveSpecifications {
		    double counts_per_rad;
		    std::string motion_strategy;
		    std::uint32_t default_acceleration;
		    std::optional<double> watchdog_timeout_sec;
		    std::optional<double> m1_max_current;
		    std::optional<double> m2_max_current;
		    std::optional<VelocityPidSettings> m1_pid_settings;
		    std::optional<VelocityPidSettings> m2_pid_settings;
		};
    
    private:

        template<typename T>
        T get_hardware_parameter_or_default(const std::string& key, const T& default_value);

        template<typename T>
        std::optional<T> get_hardware_parameter_or_nullopt(const std::string& key);

        void write_velocity(double left_vel, double right_vel);
    
    public:
        RoboClawHardware() noexcept;

    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams& params
        ) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        hardware_interface::return_type read(
            const rclcpp::Time& time,
            const rclcpp::Duration& period
        ) override;

        hardware_interface::return_type write(
            const rclcpp::Time& time,
            const rclcpp::Duration& period
        ) override;

    private:
        DriveSpecifications specs_;
        std::unique_ptr<driver::RoboClaw> roboclaw_driver_;
        std::array<std::uint32_t, 2> previous_encoder_counts_;
        std::array<double, 2> position_;
        bool wait_for_zero_velocity_after_e_stop_;
    };

    template<typename T>
    T RoboClawHardware::get_hardware_parameter_or_default(const std::string& key, const T& default_value) {
        const auto hw_params = get_hardware_info().hardware_parameters;

        auto it = hw_params.find(key);
        if (it != hw_params.end()) {
            const std::string& value = it->second;

            if constexpr (std::is_same_v<T, std::string>) {
                return value;
            }
            else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
                if constexpr (std::is_unsigned_v<T>) {
                    return static_cast<T>(std::stoull(value, nullptr, 0));
                } else {
                    return static_cast<T>(std::stoll(value, nullptr, 0));
                }
            } else if constexpr (std::is_floating_point_v<T>) {
                return static_cast<T>(std::stod(value));
            } else if constexpr (std::is_same_v<T, bool>) {
                return (value == "true" || value == "1");
            }
        }
        return default_value;
    }

    template<typename T>
    std::optional<T> RoboClawHardware::get_hardware_parameter_or_nullopt(const std::string& key) {
        const auto hw_params = get_hardware_info().hardware_parameters;
        auto it = hw_params.find(key);

        if (it != hw_params.end()) {
            const std::string& value = it->second;

            if constexpr (std::is_same_v<T, std::string>) {
                return value;
            } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
                if constexpr (std::is_unsigned_v<T>) {
                    return static_cast<T>(std::stoull(value, nullptr, 0));
                } else {
                    return static_cast<T>(std::stoll(value, nullptr, 0));
                }
            } else if constexpr (std::is_floating_point_v<T>) {
                return static_cast<T>(std::stod(value));
            } else if constexpr (std::is_same_v<T, bool>) {
                return (value == "true" || value == "1");
            }
        }
        return std::nullopt;
    }

} //namespace kait3_roboclaw_hardware

#endif //KAIT3_ROBOCLAW_HARDWARE_KAIT3_ROBOCLAW_HARDWARE_HPP_
