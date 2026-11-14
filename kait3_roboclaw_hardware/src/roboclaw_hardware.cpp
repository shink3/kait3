#include "kait3_roboclaw_hardware/roboclaw_hardware.hpp"
#include <rcutils/macros.h>

namespace kait3_roboclaw_hardware {
        
    RoboClawHardware::RoboClawHardware() noexcept
    : specs_(),
      roboclaw_driver_(nullptr),
      previous_encoder_counts_(),
      position_() {}
    
    void RoboClawHardware::write_velocity(double left_vel, double right_vel) {

        std::int32_t m1_counts_per_sec = static_cast<std::int32_t>(left_vel * specs_.counts_per_rad);
        std::int32_t m2_counts_per_sec = static_cast<std::int32_t>(right_vel * specs_.counts_per_rad);

        if (specs_.motion_strategy == "speed")  {
            roboclaw_driver_->drive_m1_m2_signed_speed(m1_counts_per_sec, m2_counts_per_sec);
        }
        else if (specs_.motion_strategy == "speed_accel") { 
            roboclaw_driver_->drive_m1_m2_signed_speed_accel(m1_counts_per_sec, m2_counts_per_sec, specs_.default_acceleration);
        }
    }

    hardware_interface::CallbackReturn RoboClawHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params
    ) {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) 
            return hardware_interface::CallbackReturn::ERROR;

        auto logger = get_logger();

        RCLCPP_INFO(logger, "Initializing hardware...");
        
        RCLCPP_INFO(logger, "Validating joints");

        if (get_hardware_info().joints.size() != 2) {
            RCLCPP_FATAL(
                logger, 
                "Hardware info reports %zu joints, but 2 (diff drive) expected.",
                get_hardware_info().joints.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    
        for (const hardware_interface::ComponentInfo& joint : get_hardware_info().joints) {
            RCLCPP_INFO(logger, "Validating interfaces for joint '%s'", joint.name.c_str());
            
            if (joint.name != "left_wheel_joint" && joint.name != "right_wheel_joint") {
                RCLCPP_FATAL(logger, "Joint name must be either 'left_wheel_joint' or 'right_wheel_joint'!");
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces.size() != 1 || 
                joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    logger,
                    "Joint '%s' must have exactly one '%s' command interface!",
                    joint.name.c_str(),
                    hardware_interface::HW_IF_VELOCITY
                );

                return hardware_interface::CallbackReturn::ERROR;
            }

            const auto has_position = std::any_of(
                joint.state_interfaces.begin(),
                joint.state_interfaces.end(),
                [](const hardware_interface::InterfaceInfo& interface) {
                    return interface.name == hardware_interface::HW_IF_POSITION;  
                }
            );
            
            const auto has_velocity = std::any_of(
                joint.state_interfaces.begin(),
                joint.state_interfaces.end(),
                [](const hardware_interface::InterfaceInfo& interface) {
                    return interface.name == hardware_interface::HW_IF_VELOCITY;
                }
            );

            if (joint.state_interfaces.size() != 2 ||
                !has_position ||
                !has_velocity
            ) {
                RCLCPP_FATAL(
                    logger,
                    "Joint '%s' must have exactly two state interfaces, '%s' and '%s'!",
                    joint.name.c_str(),
                    hardware_interface::HW_IF_POSITION,
                    hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        std::string device_path = get_hardware_parameter_or_default<std::string>("device_path", "/dev/ttyUSB0");
        std::uint32_t baud_rate = get_hardware_parameter_or_default<std::uint32_t>("baud_rate", 115200);
        std::uint8_t address = get_hardware_parameter_or_default<std::uint8_t>("address", 0x80);
        std::uint32_t read_timeout_ms = get_hardware_parameter_or_default<std::uint32_t>("read_timeout_ms", 5);
        std::uint32_t write_timeout_ms = get_hardware_parameter_or_default<std::uint32_t>("write_timeout_ms", 5);
        bool exclusive_access = get_hardware_parameter_or_default<bool>("exclusive_access", false);
        bool unacknowledged = get_hardware_parameter_or_default<bool>("unacknowledged", false);
        std::uint32_t max_attempts = get_hardware_parameter_or_default<std::uint32_t>("max_attempts", 1);

        double wheel_radius = get_hardware_parameter_or_default<double>("wheel_radius", 0.231);
        std::uint32_t quad_counts_per_revolution = get_hardware_parameter_or_default<std::uint32_t>("counts_per_revolution", 2000);
        double gear_ratio = get_hardware_parameter_or_default<double>("gear_ratio", 22.75);
        
        std::string motion_strategy = get_hardware_parameter_or_default<std::string>("motion_strategy", "speed_accel");
        std::uint32_t default_acceleration = get_hardware_parameter_or_default<std::uint32_t>("default_acceleration", 15700);

        auto watchdog_timeout_ms = get_hardware_parameter_or_nullopt<std::uint32_t>("watchdog_timeout_ms");

        auto m1_max_current = get_hardware_parameter_or_nullopt<double>("m1_max_current");
        auto m2_max_current = get_hardware_parameter_or_nullopt<double>("m2_max_current");

        auto m1_kp = get_hardware_parameter_or_nullopt<double>("m1_kp");
        auto m1_ki = get_hardware_parameter_or_nullopt<double>("m1_ki");
        auto m1_kd = get_hardware_parameter_or_nullopt<double>("m1_kd");
        auto m1_qpps= get_hardware_parameter_or_nullopt<std::uint32_t>("m1_qpps");
        auto m2_kp = get_hardware_parameter_or_nullopt<double>("m2_kp");
        auto m2_ki = get_hardware_parameter_or_nullopt<double>("m2_ki");
        auto m2_kd = get_hardware_parameter_or_nullopt<double>("m2_kd");
        auto m2_qpps = get_hardware_parameter_or_nullopt<std::uint32_t>("m2_qpps");
        
        RCLCPP_INFO(logger, "Validating hardware parameter");

        if (wheel_radius <= 0.0) {
            RCLCPP_FATAL(logger, "Parameter `wheel_radius` must be greater than zero!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (quad_counts_per_revolution == 0) {
            RCLCPP_FATAL(logger, "Parameter `counts_per_revolution` cannot be zero!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (gear_ratio <= 0.0) {
            RCLCPP_FATAL(logger, "Parameter `gear_ratio` must be greater than zero!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        
        std::optional<double> watchdog_timeout_sec = std::nullopt;
        if (watchdog_timeout_ms.has_value()) {
            constexpr std::uint32_t min_timeout_ms = 0;
            constexpr std::uint32_t max_timeout_ms = 25500;
            constexpr double ms_to_s = 1.0 / 1000.0;

            if (watchdog_timeout_ms.value() < min_timeout_ms || max_timeout_ms < watchdog_timeout_ms.value()) {

                RCLCPP_FATAL(
                    logger,
                    "Parameter `watchdog_timeout_sec` must be in range of %.1fs to %.1fs! but got %f",
                    static_cast<double>(min_timeout_ms) * ms_to_s,
                    static_cast<double>(max_timeout_ms) * ms_to_s,
                    static_cast<double>(watchdog_timeout_ms.value()) * ms_to_s
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            RCLCPP_WARN(
                logger,
                "Parameter `watchdog_timeout_sec` set to 0, disabling the watchdog. "
                "Safe motor stopping on communication loss is not guaranteed"
            );
            
            watchdog_timeout_sec = static_cast<double>(watchdog_timeout_ms.value()) * ms_to_s;
        }
        
        if (motion_strategy != "speed" && 
            motion_strategy != "speed_accel") {
            RCLCPP_FATAL(
                logger, 
                "Parameter `motion_strategy` must be one of 'speed' or 'speed_accel'! Current: %s", 
                motion_strategy.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        constexpr double motor_min_current_a = 0.0;
        constexpr double motor_max_current_a = 120.0;
        if (m1_max_current.has_value()) {
            if (m1_max_current.value() < motor_min_current_a || motor_max_current_a < m1_max_current.value()) {
                RCLCPP_FATAL(
                    logger, 
                    "Parameter `m1_max_current` must be in the range of %.1fA to %.1fA!, but got %f",
                    motor_min_current_a,
                    motor_max_current_a,
                    m1_max_current.value()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        if (m2_max_current.has_value()) {
            if (m2_max_current.value() < motor_min_current_a || motor_max_current_a < m2_max_current.value()) {
                RCLCPP_FATAL(
                    logger,
                    "Parameter `m2_max_current` must be in range of %fA to %fA!, but got %f",
                    motor_min_current_a,
                    motor_max_current_a,
                    m2_max_current.value()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        std::optional<VelocityPidSettings> m1_pid_settings = std::nullopt;
        if (m1_kp && m1_ki && m1_kd && m1_qpps) {
            if (m1_kp.value() < 0.0 || m1_ki.value() < 0.0 || m1_kd.value() < 0.0) {
                RCLCPP_FATAL(logger, "Motor 1 pid gains must be finite and non-negative!");
                return hardware_interface::CallbackReturn::ERROR;
            } 

            if (m1_qpps.value() == 0) {
                RCLCPP_FATAL(logger, "Motor 1 qpps cannot be zero!");
                return hardware_interface::CallbackReturn::ERROR;
            }
            VelocityPidSettings settings;
            settings.kp = m1_kp.value();
            settings.ki = m1_ki.value();
            settings.kd = m1_kd.value();
            settings.qpps = m1_qpps.value();
            m1_pid_settings = settings;
        }
        
        std::optional<VelocityPidSettings> m2_pid_settings = std::nullopt;
        if (m2_kp && m2_ki && m2_kd && m2_qpps) {
            if (m2_kp.value() < 0.0 || m2_ki.value() < 0.0 || m2_kd.value() < 0.0) {
                RCLCPP_FATAL(logger, "Motor 2 pid gains must be finite and non-negative!");
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (m2_qpps.value() == 0) {
                RCLCPP_FATAL(logger, "Motor 2 qpps cannot be zero!");
                return hardware_interface::CallbackReturn::ERROR;
            }

            VelocityPidSettings settings;
            settings.kp = m2_kp.value();
            settings.ki = m2_ki.value();
            settings.kd = m2_kd.value();
            settings.qpps = m2_qpps.value();
            m2_pid_settings = settings;
        }
        
        specs_.watchdog_timeout_sec = watchdog_timeout_sec;

        specs_.motion_strategy = motion_strategy;
        specs_.default_acceleration = default_acceleration;
        
        specs_.m1_max_current = m1_max_current;
        specs_.m2_max_current = m2_max_current;
        
        specs_.m1_pid_settings = m1_pid_settings;
        specs_.m2_pid_settings = m2_pid_settings;

        specs_.counts_per_rad = (gear_ratio * quad_counts_per_revolution) / (2.0 * M_PI);

        try {
            roboclaw_driver_ = std::make_unique<driver::RoboClaw>(
               device_path,
               baud_rate,
               address,
               std::chrono::milliseconds(read_timeout_ms),
               std::chrono::milliseconds(write_timeout_ms),
               exclusive_access,
               unacknowledged,
               max_attempts
            );
        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger, "Failed to construct RoboClaw driver object! what(): %s", ex.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger, "Successfully initialized hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboClawHardware::on_configure(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;

        auto logger = get_logger();

        RCLCPP_INFO(logger, "Configuring hardware...");

        if (RCUTILS_UNLIKELY(roboclaw_driver_ == nullptr)) {
           RCLCPP_FATAL(logger, "RoboClaw driver instance is null!");
           return hardware_interface::CallbackReturn::ERROR;
        }
            
        RCLCPP_INFO(logger, "Trying to open RoboClaw, device_path: %s", roboclaw_driver_->device_path().c_str());
        try {
            roboclaw_driver_->open();
        } catch (const std::system_error& ex) {
            RCLCPP_ERROR(
                logger, 
                "Failed to open RoboClaw, system error occurred! what(): message=%s code=%d, category=%s",
                ex.what(),
                ex.code().value(),
                ex.code().category().name()
            );
            return hardware_interface::CallbackReturn::ERROR;
        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger, "unexpected error occurred! what(): %s", ex.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(logger, "Successfully configured hardware");

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    

    hardware_interface::CallbackReturn RoboClawHardware::on_activate(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;
        auto logger = get_logger();

        RCLCPP_INFO(logger, "Activating hardware...");

        if (RCUTILS_UNLIKELY(roboclaw_driver_ == nullptr)) {
            RCLCPP_FATAL(logger, "RoboClaw driver instance is null!");
            return hardware_interface::CallbackReturn::ERROR;
        }
    
        try {
            RCLCPP_INFO(logger, "Checking communication with RoboClaw using command 21");
            std::string firmware_version = roboclaw_driver_->get_firmware_version();
            RCLCPP_INFO(logger, "RoboClaw communication established, firmware version: %s", firmware_version.c_str());

            RCLCPP_INFO(logger, "Trying to read encoder counts using command 78");
            auto [enc1_counts, enc2_counts] = roboclaw_driver_->get_encoder_counts();
            previous_encoder_counts_[0] = enc1_counts;
            previous_encoder_counts_[1] = enc2_counts;

            RCLCPP_INFO(logger, "Trying to set serial timeout using command 14");
            if (specs_.watchdog_timeout_sec.has_value()) {
                roboclaw_driver_->set_serial_timeout(specs_.watchdog_timeout_sec.value());
            } else {
                RCLCPP_INFO(logger, "Parameter is unset, skip sending command 14");
            }
            
            RCLCPP_INFO(logger, "Trying to set max current limit for M1 using command 133");
            if (specs_.m1_max_current.has_value()) {
                roboclaw_driver_->set_m1_max_current_limit(specs_.m1_max_current.value());
            } else {
                RCLCPP_INFO(logger, "Parameter is unset, skip sending command 133");
            }

            RCLCPP_INFO(logger, "Trying to set max current limit for M2 using command 134");
            if (specs_.m2_max_current.has_value()) {
                roboclaw_driver_->set_m2_max_current_limit(specs_.m2_max_current.value());
            } else {
                RCLCPP_INFO(logger, "Parameter is unset, skip sending command 134");
            }

            RCLCPP_INFO(logger, "Trying to set velocity PID gains and qpps for M1 using command 28");
            if (specs_.m1_pid_settings.has_value()) {
                double kp = specs_.m1_pid_settings->kp;
                double ki = specs_.m1_pid_settings->ki;
                double kd = specs_.m1_pid_settings->kd;
                std::uint32_t qpps = specs_.m1_pid_settings->qpps;
                roboclaw_driver_->set_velocity_pid_m1(kp, ki, kd, qpps);
            } else {
                RCLCPP_INFO(logger, "Parameters are incomplete or unset, skip sending command 28");
            }
        
            RCLCPP_INFO(logger, "Trying to set velocity PID gains and qpps for M2 using command 29");
            if (specs_.m2_pid_settings.has_value()) {
                double kp = specs_.m2_pid_settings->kp;
                double ki = specs_.m2_pid_settings->ki;
                double kd = specs_.m2_pid_settings->kd;
                std::uint32_t qpps = specs_.m2_pid_settings->qpps;
                roboclaw_driver_->set_velocity_pid_m2(kp, ki, kd, qpps);
            } else {
                RCLCPP_INFO(logger, "Parameters are incomplete or unset, skip sending command 29");
            }

            set_command("left_wheel_joint/velocity", 0.0);
            set_command("right_wheel_joint/velocity", 0.0);

        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger, "There was also a problem releted to communication! what(): %s", ex.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger, "Successfully activeted hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboClawHardware::on_deactivate(
        const rclcpp_lifecycle::State& previuos_state
    ) {
        (void)previuos_state;
        auto logger = get_logger();

        RCLCPP_INFO(logger, "Deactivating hardware...");

        if (RCUTILS_UNLIKELY(roboclaw_driver_ == nullptr)) {
            RCLCPP_FATAL(logger, "RoboClaw driver instance is null!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        try {
            set_command("left_wheel_joint/velocity", 0.0);
            set_command("right_wheel_joint/velocity", 0.0);

            RCLCPP_INFO(logger, "Trying to stop the motor (freewheeling)");
            roboclaw_driver_->drive_m1_m2_signed_duty_cycle(0, 0);
             
        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger, "There was a problem ralated to communication. what(): %s", ex.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger, "Successfully deactivated hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboClawHardware::on_cleanup(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;
        auto logger = get_logger();

        RCLCPP_INFO(logger, "Cleaning up hardware...");

        if (RCUTILS_UNLIKELY(roboclaw_driver_ == nullptr)) {
            RCLCPP_FATAL(logger, "RoboClaw driver instance is null!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        roboclaw_driver_->close();
        RCLCPP_INFO(logger, "Successfully cleaned up hardware");
        return hardware_interface::CallbackReturn::SUCCESS;    
    }

    hardware_interface::CallbackReturn RoboClawHardware::on_shutdown(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;
        auto logger = get_logger();

        RCLCPP_INFO(logger, "Shutting down hardware...");

        if (RCUTILS_UNLIKELY(roboclaw_driver_ != nullptr)) {
            RCLCPP_FATAL(logger, "RoboClaw driver instance is null!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        roboclaw_driver_->close();
    
        RCLCPP_INFO(logger, "Successfully shut down hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RoboClawHardware::on_error(
        const rclcpp_lifecycle::State& previous_state
    ) {
        auto logger = get_logger();

        if (RCUTILS_UNLIKELY(roboclaw_driver_ == nullptr)) {
            RCLCPP_FATAL(logger, "RoboClaw driver instance is null!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_ERROR(logger, "Error raised in %s, transitioning to Unconfigured state", previous_state.label().c_str());
        roboclaw_driver_->close();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RoboClawHardware::read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) {
        (void)time;
        (void)period;
        
        auto logger = get_logger();

        try {
            auto [left_encoder_counts, right_encoder_counts] = roboclaw_driver_->get_encoder_counts();
            
            std::int32_t delta_left_counts = left_encoder_counts - previous_encoder_counts_[0];
            std::int32_t delta_right_counts = right_encoder_counts - previous_encoder_counts_[1];

            previous_encoder_counts_[0] = left_encoder_counts;
            previous_encoder_counts_[1] = right_encoder_counts;

            position_[0] += static_cast<double>(delta_left_counts) / specs_.counts_per_rad;
            position_[1] += static_cast<double>(delta_right_counts) / specs_.counts_per_rad;

            auto [m1_ispeed, m2_ispeed] = roboclaw_driver_->get_instantaneous_speeds();
            std::int32_t left_ispeed = m1_ispeed;
            std::int32_t right_ispeed = m2_ispeed;
            
            double left_vel = static_cast<double>(left_ispeed) / specs_.counts_per_rad;
            double right_vel = static_cast<double>(right_ispeed) / specs_.counts_per_rad;

            set_state("left_wheel_joint/position", position_[0]);
            set_state("right_wheel_joint/position", position_[1]);
            set_state("left_wheel_joint/velocity", left_vel);
            set_state("right_wheel_joint/velocity", right_vel);

        } catch(const std::exception& ex) {
            RCLCPP_FATAL(logger, "There was a problem releted to communication! what(): %s", ex.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RoboClawHardware::write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) {
        (void)time;
        (void)period;

        auto logger = get_logger();

        try {
            double left_vel = get_command("left_wheel_joint/velocity");
            double right_vel = get_command("right_wheel_joint/velocity");

            write_velocity(left_vel, right_vel);
       
        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger, "There was a problem releted to communication! what(): %s", ex.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
    
} //namespace kait3_roboclaw_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    kait3_roboclaw_hardware::RoboClawHardware,
    hardware_interface::SystemInterface
)
