#ifndef KAIT3_ROBOCLAW_HARDWARE_DRIVER_COMMAND_HPP_
#define KAIT3_ROBOCLAW_HARDWARE_DRIVER_COMMAND_HPP_

#include <cstdint>

#define ROBOCLAW_CMD_GROUP_QUERY_FLAG   (0x1000)
#define ROBOCLAW_CMD_GROUP_CTL_FLAG     (0x2000)

#define ROBOCLAW_CMD_GROUP_MASK         (0xF000)
#define ROBOCLAW_CMD_VALUE_MASK         (0x00FF)

#define ROBOCLAW_CMD_GROUP_QUERY(x)  (ROBOCLAW_CMD_GROUP_QUERY_FLAG | (x))
#define ROBOCLAW_CMD_GROUP_CTL(x)    (ROBOCLAW_CMD_GROUP_CTL_FLAG   | (x))

namespace kait3_roboclaw_hardware::driver {

    enum class RoboClawCommand : std::uint16_t {
        drive_forward_m1                               = ROBOCLAW_CMD_GROUP_CTL(0),
        drive_backward_m1                              = ROBOCLAW_CMD_GROUP_CTL(1),
        set_main_voltage_min                           = ROBOCLAW_CMD_GROUP_CTL(2),
        set_main_voltage_max                           = ROBOCLAW_CMD_GROUP_CTL(3),
        drive_forward_m2                               = ROBOCLAW_CMD_GROUP_CTL(4),
        drive_backward_m2                              = ROBOCLAW_CMD_GROUP_CTL(5),
        drive_m1_7bit                                  = ROBOCLAW_CMD_GROUP_CTL(6),
        drive_m2_7bit                                  = ROBOCLAW_CMD_GROUP_CTL(7),
        drive_forward_mixed_mode                       = ROBOCLAW_CMD_GROUP_CTL(8),
        drive_backward_mixed_mode                      = ROBOCLAW_CMD_GROUP_CTL(9),
        turn_right_mixed_mode                          = ROBOCLAW_CMD_GROUP_CTL(10),
        turn_left_mixed_mode                           = ROBOCLAW_CMD_GROUP_CTL(11),
        drive_7bit                                     = ROBOCLAW_CMD_GROUP_CTL(12),
        turn_7bit                                      = ROBOCLAW_CMD_GROUP_CTL(13),

        set_serial_timeout                             = ROBOCLAW_CMD_GROUP_CTL(14),
        get_serial_timeout                             = ROBOCLAW_CMD_GROUP_QUERY(15),
        get_firmware_version                           = ROBOCLAW_CMD_GROUP_QUERY(21),
        get_main_battery_voltage                       = ROBOCLAW_CMD_GROUP_QUERY(24),
        get_logic_battery_voltage                      = ROBOCLAW_CMD_GROUP_QUERY(25),
        set_min_logic_battery_voltage_level            = ROBOCLAW_CMD_GROUP_CTL(26),
        set_max_logic_battery_voltage_level            = ROBOCLAW_CMD_GROUP_CTL(27),
        get_motor_pwms                                 = ROBOCLAW_CMD_GROUP_QUERY(48),
        get_motor_currents                             = ROBOCLAW_CMD_GROUP_QUERY(49),
        set_main_battery_voltages                      = ROBOCLAW_CMD_GROUP_CTL(57),
        set_logic_battery_voltages                     = ROBOCLAW_CMD_GROUP_CTL(58),
        get_main_battery_voltages                      = ROBOCLAW_CMD_GROUP_QUERY(59),
        get_logic_battery_voltages                     = ROBOCLAW_CMD_GROUP_QUERY(60),
        set_s3_s4_s5_modes                             = ROBOCLAW_CMD_GROUP_CTL(74),
        get_s3_s4_s5_modes                             = ROBOCLAW_CMD_GROUP_QUERY(75),
        set_deadband_rc_analog_controls                = ROBOCLAW_CMD_GROUP_CTL(76),
        get_deadband_rc_analog_controls                = ROBOCLAW_CMD_GROUP_QUERY(77),
        restore_defaults                               = ROBOCLAW_CMD_GROUP_CTL(80),
        get_default_duty_accels                        = ROBOCLAW_CMD_GROUP_QUERY(81),
        get_temperature                                = ROBOCLAW_CMD_GROUP_QUERY(82),
        get_temperature2                               = ROBOCLAW_CMD_GROUP_QUERY(83),
        get_status                                     = ROBOCLAW_CMD_GROUP_QUERY(90),
        get_encoder_modes                              = ROBOCLAW_CMD_GROUP_QUERY(91),
        set_m1_encoder_mode                            = ROBOCLAW_CMD_GROUP_CTL(92),
        set_m2_encoder_mode                            = ROBOCLAW_CMD_GROUP_CTL(93),
        write_settings_to_eeprom                       = ROBOCLAW_CMD_GROUP_CTL(94),
        read_settings_from_eeprom                      = ROBOCLAW_CMD_GROUP_QUERY(95),
        set_standard_config_settings                   = ROBOCLAW_CMD_GROUP_CTL(98),
        get_standard_config_settings                   = ROBOCLAW_CMD_GROUP_QUERY(99),
        set_m1_max_current_limit                       = ROBOCLAW_CMD_GROUP_CTL(133),
        set_m2_max_current_limit                       = ROBOCLAW_CMD_GROUP_CTL(134),
        get_m1_max_current_limit                       = ROBOCLAW_CMD_GROUP_QUERY(135),
        get_m2_max_current_limit                       = ROBOCLAW_CMD_GROUP_QUERY(136),
        set_pwm_mode                                   = ROBOCLAW_CMD_GROUP_CTL(148),
        get_pwm_mode                                   = ROBOCLAW_CMD_GROUP_QUERY(149),

        get_encoder_count_m1                           = ROBOCLAW_CMD_GROUP_QUERY(16),
        get_encoder_count_m2                           = ROBOCLAW_CMD_GROUP_QUERY(17),
        get_speed_m1                                   = ROBOCLAW_CMD_GROUP_QUERY(18),
        get_speed_m2                                   = ROBOCLAW_CMD_GROUP_QUERY(19),
        reset_encoders_counts                          = ROBOCLAW_CMD_GROUP_CTL(20),
        set_encoder1_count                             = ROBOCLAW_CMD_GROUP_CTL(22),
        set_encoder2_count                             = ROBOCLAW_CMD_GROUP_CTL(23),
        get_raw_speed_m1                               = ROBOCLAW_CMD_GROUP_QUERY(30),
        get_raw_speed_m2                               = ROBOCLAW_CMD_GROUP_QUERY(31),
        get_encoder_counts                             = ROBOCLAW_CMD_GROUP_QUERY(78),
        get_instantaneous_speeds                       = ROBOCLAW_CMD_GROUP_QUERY(79),
    
        set_velocity_pid_m1                            = ROBOCLAW_CMD_GROUP_CTL(28),
        set_velocity_pid_m2                            = ROBOCLAW_CMD_GROUP_CTL(29),
        drive_m1_signed_duty                           = ROBOCLAW_CMD_GROUP_CTL(32),
        drive_m2_signed_duty                           = ROBOCLAW_CMD_GROUP_CTL(33),
        drive_m1_m2_signed_duty                        = ROBOCLAW_CMD_GROUP_CTL(34),
        drive_m1_signed_speed                          = ROBOCLAW_CMD_GROUP_CTL(35),
        drive_m2_signed_speed                          = ROBOCLAW_CMD_GROUP_CTL(36),
        drive_m1_m2_signed_speed                       = ROBOCLAW_CMD_GROUP_CTL(37),
        drive_m1_signed_speed_accel                    = ROBOCLAW_CMD_GROUP_CTL(38),
        drive_m2_signed_speed_accel                    = ROBOCLAW_CMD_GROUP_CTL(39),
        drive_m1_m2_signed_speed_accel                 = ROBOCLAW_CMD_GROUP_CTL(40),
        drive_m1_m2_individual_signed_speed_accel      = ROBOCLAW_CMD_GROUP_CTL(50),
        drive_m1_signed_duty_accel                     = ROBOCLAW_CMD_GROUP_CTL(52),
        drive_m2_signed_duty_accel                     = ROBOCLAW_CMD_GROUP_CTL(53),
        drive_m1_m2_signed_duty_accel                  = ROBOCLAW_CMD_GROUP_CTL(54),
        get_velocity_pid_m1                            = ROBOCLAW_CMD_GROUP_QUERY(55),
        get_velocity_pid_m2                            = ROBOCLAW_CMD_GROUP_QUERY(56),
        set_default_duty_accel_m1                      = ROBOCLAW_CMD_GROUP_CTL(68),
        set_default_duty_accel_m2                      = ROBOCLAW_CMD_GROUP_CTL(69),
    };   

} //namespace kait3_roboclaw_hardware::driver
#endif //KAIT3_ROBOCLAW_HARDWARE_DRIVER_COMMAND_HPP_
