#ifndef KAIT3_CONTROLLER_EMERGENCY_STOP_CONTROLLER_HPP_
#define KAIT3_CONTROLLER_EMERGENCY_STOP_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <chrono>

namespace kait3_controller {

    class EmergencyStopController : public controller_interface::ControllerInterface {
    private:
        using EmptyMessage = std_msgs::msg::Empty;
        using BoolMessage  = std_msgs::msg::Bool;

    private:
        struct EmergencyStopStatus {
            bool active;
            std::chrono::steady_clock::time_point received_time;
        };

    public:
        EmergencyStopController();

    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state
        ) override;


        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state
        ) override;
    
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        controller_interface::return_type update(
            const rclcpp::Time& time,
            const rclcpp::Duration& period
        ) override;

    private:
        bool is_valid_topic_name(const std::string& topic_name) const noexcept;

        void on_emergency_stop_message_received(BoolMessage::ConstSharedPtr message);

        void release_resources() noexcept;

    private:
        rclcpp::Subscription<BoolMessage>::SharedPtr e_stop_subscription_;
        realtime_tools::RealtimePublisher<EmptyMessage>::SharedPtr e_stop_activated_publisher_;
        realtime_tools::RealtimePublisher<EmptyMessage>::SharedPtr e_stop_deactivated_publisher_;
        realtime_tools::RealtimePublisher<EmptyMessage>::SharedPtr comm_timed_out_publisher_;
        realtime_tools::RealtimePublisher<EmptyMessage>::SharedPtr comm_recovered_publisher_;
        realtime_tools::RealtimeBuffer<EmergencyStopStatus> realtime_buffer_;
        std::chrono::steady_clock::duration comm_timeout_;
       
        bool enable_notifications_;
        bool e_stop_activated_notification_published_;
        bool comm_timed_out_notification_published_;
    };
    

} //namespace kait3_controller

#endif //KAIT3_CONTROLLER_EMERGENCY_STOP_CONTROLLER_HPP_
