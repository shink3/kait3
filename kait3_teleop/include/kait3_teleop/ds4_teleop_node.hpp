#ifndef KAIT3_TELEOP_DS4_TELEOP_NODE_HPP_
#define KAIT3_TELEOP_DS4_TELEOP_NODE_HPP_

#include "kait3_teleop/ds4_feedback_interface.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <unordered_map>
#include <type_traits>
#include <memory>
#include <string>
#include <cmath>
#include <stdexcept>

namespace kait3_teleop {

    class DualShock4TeleopNode : public rclcpp::Node {
    private:
        using EmptyMessage        = std_msgs::msg::Empty;
        using BoolMessage         = std_msgs::msg::Bool;
        using Float32Message      = std_msgs::msg::Float32;
        using ColorMessage        = std_msgs::msg::ColorRGBA;
        using TwistMessage        = geometry_msgs::msg::Twist;
        using TwistStampedMessage = geometry_msgs::msg::TwistStamped;
        using JoyMessage          = sensor_msgs::msg::Joy;

    private:
        static constexpr int num_buttons = 16;
        static constexpr int num_axes    = 6;

    private:
        struct ButtonEvent {
            int button;
            bool was_pressed;
            rclcpp::Publisher<EmptyMessage>::SharedPtr publisher;
        };

    public:
        DualShock4TeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
   
        ~DualShock4TeleopNode() noexcept ;

    private:
        void on_joy_message_received(JoyMessage::ConstSharedPtr message);

        void on_rumble_message_received(Float32Message::ConstSharedPtr message);

        void on_lightbar_color_message_received(ColorMessage::ConstSharedPtr message);
        
        void on_flash_message_received(BoolMessage::ConstSharedPtr message);

        void process_twist(JoyMessage::ConstSharedPtr message);

        void publish_twist(double linear_x, double angular_z);

        void process_button_events(JoyMessage::ConstSharedPtr message);

        void set_scale(const std::string& name, double scale);

        void validate_event_name(const std::string& event_name); 

        template<typename T>
        T get_parameter_in_range(const std::string& parameter, T lower, T upper);

        rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& parameters);
 
    private:
        std::string base_frame_id_;

        int slow_button_;
        int normal_button_;
        int turbo_button_;
        int axes_linear_;
        int axes_angular_;
        std::unordered_map<std::string, double> scales_;
        
        std::unique_ptr<DualShock4FeedbackInterface> feedback_interface_;
        rclcpp::Subscription<JoyMessage>::SharedPtr joy_subscription_;
        rclcpp::Subscription<Float32Message>::SharedPtr rumble_subscription_;
        rclcpp::Subscription<ColorMessage>::SharedPtr lightbar_color_subscription_;
        rclcpp::Subscription<BoolMessage>::SharedPtr flash_subscription_;
        rclcpp::executors::SingleThreadedExecutor feedback_executor_;
        std::thread feedback_thread_;
        float rumble_weak_multiplier_;
        float rumble_strong_multiplier_;
        float flash_bright_cycle_;
        float flash_dark_cycle_;
        
        bool stamped_;
        bool was_twist_published_;
        rclcpp::Publisher<TwistMessage>::SharedPtr twist_publisher_;
        rclcpp::Publisher<TwistStampedMessage>::SharedPtr twist_stamped_publisher_;
       
        std::vector<ButtonEvent> button_events_;

        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };

    template<typename T>
    T DualShock4TeleopNode::get_parameter_in_range(const std::string& parameter, T lower, T upper) {
        static_assert(
            (std::is_integral_v<T> && !std::is_same_v<T, bool>) || std::is_floating_point_v<T>, 
            "T must be integer or floating-point type"
        );
        
        using parameter_type = std::conditional_t<std::is_integral_v<T>, std::int64_t, double>;

        const parameter_type value = this->get_parameter(parameter).get_value<parameter_type>();

        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(value)) 
                throw std::invalid_argument("Parameter `" + parameter + "` must be finite");
        }

        if (value < lower || upper < value) {
            throw std::invalid_argument("Parameter `" + parameter + "` must be in the range of " + 
                std::to_string(lower) + " to " + std::to_string(upper) + ", but got " + std::to_string(value));
        }

        return static_cast<T>(value);
    }

} //namespace kait3_teleop

#endif //KAIT3_TELEOP_DS4_TELEOP_NODE_HPP_
