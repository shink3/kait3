#ifndef KAIT3_TELEOP_DS4_TELEOP_NODE_HPP_
#define KAIT3_TELEOP_DS4_TELEOP_NODE_HPP_

#include "kait3_teleop/ds4_feedback_interface.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>

namespace kait3_teleop {

    class DualShock4TeleopNode : public rclcpp::Node {
    private:
        using BoolMessage         = std_msgs::msg::Bool;
        using Float32Message      = std_msgs::msg::Float32;
        using ColorMessage        = std_msgs::msg::ColorRGBA;
        using TwistMessage        = geometry_msgs::msg::Twist;
        using TwistStampedMessage = geometry_msgs::msg::TwistStamped;
        using JoyMessage          = sensor_msgs::msg::Joy;

    public:
        DualShock4TeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
   
        ~DualShock4TeleopNode() noexcept ;

    private:
        void on_joy_message_received(JoyMessage::ConstSharedPtr message);

        void on_rumble_message_received(Float32Message::ConstSharedPtr message);

        void on_lightbar_color_message_received(ColorMessage::ConstSharedPtr message);
        
        void on_flush_message_received(BoolMessage::ConstSharedPtr message);

        void publish_twist(double linear_x, double angular_z);

        void set_scale(const std::string& name, double scale);

        float get_and_check_feedback_parameter(const std::string& parameter, double lower, double upper); 

        rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& parameters);

   
    private:
        bool initialized_;
        bool was_twist_published_;
        bool stamped_;
        std::string base_frame_id_;
        int slow_button_;
        int normal_button_;
        int turbo_button_;
        int axes_linear_;
        int axes_angular_;
        std::map<std::string, double> scales_;
        float rumble_weak_multiplier_;
        float rumble_strong_multiplier_;
        float flush_bright_cycle_;
        float flush_dark_cycle_;

        std::unique_ptr<DualShock4FeedbackInterface> feedback_interface_;
        rclcpp::Subscription<JoyMessage>::SharedPtr joy_subscription_;
        rclcpp::Subscription<Float32Message>::SharedPtr rumble_subscription_;
        rclcpp::Subscription<ColorMessage>::SharedPtr lightbar_color_subscription_;
        rclcpp::Subscription<BoolMessage>::SharedPtr flush_subscription_;
        rclcpp::executors::SingleThreadedExecutor feedback_executor_;
        std::thread feedback_thread_;
        rclcpp::Publisher<TwistMessage>::SharedPtr twist_publisher_;
        rclcpp::Publisher<TwistStampedMessage>::SharedPtr twist_stamped_publisher_;
        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };

} //namespace kait3_teleop

#endif //KAIT3_TELEOP_DS4_TELEOP_NODE_HPP_
