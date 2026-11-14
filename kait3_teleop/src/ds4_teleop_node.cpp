#include "kait3_teleop/ds4_teleop_node.hpp"
#include <unordered_set>

namespace kait3_teleop {

    DualShock4TeleopNode::DualShock4TeleopNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ds4_teleop_node", options), initialized_(false), was_twist_published_(false) {
        this->declare_parameter<std::string>("device_path", "/dev/dualshock4");
        this->declare_parameter<bool>("stamped", false);
        this->declare_parameter<std::string>("base_frame_id", "base_link");

        this->declare_parameter<int>("slow_button", -1);
        this->declare_parameter<int>("normal_button", 9);
        this->declare_parameter<int>("turbo_button", 10);
        this->declare_parameter<int>("axes_linear", 1);
        this->declare_parameter<int>("axes_angular", 2);

        this->declare_parameter<double>("scale_linear_slow", 0.0);
        this->declare_parameter<double>("scale_angular_slow", 0.0);
        this->declare_parameter<double>("scale_linear", 0.5);
        this->declare_parameter<double>("scale_angular", 1.0);
        this->declare_parameter<double>("scale_linear_turbo", 1.0);
        this->declare_parameter<double>("scale_angular_turbo", 2.0);

        this->declare_parameter<bool>("feedback.enabled", true);
        this->declare_parameter<double>("feedback.rumble_weak_multiplier", 1.0);
        this->declare_parameter<double>("feedback.rumble_strong_multiplier", 1.0);
        this->declare_parameter<double>("feedback.flush_bright_cycle", 0.5);
        this->declare_parameter<double>("feedback.flush_dark_cycle", 0.5);

        try {
            stamped_ = this->get_parameter("stamped").as_bool();
            base_frame_id_ = this->get_parameter("base_frame_id").as_string();
            
            slow_button_ = this->get_parameter("slow_button").as_int();
            normal_button_ = this->get_parameter("normal_button").as_int();
            turbo_button_ = this->get_parameter("turbo_button").as_int();
            axes_linear_ = this->get_parameter("axes_linear").as_int();
            axes_angular_ = this->get_parameter("axes_angular").as_int();
            
            set_scale("scale_linear_slow", this->get_parameter("scale_linear_slow").as_double());
            set_scale("scale_angular_slow", this->get_parameter("scale_angular_slow").as_double());
            set_scale("scale_linear", this->get_parameter("scale_linear").as_double());
            set_scale("scale_angular", this->get_parameter("scale_angular").as_double());
            set_scale("scale_linear_turbo", this->get_parameter("scale_linear_turbo").as_double());
            set_scale("scale_angular_turbo", this->get_parameter("scale_angular_turbo").as_double());
            

            if (this->get_parameter("feedback.enabled").as_bool()) {

                rumble_weak_multiplier_ = get_and_check_feedback_parameter("feedback.rumble_weak_multiplier",0.0, 1.0);        
                rumble_strong_multiplier_ = get_and_check_feedback_parameter("feedback.rumble_strong_multiplier", 0.0, 1.0);

                flush_bright_cycle_ = get_and_check_feedback_parameter("feedback.flush_bright_cycle", 0.0, 2.55);
                flush_dark_cycle_ = get_and_check_feedback_parameter("feedback.flush_dark_cycle", 0.0, 2.55);

                feedback_interface_ = std::make_unique<DualShock4FeedbackInterface>(
                    this->get_parameter("device_path").as_string()
                );

                feedback_interface_->open();

                rclcpp::CallbackGroup::SharedPtr callback_group = this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive,
                    false
                );

                rclcpp::SubscriptionOptions subscription_options;
                subscription_options.callback_group = callback_group;

                rumble_subscription_ = this->create_subscription<Float32Message>(
                    "dualshock4/rumble_intensity",
                    rclcpp::QoS(1).reliable(),
                    std::bind(&DualShock4TeleopNode::on_rumble_message_received, this, std:: placeholders::_1),
                    subscription_options
                );

                lightbar_color_subscription_ = this->create_subscription<ColorMessage>(
                    "dualshock4/lightbar_color",
                    rclcpp::QoS(1).reliable(),
                    std::bind(&DualShock4TeleopNode::on_lightbar_color_message_received, this, std::placeholders::_1),
                    subscription_options
                );

                flush_subscription_ = this->create_subscription<BoolMessage>(
                    "dualshock4/enable_flush",
                    rclcpp::QoS(1).reliable(),
                    std::bind(&DualShock4TeleopNode::on_flush_message_received, this, std::placeholders::_1),
                    subscription_options
                );
                
                feedback_executor_.add_callback_group(callback_group, this->get_node_base_interface());

                feedback_thread_ = std::thread([this](){
                    feedback_executor_.spin();     
                });

            }

            if (stamped_) {
                twist_stamped_publisher_ = this->create_publisher<TwistStampedMessage>(
                    "cmd_vel",
                    10
                );
            } else {
                twist_publisher_ = this->create_publisher<TwistMessage>(
                    "cmd_vel",
                    10
                );
            }

            joy_subscription_ = this->create_subscription<JoyMessage>(
                "joy",
                10,
                std::bind(&DualShock4TeleopNode::on_joy_message_received, this, std::placeholders::_1)
            );

            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&DualShock4TeleopNode::on_parameter_change, this, std::placeholders::_1)
            );

        } catch (const std::exception& ex) {
            RCLCPP_FATAL(this->get_logger(), "%s", ex.what());
            throw;
        }

        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "ds4_teleop_node initialized");
    }

    DualShock4TeleopNode::~DualShock4TeleopNode() noexcept {
        try {
            feedback_executor_.cancel();

            if (feedback_thread_.joinable())
                feedback_thread_.join(); 
        } catch (...) {
        }
    }

    void DualShock4TeleopNode::on_joy_message_received(DualShock4TeleopNode::JoyMessage::ConstSharedPtr message) {
        

        if ((slow_button_ >= 0 && static_cast<std::size_t>(slow_button_) >= message->buttons.size()) ||
            (normal_button_ >= 0 && static_cast<std::size_t>(normal_button_) >= message->buttons.size()) ||
            (turbo_button_ >= 0 && static_cast<std::size_t>(turbo_button_) >= message->buttons.size()) ||
            (axes_linear_ >= 0 && static_cast<std::size_t>(axes_linear_) >= message->axes.size()) ||
            (axes_angular_ >= 0 && static_cast<std::size_t>(axes_angular_) >= message->axes.size())
        ) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Joy message size smaller than configured indexes"
            );
            return;
        }

        bool slow_button_pressed   = (slow_button_ >= 0 && message->buttons[slow_button_] == 1);
        bool normal_button_pressed = (normal_button_ >= 0 && message->buttons[normal_button_] == 1);
        bool turbo_button_pressed  = (turbo_button_ >= 0 && message->buttons[turbo_button_] == 1);

        std::size_t pressed_count = static_cast<std::size_t>(slow_button_pressed) +
                                    static_cast<std::size_t>(normal_button_pressed) +
                                    static_cast<std::size_t>(turbo_button_pressed);

        std::string scale_linear_key;
        std::string scale_angular_key;

        if (pressed_count == 0) {    
            if (was_twist_published_) {
                publish_twist(0.0, 0.0);
                was_twist_published_ = false;
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5 * 1000,
                "No enable button pressed, so the twist will not be published"
            );
            return;

        } else if (pressed_count == 1) {
           if (slow_button_pressed) {
                scale_linear_key = "scale_linear_slow";
                scale_angular_key = "scale_angular_slow";
           } else if (normal_button_pressed) {
                scale_linear_key = "scale_linear";
                scale_angular_key = "scale_angular";
           } else {
                scale_linear_key = "scale_linear_turbo";
                scale_angular_key = "scale_angular_turbo";
           }

        } else {
            if (was_twist_published_) {
                publish_twist(0.0, 0.0);
                was_twist_published_ = false;
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "Multiple enable buttons pressed simulataneously, ignoring input"
            );

            return;
        }

        double scale_linear = scales_[scale_linear_key];
        double scale_angular = scales_[scale_angular_key];

        double linear_x = (axes_linear_ >= 0) 
            ? message->axes[axes_linear_] * scale_linear
            : 0.0;
        double angular_z = (axes_angular_ >= 0)
            ? message->axes[axes_angular_] * scale_angular
            : 0.0;

        publish_twist(linear_x, angular_z);
        was_twist_published_ = true;
    }

    void DualShock4TeleopNode::on_rumble_message_received(DualShock4TeleopNode::Float32Message::ConstSharedPtr message) {
        float value = message->data;
        if (!std::isfinite(value) || (value < 0.0f || 1.0f < value)) {
            RCLCPP_ERROR(
                this->get_logger(), 
                "Invalid rumble intensity received. intensity must be in range of 0.0 to 1.0, but got %f",
                value
            );
            return;
        }
        try {
            float weak = value * rumble_weak_multiplier_;
            float strong = value * rumble_strong_multiplier_;

            feedback_interface_->set_rumble(weak, strong);

        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set rumble. what(): %s", ex.what());
        }
    }

    void DualShock4TeleopNode::on_lightbar_color_message_received(DualShock4TeleopNode::ColorMessage::ConstSharedPtr message) {
        float red = message->r;
        float green = message->g;
        float blue = message->b;

        if (!std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue) ||
            (red < 0.0f || 1.0f < red) ||
            (green < 0.0f || 1.0f < green) ||
            (blue < 0.0f || 1.0f < blue)
        ) {
            RCLCPP_ERROR(
                this->get_logger(), 
                "Invalid lightbar color received. rgb value must be in the range of 0.0 to 1.0, but got (%f, %f, %f)",
                red, green, blue
            );
            return;
        }

        try {
            feedback_interface_->set_lightbar_color(red, green, blue);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set lightbar color. what(): %s", ex.what());
        }
    }

    void DualShock4TeleopNode::on_flush_message_received(DualShock4TeleopNode::BoolMessage::ConstSharedPtr message) {
        try {
            const float gate = static_cast<float>(message->data);
            float bright_cycle = gate * flush_bright_cycle_;
            float dark_cycle = gate * flush_dark_cycle_;

            feedback_interface_->set_flush(bright_cycle, dark_cycle);

        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set flush. what(): %s", ex.what());
        }
    }

    void DualShock4TeleopNode::publish_twist(double linear_x, double angular_z) {
        TwistMessage twist_message;
        twist_message.linear.x = linear_x;
        twist_message.linear.y = 0.0;
        twist_message.linear.z = 0.0;
        twist_message.angular.x = 0.0;
        twist_message.angular.y = 0.0;
        twist_message.angular.z = angular_z;

        if (stamped_) {
            TwistStampedMessage twist_stamped_message;
            twist_stamped_message.header.stamp = this->now();
            twist_stamped_message.header.frame_id = base_frame_id_;
            twist_stamped_message.twist = twist_message;
        
            twist_stamped_publisher_->publish(twist_stamped_message);
        } else {
            twist_publisher_->publish(twist_message);
        }
    }

    void DualShock4TeleopNode::set_scale(const std::string& name, double scale) {
        if (!std::isfinite(scale) || scale < 0.0) 
            throw std::invalid_argument("Parameter `" + name + "` must be finite and non-negative");

        scales_[name] = scale;
    }

    float DualShock4TeleopNode::get_and_check_feedback_parameter(const std::string& parameter, double lower, double upper) {
        double value = this->get_parameter(parameter).as_double();

        if (!std::isfinite(value) | (value < lower || upper < value)) 
            throw std::invalid_argument("Parameter `" + parameter + "` must be in the range of " + 
                std::to_string(lower) + " to " + std::to_string(upper)
            );

        return static_cast<float>(value);
    }

    rcl_interfaces::msg::SetParametersResult DualShock4TeleopNode::on_parameter_change(
        const std::vector<rclcpp::Parameter>& parameters
    ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        const std::unordered_set<std::string> tunable_parameters = {
            "scale_linear_slow",  "scale_angular_slow",
            "scale_linear",       "scale_angular",
            "scale_linear_turbo", "scale_angular_turbo"
        };

        for (const auto& parameter : parameters) {
            const auto& name = parameter.get_name();
            
            if (tunable_parameters.count(name) == 1) {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    result.successful = false;
                    result.reason = "Parameter `" + name + "` must be a double";
                    return result;
                }

                double scale = parameter.as_double();
                if (scale < 0.0) {
                    result.successful = false;
                    result.reason = "Parameter `" + name + "` must be positive";
                    return result;
                }
                scales_[name] = scale;
            } else {
                if (initialized_) {
                    result.successful = false;
                    result.reason = "Parameter `" + name + "` cannot be changed dynamically";
                    return result;
                }
            }
        }
        return result;
    }

} //namespace kait3_teleop
