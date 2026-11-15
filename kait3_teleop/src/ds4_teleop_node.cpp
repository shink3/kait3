#include "kait3_teleop/ds4_teleop_node.hpp"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <limits>

namespace kait3_teleop {

    DualShock4TeleopNode::DualShock4TeleopNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ds4_teleop_node", options), was_twist_published_(false) {
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

        this->declare_parameter<bool>("button_events.enabled", false);

        try {
            stamped_ = this->get_parameter("stamped").as_bool();
            base_frame_id_ = this->get_parameter("base_frame_id").as_string();
            
            slow_button_ = get_parameter_in_range<int>("slow_button", std::numeric_limits<int>::min(), num_buttons - 1);
            normal_button_ = get_parameter_in_range<int>("normal_button", std::numeric_limits<int>::min(), num_buttons - 1);
            turbo_button_ = get_parameter_in_range<int>("turbo_button", std::numeric_limits<int>::min(), num_buttons - 1);

            axes_linear_ = get_parameter_in_range<int>("axes_linear", 0, num_axes - 1);
            axes_angular_ = get_parameter_in_range<int>("axes_angular", 0, num_axes - 1);
            
            set_scale("scale_linear_slow", this->get_parameter("scale_linear_slow").as_double());
            set_scale("scale_angular_slow", this->get_parameter("scale_angular_slow").as_double());
            set_scale("scale_linear", this->get_parameter("scale_linear").as_double());
            set_scale("scale_angular", this->get_parameter("scale_angular").as_double());
            set_scale("scale_linear_turbo", this->get_parameter("scale_linear_turbo").as_double());
            set_scale("scale_angular_turbo", this->get_parameter("scale_angular_turbo").as_double());
            
            if (this->get_parameter("feedback.enabled").as_bool()) {
                this->declare_parameter<std::string>("feedback.device_path", "/dev/dualshock4");
                this->declare_parameter<double>("feedback.rumble_weak_multiplier", 1.0);
                this->declare_parameter<double>("feedback.rumble_strong_multiplier", 1.0);
                this->declare_parameter<double>("feedback.flash_bright_duration", 0.5);
                this->declare_parameter<double>("feedback.flash_dark_duration", 0.5);

                rumble_weak_multiplier_ = static_cast<float>(
                    get_parameter_in_range<double>("feedback.rumble_weak_multiplier",0.0, 1.0)
                );

                rumble_strong_multiplier_ = static_cast<float>(
                    get_parameter_in_range<double>("feedback.rumble_strong_multiplier", 0.0, 1.0)
                );

                flash_bright_cycle_ = static_cast<float>(
                    get_parameter_in_range<double>("feedback.flash_bright_duration", 0.0, 2.55)
                );

                flash_dark_cycle_ = static_cast<float>(
                    get_parameter_in_range<double>("feedback.flash_dark_duration", 0.0, 2.55)
                );

                feedback_interface_ = std::make_unique<DualShock4FeedbackInterface>(
                    this->get_parameter("feedback.device_path").as_string()
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

                flash_subscription_ = this->create_subscription<BoolMessage>(
                    "dualshock4/enable_flash",
                    rclcpp::QoS(1).reliable(),
                    std::bind(&DualShock4TeleopNode::on_flash_message_received, this, std::placeholders::_1),
                    subscription_options
                );
                
                feedback_executor_.add_callback_group(callback_group, this->get_node_base_interface());

                feedback_thread_ = std::thread([this](){
                    feedback_executor_.spin();     
                });

            }

            if (this->get_parameter("button_events.enabled").as_bool()) {
                this->declare_parameter<std::vector<std::string>>("button_events.event_names");

                const auto event_names = this->get_parameter("button_events.event_names").as_string_array();

                button_events_.reserve(event_names.size());

                for (const auto& event_name : event_names) {
                    validate_event_name(event_name);

                    const std::string param_name = "button_events." + event_name + ".button";
                    this->declare_parameter<int>(param_name);

                    const int button = get_parameter_in_range(param_name, 0, num_buttons - 1);

                    auto publisher = this->create_publisher<EmptyMessage>(
                        "button_events/" + event_name,
                        rclcpp::QoS(1).reliable()
                    );

                    button_events_.emplace_back(ButtonEvent { button, false, std::move(publisher) });
                }
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

        assert(message->buttons.size() == static_cast<std::size_t>(num_buttons));
        assert(message->axes.size() == static_cast<std::size_t>(num_axes));
        
        process_twist(message);
        process_button_events(message);
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

    void DualShock4TeleopNode::on_flash_message_received(DualShock4TeleopNode::BoolMessage::ConstSharedPtr message) {
        try {
            const float gate = static_cast<float>(message->data);
            float bright_cycle = gate * flash_bright_cycle_;
            float dark_cycle = gate * flash_dark_cycle_;

            feedback_interface_->set_flash(bright_cycle, dark_cycle);

        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set flush. what(): %s", ex.what());
        }
    }

    void DualShock4TeleopNode::process_twist(DualShock4TeleopNode::JoyMessage::ConstSharedPtr message) {
        
        bool slow_button_pressed   = (slow_button_ >= 0 && message->buttons[slow_button_] == 1);
        bool normal_button_pressed = (normal_button_ >= 0 && message->buttons[normal_button_] == 1);
        bool turbo_button_pressed  = (turbo_button_ >= 0 && message->buttons[turbo_button_] == 1);

        std::size_t pressed_count = static_cast<std::size_t>(slow_button_pressed) +
                                    static_cast<std::size_t>(normal_button_pressed) +
                                    static_cast<std::size_t>(turbo_button_pressed);

        if (pressed_count == 0) {    
            if (was_twist_published_) {
                publish_twist(0.0, 0.0);
                was_twist_published_ = false;
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "No enable button pressed, so the twist will not be published"
            );
        } else  if (pressed_count > 1) {
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
        } else {
            double scale_linear, scale_angular;
        
            if (slow_button_pressed) {
                scale_linear  = scales_["scale_linear_slow"];
                scale_angular = scales_["scale_angular_slow"];
            } else if (normal_button_pressed) {
                scale_linear  = scales_["scale_linear"];
                scale_angular = scales_["scale_angular"];
            } else {
                scale_linear  = scales_["scale_linear_turbo"];
                scale_angular = scales_["scale_angular_turbo"];
            }

            double linear_x  = message->axes[axes_linear_] * scale_linear;
            double angular_z = message->axes[axes_angular_] * scale_angular;
                

            publish_twist(linear_x, angular_z);
            was_twist_published_ = true;
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

    void DualShock4TeleopNode::process_button_events(DualShock4TeleopNode::JoyMessage::ConstSharedPtr message) {
        for (auto& event : button_events_) {
            const bool is_pressed = (message->buttons[event.button] == 1);
            const bool was_pressed = event.was_pressed;

            if (!was_pressed && is_pressed) {
                event.publisher->publish(EmptyMessage{});
            }

            event.was_pressed = is_pressed;
        }
    }

    void DualShock4TeleopNode::set_scale(const std::string& name, double scale) {
        if (!std::isfinite(scale) || scale < 0.0) 
            throw std::invalid_argument("Parameter `" + name + "` must be finite and non-negative");

        scales_[name] = scale;
    }

    void DualShock4TeleopNode::validate_event_name(const std::string& event_name) {
        if (event_name.empty()) {
            throw std::invalid_argument(
                "event name in parameter `button_events.event_names` cannot be empty"
            );
        }
        
        const unsigned char first = static_cast<unsigned char>(event_name.front());
        
        if (!std::isalpha(first) && first != '_') {
            throw std::invalid_argument(
                "event name '" + event_name + "' in parameter `button_events.event_names` must start with a letter or _"
            );
        }

        const bool is_valid_chars = std::all_of(event_name.begin(), event_name.end(), [](unsigned char c) {
            return std::isalnum(c) || c == '_';
        });

        if (!is_valid_chars) {
            throw std::invalid_argument(
                "event name '" + event_name + "' in parameter `button_events.event_names` must contain only alphanumeric characters or _"
            );
        }

    }

    rcl_interfaces::msg::SetParametersResult DualShock4TeleopNode::on_parameter_change(
        const std::vector<rclcpp::Parameter>& parameters
    ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        static const std::unordered_set<std::string> tunable_parameters = {
            "scale_linear_slow",  "scale_angular_slow",
            "scale_linear",       "scale_angular",
            "scale_linear_turbo", "scale_angular_turbo"
        };

        for (const auto& parameter : parameters) {
            const auto& name = parameter.get_name();
            const auto& type = parameter.get_type();
            
            if (tunable_parameters.count(name) == 0) {
                result.successful = false;
                result.reason = "Parameter `" + name + "` cannot be changed dynamically";
                return result;
            }

            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason = "Parameter `" + name + "` must be double";
                return result;
            }

            const double scale = parameter.as_double();
            if (!std::isfinite(scale) || scale < 0.0) {
                result.successful = false;
                result.reason = "Parameter `" + name + "` must be finite and non-negative";
                return result;
            }

            scales_[name] = scale;
        }     
        return result;
    }

} //namespace kait3_teleop
