#include "kait3_controller/emergency_stop_controller.hpp"
#include <controller_interface/controller_interface_base.hpp>
#include <rcl/validate_topic_name.h>
#include <rcutils/macros.h>

namespace kait3_controller {

    EmergencyStopController::EmergencyStopController() {}

    controller_interface::InterfaceConfiguration EmergencyStopController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration configuration;

        configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        configuration.names.push_back("emergency_stop/active");

        return configuration;
     }

    controller_interface::InterfaceConfiguration EmergencyStopController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration configuration;
        
        configuration.type = controller_interface::interface_configuration_type::NONE;

        return configuration;
    }

    controller_interface::CallbackReturn EmergencyStopController::on_init() {
        auto logger = this->get_node()->get_logger();

        try {
            this->auto_declare<std::string>("emergency_stop_topic_name", "emergency_stop");
            this->auto_declare<double>("communication_timeout", 0.5);
            this->auto_declare<bool>("enable_notifications", true);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(logger, "Exception thrown during init stage. what(): %s", ex.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger, "Initialize successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }
    
    controller_interface::CallbackReturn EmergencyStopController::on_configure(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;

        auto node = this->get_node();
        auto logger = node->get_logger();

        try {
            auto topic_name = node->get_parameter("emergency_stop_topic_name").as_string(); 
            if (!is_valid_topic_name(topic_name)) {
                RCLCPP_ERROR(logger, "Parameter `emergency_stop_topic_name` is invalid");
                return controller_interface::CallbackReturn::ERROR;
            }

            double communication_timeout = node->get_parameter("communication_timeout").as_double();
            if (!std::isfinite(communication_timeout) || communication_timeout <= 0.0) {
                RCLCPP_ERROR(logger, "Parameter `communication_timeout` must be finite and greater than zero");
                return controller_interface::CallbackReturn::ERROR;
            }

            comm_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(communication_timeout)
            );

            e_stop_subscription_ = node->create_subscription<BoolMessage>(
                topic_name,
                rclcpp::QoS(1).best_effort(),
                std::bind(&EmergencyStopController::on_emergency_stop_message_received, this, std::placeholders::_1)
            );
            
            enable_notifications_ = node->get_parameter("enable_notifications").as_bool();
                
            if (enable_notifications_) {
                e_stop_activated_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<EmptyMessage>>(
                    node, 
                    "~/emergency_stop/activated", 
                    rclcpp::QoS(1).reliable()
                );

                e_stop_deactivated_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<EmptyMessage>>(
                    node, 
                    "~/emergency_stop/deactivated", 
                    rclcpp::QoS(1).reliable()
                );

                comm_timed_out_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<EmptyMessage>>(
                    node,
                    "~/communication/timed_out",
                    rclcpp::QoS(1).reliable()
                );

                comm_recovered_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<EmptyMessage>>(
                    node,
                    "~/communication/recovered",
                    rclcpp::QoS(1).reliable()
                );
            }
        } catch (const std::exception& ex) {
            release_resources();

            RCLCPP_ERROR(logger, "Exception thrown during configure stage. what(): %s", ex.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger, "Configured successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn EmergencyStopController::on_activate(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;

        auto logger = this->get_node()->get_logger();

        if (RCUTILS_UNLIKELY(!command_interfaces_[0].set_value(1.0))) {
            RCLCPP_ERROR(logger, "Failed to set command");
            return controller_interface::CallbackReturn::ERROR;
        }

        e_stop_activated_notification_published_ = false;
        comm_timed_out_notification_published_ = false;

        EmergencyStopStatus initial_status;
        initial_status.active = false;
        initial_status.received_time = std::chrono::steady_clock::time_point::min();

        realtime_buffer_.writeFromNonRT(initial_status);

        RCLCPP_INFO(logger, "Activate successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn EmergencyStopController::on_deactivate(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;
            
        RCLCPP_INFO(this->get_node()->get_logger(), "Deactivate successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }
    
    controller_interface::CallbackReturn EmergencyStopController::on_cleanup(
        const rclcpp_lifecycle::State& previous_state
    ) {
        (void)previous_state;
        release_resources();
        RCLCPP_INFO(this->get_node()->get_logger(), "Cleanup successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn EmergencyStopController::on_error(
        const rclcpp_lifecycle::State& previous_state
    ) {
        release_resources();

        RCLCPP_INFO(
            this->get_node()->get_logger(), 
            "Error raised in %s, transitioning to Unconfigured state.", 
            previous_state.label().c_str()
        );

        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::return_type EmergencyStopController::update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) {
       (void)time;
       (void)period;

        EmergencyStopStatus *status = realtime_buffer_.readFromRT();

        bool received = status->received_time != std::chrono::steady_clock::time_point::min();

        bool comm_timeout = received && (std::chrono::steady_clock::now() - status->received_time) >= comm_timeout_;

        bool comm_error = !received || comm_timeout;
    
        bool e_stop_active = comm_error || status->active;
   		
   		/**     
         * NOTE:
		 *
		 * If LoanedCommandInterface::set_value() returns false, the command value
		 * may not have been updated due to a failure to acquire an internal lock.
		 * 
         * In this case, the stop command may not be reflected in the command
		 * interface. Therefore, a hardware-side watchdog should trigger a safe
		 * state if command updates cease.
         *
		 * Since this system operates with hardware components in synchronous mode,
		 * lock contention within the real-time loop is not expected under normal
		 * operating conditions.
		 *
		 * Such a failure is therefore treated as an anomalous condition.
		 * The controller returns ERROR to notify the controller manager.
		*/
        if (RCUTILS_UNLIKELY(!command_interfaces_[0].set_value(e_stop_active ? 1.0 : 0.0))) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to set command");
            return controller_interface::return_type::ERROR;
        }
        
        if (!enable_notifications_ || !received) 
           return controller_interface::return_type::OK;

        if (!comm_timeout && status->active && !e_stop_activated_notification_published_) {
            if (e_stop_activated_publisher_->try_publish(EmptyMessage{})) { 
               e_stop_activated_notification_published_ = true;
            }
        } else if (!comm_timeout && !status->active && e_stop_activated_notification_published_) {
            if (e_stop_deactivated_publisher_->try_publish(EmptyMessage{})) {
                e_stop_activated_notification_published_ = false;
            }
        }

        if (comm_timeout && !comm_timed_out_notification_published_) {
            if (comm_timed_out_publisher_->try_publish(EmptyMessage{})) {
                comm_timed_out_notification_published_ = true;
            }
        } else if (!comm_timeout && comm_timed_out_notification_published_) {
            if (comm_recovered_publisher_->try_publish(EmptyMessage{})) {
                comm_timed_out_notification_published_ = false;
            }
        }
        
        return controller_interface::return_type::OK;
    }

    bool EmergencyStopController::is_valid_topic_name(const std::string& topic_name) const noexcept {
        int validation_result;
        std::size_t invalid_index = 0;

        rcl_ret_t ret = rcl_validate_topic_name(topic_name.c_str(), &validation_result, &invalid_index);

        return ret == RCL_RET_OK && validation_result == RCL_TOPIC_NAME_VALID;
    }

    void EmergencyStopController::on_emergency_stop_message_received(
        EmergencyStopController::BoolMessage::ConstSharedPtr message
    ) {
        EmergencyStopStatus status;
        status.active = message->data;
        status.received_time = std::chrono::steady_clock::now();

        realtime_buffer_.writeFromNonRT(status);
    }
    
    void EmergencyStopController::release_resources() noexcept {
        e_stop_subscription_.reset();
        e_stop_activated_publisher_.reset();
        e_stop_deactivated_publisher_.reset();
        comm_timed_out_publisher_.reset();
        comm_recovered_publisher_.reset();
    }
} //namespace kait3_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    kait3_controller::EmergencyStopController,
    controller_interface::ControllerInterface
)
