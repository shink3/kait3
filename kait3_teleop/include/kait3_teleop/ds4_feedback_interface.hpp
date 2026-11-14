#ifndef KAIT3_TELEOP_DS4_FEEDBACK_INTERFACE_HPP_
#define KAIT3_TELEOP_DS4_FEEDBACK_INTERFACE_HPP_

#include <array>
#include <cstdint>
#include <string>

/**
 * References:
 *  - https://www.psdevwiki.com/ps4/DS4-BT#0x11_2
 *  - https://chromium.googlesource.com/chromium/src/+/HEAD/device/gamepad/dualshock4_controller.cc
 **/

namespace kait3_teleop {

    class DualShock4FeedbackInterface {
    private:

        enum class FeedbackFlags {
            NONE    = 0x00,
            RUMBLE  = 0x01,
            COLOR   = 0x02,
            FLUSH   = 0x04,
            UNKNOWN = 0x08,
            LED     = 0x06, // COLOR  | FLUSH 
            ALL     = 0x07, // RUMBLE | COLOR | FLUSH
        };

    public:
        DualShock4FeedbackInterface(const std::string& device_path);
        
        ~DualShock4FeedbackInterface() noexcept;
       
        void open();

        bool is_open() const noexcept;

        void close() noexcept;

        void set_lightbar_color(float red, float green, float blue);

        void set_lightbar_color(std::uint8_t red, std::uint8_t green, std::uint8_t blue);

        void set_rumble(float weak, float strong);

        void set_rumble(std::uint8_t weak, std::uint8_t strong);

        void set_flush(float led_bright_sec, float led_dark_sec);

        void set_flush(std::uint8_t led_bright, std::uint8_t led_dark);

    private:
        void send_output_report(
            FeedbackFlags flags,
            std::uint8_t red, 
            std::uint8_t green, 
            std::uint8_t blue, 
            std::uint8_t rumble_weak, 
            std::uint8_t rumble_strong,
            std::uint8_t flush_led_bright,
            std::uint8_t flush_led_dark
        ); 

    private:
        int fd_;
        std::string device_path_;
        std::array<std::uint8_t, 78> packet_buffer_;
        std::uint8_t red_;
        std::uint8_t green_;
        std::uint8_t blue_;
    };
} //kait3_teleop

#endif //KAIT3_TELEOP_DS4_FEEDBACK_INTERFACE_HPP_
