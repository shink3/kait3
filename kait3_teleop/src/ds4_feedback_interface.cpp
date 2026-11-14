#include "kait3_teleop/ds4_feedback_interface.hpp"
#include <cerrno>
#include <cmath>
#include <stdexcept>
#include <fcntl.h>
#include <system_error>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <zlib.h>

namespace kait3_teleop {

    namespace {

         inline std::uint32_t calculate_crc32(const std::uint8_t *data, std::size_t size) {
            constexpr std::uint8_t bt_header = 0xA2;

            ulong crc = crc32_z(0L, Z_NULL, 0);
            crc = crc32_z(crc, &bt_header, sizeof(bt_header));
            crc = crc32_z(crc, data, size);

            return static_cast<std::uint32_t>(crc);
         }

    }
    
    DualShock4FeedbackInterface::DualShock4FeedbackInterface(const std::string& device_path)
    : fd_(-1), device_path_(device_path), packet_buffer_(),
      red_(0), green_(0), blue_(255) {}

    DualShock4FeedbackInterface::~DualShock4FeedbackInterface() noexcept {
        try {
            set_rumble(0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            set_flush(0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            set_lightbar_color(0.0f, 0.0f, 1.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } catch (...) {
        }

        close();
    }

    void DualShock4FeedbackInterface::open() {
        if (fd_ != -1) 
            throw std::logic_error("device is already open");

        int fd = ::open(device_path_.c_str(), O_RDWR);
            
        if (fd == -1)
            throw std::system_error(errno, std::system_category(), "open"); 
        
        fd_ = fd;
    }

    bool DualShock4FeedbackInterface::is_open() const noexcept {
        return fd_ >= 0;
    }

    void DualShock4FeedbackInterface::close() noexcept {
        if (fd_ == -1) return;

        ::close(fd_);
        fd_ = -1;
    }

    void DualShock4FeedbackInterface::set_lightbar_color(float red, float green, float blue) {
        set_lightbar_color( 
            static_cast<std::uint8_t>(std::lround(red * 255.0f)),
            static_cast<std::uint8_t>(std::lround(green * 255.0f)),
            static_cast<std::uint8_t>(std::lround(blue * 255.0f))
        );
    }

    void DualShock4FeedbackInterface::set_lightbar_color(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
        send_output_report(FeedbackFlags::COLOR, red, green, blue, 0, 0, 0, 0);
    }

    void DualShock4FeedbackInterface::set_rumble(float weak, float strong) {
        set_rumble(
            static_cast<std::uint8_t>(std::lround(weak * 255.0)),
            static_cast<std::uint8_t>(std::lround(strong * 255.0))
        ); 
    }

    void DualShock4FeedbackInterface::set_flush(float led_bright_sec, float led_dark_sec) {
        set_flush(
            static_cast<std::uint8_t>(std::lround(led_bright_sec * 100.0f)),
            static_cast<std::uint8_t>(std::lround(led_dark_sec * 100.0f))
        );
    }

    void DualShock4FeedbackInterface::set_flush(std::uint8_t led_bright, std::uint8_t led_dark) {
        send_output_report(FeedbackFlags::LED, red_, green_, blue_, 0, 0, led_bright, led_dark);
    }

    void DualShock4FeedbackInterface::set_rumble(std::uint8_t weak, std::uint8_t strong) {
        send_output_report(FeedbackFlags::RUMBLE, 0, 0, 0, weak, strong, 0, 0);
    }

    void DualShock4FeedbackInterface::send_output_report(
            DualShock4FeedbackInterface::FeedbackFlags flags,
            std::uint8_t red, 
            std::uint8_t green, 
            std::uint8_t blue, 
            std::uint8_t rumble_weak, 
            std::uint8_t rumble_strong,
            std::uint8_t flush_led_bright,
            std::uint8_t flush_led_dark
        ) {
    	if (!is_open()) 
   		    throw std::logic_error("device is not open");
    
        packet_buffer_.fill(0);

        packet_buffer_[0]  = 0x11;          
        packet_buffer_[1]  = 0xC0;       
        packet_buffer_[3]  = 0xf0 | static_cast<std::uint8_t>(flags);          
        packet_buffer_[6]  = rumble_weak;
        packet_buffer_[7]  = rumble_strong;
        packet_buffer_[8]  = red;          
        packet_buffer_[9]  = green;        
        packet_buffer_[10] = blue;
        packet_buffer_[11] = flush_led_bright;
        packet_buffer_[12] = flush_led_dark;
        
        std::uint32_t crc = calculate_crc32(packet_buffer_.data(), packet_buffer_.size() - sizeof(uint32_t));

        packet_buffer_[74] = static_cast<std::uint8_t>(crc);
        packet_buffer_[75] = static_cast<std::uint8_t>(crc >> 8);
        packet_buffer_[76] = static_cast<std::uint8_t>(crc >> 16);
        packet_buffer_[77] = static_cast<std::uint8_t>(crc >> 24);

        ssize_t num_written;
        do {
            num_written = ::write(fd_, packet_buffer_.data(), packet_buffer_.size());
        } while (num_written < 0 && errno == EINTR); 
    
       if (num_written < 0) 
            throw std::system_error(errno, std::system_category(), "write");
       
        if (static_cast<std::size_t>(num_written) != packet_buffer_.size()) 
            throw std::runtime_error("incomplete DualShock4 output report write");

        if ((static_cast<std::uint8_t>(flags) & static_cast<std::uint8_t>(FeedbackFlags::COLOR)) != 0) {
            red_   = red;
            green_ = green;
            blue_  = blue;
        }
    }
} //namespace kait3_teleop
