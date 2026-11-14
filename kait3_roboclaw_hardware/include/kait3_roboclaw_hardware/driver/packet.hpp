#ifndef KAIT3_ROBOCLAW_HARDWARE_DRIVER_PACKET_HPP_
#define KAIT3_ROBOCLAW_HARDWARE_DRIVER_PACKET_HPP_

#include "kait3_roboclaw_hardware/util/bit_conv.hpp"
#include <cstring>
#include <tuple>
#include <vector>
#include <stdexcept>

namespace kait3_roboclaw_hardware::driver {

    struct PacketBufferWriter {
    public:
        static constexpr std::size_t address_size_bytes = 1;
        static constexpr std::size_t command_size_bytes = 1;
        static constexpr std::size_t header_size_bytes  = address_size_bytes + command_size_bytes; 
    
    public:
        constexpr explicit PacketBufferWriter(std::vector<std::uint8_t>& buffer) noexcept
        : buffer_(buffer), num_written_(0), flushed_(false) {}

        PacketBufferWriter(const PacketBufferWriter&) = delete;
    
        PacketBufferWriter& operator=(const PacketBufferWriter&) = delete;

        PacketBufferWriter(PacketBufferWriter&&) noexcept = delete;

        PacketBufferWriter& operator=(PacketBufferWriter&&) noexcept = delete;

        ~PacketBufferWriter() noexcept = default;

    public:
        std::size_t num_written() const noexcept { return num_written_; }

        template<typename T>
        bool try_write(T value) noexcept {
            if (flushed_) 
                return false;
    
            if (sizeof(T) > buffer_.size() - num_written_) 
                return false;
    
            num_written_ += util::write_integer_big_endian<T>(current(), value);
            return true;
        }

        template<typename... Args>
        bool try_write_many(Args... args) noexcept {
            if (flushed_) 
                return false;

            constexpr std::size_t total_size_bytes = (sizeof(Args) + ... + 0);

            if (total_size_bytes > buffer_.size() - num_written_)
                return false;

            ((num_written_+= util::write_integer_big_endian(current(), args)), ...);

            return true;
        }

        bool try_write_bytes(const std::uint8_t *source, std::size_t offset, std::size_t count) noexcept {
            if (flushed_)
                return false;

            if (count > buffer_.size() - num_written_) 
                return false;
            
            if (count != 0)
                std::memcpy(current(), source + offset, count);
            num_written_ += count;

            return true;
        }

        bool try_write_header(std::uint8_t address, std::uint8_t command) noexcept {
            if (flushed_) 
                return false;

            if (header_size_bytes > buffer_.size() - num_written_)
                return false;

            buffer_[num_written_++] = address;
            buffer_[num_written_++] = command;

            return true;
        }

        template<typename T>
        void write(T value) {
            if (flushed_) 
                throw std::runtime_error("cannot write after flush");

            if (sizeof(T) > buffer_.size() - num_written_) 
                throw std::out_of_range("buffer overflow when writing integer");

            num_written_ += util::write_integer_big_endian<T>(current(), value);
        }

        template<typename... Args>
        void write_many(Args... args) {
            (write(args), ...);
        }

        void write_bytes(const std::uint8_t *source, std::size_t offset, std::size_t count) {
            if (flushed_)
                throw std::runtime_error("cannot write bytes after flush");

            if (count > buffer_.size() - num_written_)
                throw std::out_of_range("buffer overflow when writing bytes");

            if (count != 0)
                std::memcpy(current(), source + offset, count);

            num_written_ += count;
        }

        void write_header(std::uint8_t address, std::uint8_t command) {
            if (flushed_) 
                throw  std::runtime_error("cannot write header after flush");

            if (num_written_ > buffer_.size() || header_size_bytes > buffer_.size() - num_written_)
                throw std::out_of_range("buffer overflow when writing header");

            buffer_[num_written_++] = address;
            buffer_[num_written_++] = command;
        }

        bool try_flush() noexcept {
            if (flushed_) 
                return false;
    
            flushed_ = true;
            return true;
        }

        void flush() {
            if (flushed_) 
                throw std::runtime_error("cannot flush after flush");

            flushed_ = true;
        }

    private:
        std::uint8_t *current() noexcept { return buffer_.data() + num_written_; }

        std::vector<std::uint8_t>& buffer_;
        std::size_t num_written_;
        bool flushed_;
    };

    struct PacketBufferReader {
    public:
        constexpr PacketBufferReader(std::vector<std::uint8_t>& buffer) noexcept
        : buffer_(buffer), num_read_(0) {}

        PacketBufferReader(const PacketBufferReader&) = delete;

        PacketBufferReader& operator=(const PacketBufferReader&) = delete;

        PacketBufferReader(PacketBufferReader&&) noexcept = delete;

        PacketBufferReader& operator=(PacketBufferReader&&) noexcept = delete;
        
    public:
        std::size_t num_read() const noexcept { return num_read_; }

        template<typename T>
        bool try_read(T& value) noexcept {
            if (sizeof(T) > buffer_.size() - num_read_) 
                return false;

            num_read_ += util::read_integer_big_endian<T>(current(), value);
            
            return true;
        }

        template<typename... Args>
        bool try_read_many(Args&... args) noexcept {
            constexpr std::size_t total_size_bytes = (sizeof(Args) + ... + 0);
            if (total_size_bytes > buffer_.size() - num_read_) 
                return false;

            ((num_read_ += util::read_integer_big_endian(current(), args)), ...);
            return true;
        }
       
        bool try_read_bytes(std::uint8_t *destination, std::size_t offset, std::size_t count) noexcept {
            if (count > buffer_.size() - num_read_) 
                return false;

            if (count != 0) {
                std::memcpy(destination + offset, current(), count);
            }
            num_read_ += count;
            return true;
        }

        template<typename T>
        T read() {
            T value;
            if (!try_read(value)) 
                throw  std::out_of_range("buffer underflow when reading integer");

            return value;
        }

        template<typename... Args>
        std::tuple<Args...> read_many() {
            std::tuple<Args...> values{};
            
            const bool success = std::apply(
                [this](auto&... args) {
                    return try_read_many(args...);
                },
                values
            );

            if (!success)
                throw  std::out_of_range("buffer underflow when reading integers");

            return values;
        }

        void read_bytes(std::uint8_t *destination, std::size_t offset, std::size_t count) {
            if (!try_read_bytes(destination, offset, count))
                throw std::out_of_range("buffer underflow when reading bytes");
        }
            
    
    private:
        const std::uint8_t *current() const noexcept { return buffer_.data() + num_read_; }

        std::vector<std::uint8_t>& buffer_;
        std::size_t num_read_;
    };
} //namespace kait3_roboclaw_hardware::driver

#endif //KAIT3_ROBOCLAW_HARDWARE_DRIVER_PACKET_HPP_
