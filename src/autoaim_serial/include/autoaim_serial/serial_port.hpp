#ifndef AUTOAIM_SERIAL__SERIAL_PORT_HPP_
#define AUTOAIM_SERIAL__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace autoaim_serial {

// Thin POSIX termios wrapper. One instance owns one file descriptor; callers
// are responsible for serializing open/close/write between threads.
class SerialPort {
public:
    enum class ReadResult { kOk, kTimeout, kError, kClosed };

    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens and configures a raw 8N1 port. Returns false and fills `error` on
    // failure (unsupported baudrate, missing device, termios error).
    bool open(const std::string& device, int baudrate, std::string* error);
    void close();
    bool is_open() const;

    // Waits up to timeout_ms for bytes. kOk means bytes_read > 0.
    ReadResult read(
        std::uint8_t* buffer,
        std::size_t capacity,
        int timeout_ms,
        std::size_t* bytes_read,
        std::string* error
    );

    // Writes the whole buffer, spending at most timeout_ms in total.
    bool write_all(const std::uint8_t* data, std::size_t size, int timeout_ms, std::string* error);

private:
    int fd_ = -1;
};

} // namespace autoaim_serial

#endif // AUTOAIM_SERIAL__SERIAL_PORT_HPP_
