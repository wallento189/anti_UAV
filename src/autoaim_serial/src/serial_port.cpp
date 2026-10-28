#include "autoaim_serial/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>

namespace autoaim_serial {

namespace {
    struct BaudRateEntry {
        int baudrate;
        speed_t speed;
    };

    // Fixed table instead of Bxxxx arithmetic: an unsupported value must fail
    // loudly at startup rather than silently falling back to another rate.
    constexpr BaudRateEntry kBaudRates[] = {
        {9600, B9600},
        {19200, B19200},
        {38400, B38400},
        {57600, B57600},
        {115200, B115200},
        {230400, B230400},
        {460800, B460800},
        {921600, B921600},
    };

    bool lookup_speed(int baudrate, speed_t* speed) {
        for (const BaudRateEntry& entry: kBaudRates) {
            if (entry.baudrate == baudrate) {
                *speed = entry.speed;
                return true;
            }
        }
        return false;
    }

    std::string errno_message(const std::string& action) {
        return action + ": " + std::strerror(errno);
    }
} // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& device, int baudrate, std::string* error) {
    close();

    speed_t speed = B0;
    if (!lookup_speed(baudrate, &speed)) {
        if (error != nullptr) {
            *error = "unsupported baudrate " + std::to_string(baudrate)
                + " (supported: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)";
        }
        return false;
    }

    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (error != nullptr) {
            *error = errno_message("open " + device);
        }
        return false;
    }

    termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        if (error != nullptr) {
            *error = errno_message("tcgetattr " + device);
        }
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
        if (error != nullptr) {
            *error = errno_message("set speed on " + device);
        }
        ::close(fd);
        return false;
    }
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        if (error != nullptr) {
            *error = errno_message("tcsetattr " + device);
        }
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::is_open() const {
    return fd_ >= 0;
}

SerialPort::ReadResult SerialPort::read(
    std::uint8_t* buffer,
    std::size_t capacity,
    int timeout_ms,
    std::size_t* bytes_read,
    std::string* error
) {
    if (bytes_read != nullptr) {
        *bytes_read = 0;
    }
    if (fd_ < 0) {
        if (error != nullptr) {
            *error = "serial port is not open";
        }
        return ReadResult::kError;
    }

    pollfd pfd {};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, timeout_ms);
    if (ready == 0) {
        return ReadResult::kTimeout;
    }
    if (ready < 0) {
        if (errno == EINTR) {
            return ReadResult::kTimeout;
        }
        if (error != nullptr) {
            *error = errno_message("poll");
        }
        return ReadResult::kError;
    }
    if ((pfd.revents & (POLLERR | POLLNVAL)) != 0) {
        if (error != nullptr) {
            *error = "poll reported a broken serial port";
        }
        return ReadResult::kError;
    }

    const ssize_t received = ::read(fd_, buffer, capacity);
    if (received > 0) {
        if (bytes_read != nullptr) {
            *bytes_read = static_cast<std::size_t>(received);
        }
        return ReadResult::kOk;
    }
    if (received == 0) {
        if (error != nullptr) {
            *error = "serial device hung up";
        }
        return ReadResult::kClosed;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return ReadResult::kTimeout;
    }
    if (error != nullptr) {
        *error = errno_message("read");
    }
    return ReadResult::kError;
}

bool SerialPort::write_all(
    const std::uint8_t* data,
    std::size_t size,
    int timeout_ms,
    std::string* error
) {
    if (fd_ < 0) {
        if (error != nullptr) {
            *error = "serial port is not open";
        }
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::size_t written = 0;
    while (written < size) {
        const ssize_t sent = ::write(fd_, data + written, size - written);
        if (sent > 0) {
            written += static_cast<std::size_t>(sent);
            continue;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       deadline - std::chrono::steady_clock::now()
            )
                                       .count();
            if (remaining <= 0) {
                if (error != nullptr) {
                    *error = "serial write timed out";
                }
                return false;
            }
            pollfd pfd {};
            pfd.fd = fd_;
            pfd.events = POLLOUT;
            const int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
            if (ready == 0) {
                if (error != nullptr) {
                    *error = "serial write timed out";
                }
                return false;
            }
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (error != nullptr) {
                    *error = errno_message("poll for write");
                }
                return false;
            }
            continue;
        }
        if (error != nullptr) {
            *error = errno_message("write");
        }
        return false;
    }
    return true;
}

} // namespace autoaim_serial
