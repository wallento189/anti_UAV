#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "autoaim_interfaces/msg/gimbal_command.hpp"
#include "autoaim_interfaces/msg/gimbal_state.hpp"
#include "autoaim_interfaces/msg/tracked_target.hpp"
#include "autoaim_serial/protocol.hpp"
#include "autoaim_serial/serial_port.hpp"

namespace SerialNode {

namespace {
    constexpr int kReadTimeoutMs = 50;
    constexpr int kWriteTimeoutMs = 5;
    constexpr int kReconnectSleepSliceMs = 20;
    constexpr double kWarnTargetIntervalS = 0.2;

    const char* parse_error_name(autoaim_serial::ParseError kind) {
        switch (kind) {
            case autoaim_serial::ParseError::kBadLength:
                return "bad length";
            case autoaim_serial::ParseError::kBadCrc:
                return "bad CRC";
            case autoaim_serial::ParseError::kBadFeedback:
                return "bad feedback payload";
            default:
                return "garbage";
        }
    }

    std::string hex_bytes(const std::uint8_t* data, std::size_t size) {
        std::string out;
        out.reserve(size * 3);
        char byte[4];
        for (std::size_t i = 0; i < size; ++i) {
            std::snprintf(byte, sizeof(byte), "%02X", static_cast<unsigned>(data[i]));
            if (i > 0) {
                out.push_back(' ');
            }
            out += byte;
        }
        return out;
    }
} // namespace

class SerialNode: public rclcpp::Node {
public:
    explicit SerialNode(const rclcpp::NodeOptions& options);
    ~SerialNode() override;

private:
    void get_parameters();
    void open_port();
    bool reopen_port();
    void start_reader();
    void stop_reader();
    void reader_loop();
    void handle_feedback(const autoaim_serial::Feedback& feedback);
    void handle_parse_error(const autoaim_serial::ParseErrorInfo& error);
    void on_target(const autoaim_interfaces::msg::TrackedTarget::ConstSharedPtr& msg);
    void on_statistics();
    void sleep_interruptible(double seconds);

    std::string device_;
    int baudrate_ = 0;
    std::string target_topic_;
    std::string send_topic_;
    std::string feedback_topic_;
    std::string frame_id_;
    double feedback_timeout_s_ = 0.0;
    double reconnect_interval_s_ = 0.0;
    double statistics_period_s_ = 0.0;

    autoaim_serial::SerialPort port_;
    std::mutex port_mutex_;
    std::atomic<bool> port_ready_ {false};
    std::atomic<bool> stop_ {false};
    std::thread reader_thread_;
    std::unique_ptr<autoaim_serial::FrameParser> parser_;

    rclcpp::Subscription<autoaim_interfaces::msg::TrackedTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<autoaim_interfaces::msg::GimbalCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<autoaim_interfaces::msg::GimbalState>::SharedPtr feedback_pub_;
    rclcpp::TimerBase::SharedPtr statistics_timer_;

    std::uint8_t tx_seq_ = 0;

    // Monotonic totals, delta-reported every statistics period.
    std::atomic<std::size_t> tx_frames_ {0};
    std::atomic<std::size_t> tx_errors_ {0};
    std::atomic<std::size_t> tx_dropped_ {0};
    std::atomic<std::size_t> rx_frames_ {0};
    std::atomic<std::size_t> sanitized_fields_ {0};
    std::size_t last_tx_frames_ = 0;
    std::size_t last_tx_errors_ = 0;
    std::size_t last_tx_dropped_ = 0;
    std::size_t last_rx_frames_ = 0;
    std::size_t last_crc_errors_ = 0;
    std::size_t last_ignored_frames_ = 0;
    std::size_t last_resyncs_ = 0;
    std::size_t last_sanitized_ = 0;

    std::chrono::steady_clock::time_point last_target_time_ {};
    bool has_target_time_ = false;
    double max_target_interval_s_ = 0.0;
    std::atomic<std::int64_t> last_feedback_ns_ {0};
};

SerialNode::SerialNode(const rclcpp::NodeOptions& options):
    Node("autoaim_serial", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
    get_parameters();
    open_port();

    command_pub_ =
        create_publisher<autoaim_interfaces::msg::GimbalCommand>(send_topic_, rclcpp::QoS(10));
    feedback_pub_ =
        create_publisher<autoaim_interfaces::msg::GimbalState>(feedback_topic_, rclcpp::QoS(10));

    parser_ = std::make_unique<autoaim_serial::FrameParser>(
        [this](const autoaim_serial::Feedback& feedback) { handle_feedback(feedback); },
        [this](const autoaim_serial::ParseErrorInfo& error) { handle_parse_error(error); }
    );

    target_sub_ = create_subscription<autoaim_interfaces::msg::TrackedTarget>(
        target_topic_,
        rclcpp::QoS(10),
        std::bind(&SerialNode::on_target, this, std::placeholders::_1)
    );

    statistics_timer_ = create_wall_timer(
        std::chrono::duration<double>(statistics_period_s_),
        std::bind(&SerialNode::on_statistics, this)
    );

    start_reader();

    RCLCPP_INFO(
        this->get_logger(),
        "autoaim_serial started: '%s' @ %d baud, subscribing '%s', publishing '%s' and '%s'",
        device_.c_str(),
        baudrate_,
        target_topic_.c_str(),
        send_topic_.c_str(),
        feedback_topic_.c_str()
    );
}

SerialNode::~SerialNode() {
    stop_reader();
    std::lock_guard<std::mutex> lock(port_mutex_);
    port_.close();
}

void SerialNode::get_parameters() {
    device_ = declare_parameter<std::string>("device");
    baudrate_ = declare_parameter<int>("baudrate");
    target_topic_ = declare_parameter<std::string>("target_topic");
    send_topic_ = declare_parameter<std::string>("send_topic");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic");
    frame_id_ = declare_parameter<std::string>("frame_id");
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s");
    reconnect_interval_s_ = declare_parameter<double>("reconnect_interval_s");
    statistics_period_s_ = declare_parameter<double>("statistics_period_s");

    if (baudrate_ <= 0) {
        throw std::invalid_argument("baudrate must be positive");
    }
    if (!autoaim_serial::SerialPort::is_supported_baudrate(baudrate_)) {
        throw std::invalid_argument("unsupported baudrate " + std::to_string(baudrate_));
    }
    if (feedback_timeout_s_ <= 0.0) {
        throw std::invalid_argument("feedback_timeout_s must be positive");
    }
    if (reconnect_interval_s_ <= 0.0) {
        throw std::invalid_argument("reconnect_interval_s must be positive");
    }
    if (statistics_period_s_ <= 0.0) {
        throw std::invalid_argument("statistics_period_s must be positive");
    }
}

void SerialNode::open_port() {
    std::string error;
    if (!port_.open(device_, baudrate_, &error)) {
        // Not fatal: the reader thread keeps retrying and picks the device up
        // as soon as it shows up.
        port_ready_.store(false);
        RCLCPP_ERROR(
            this->get_logger(),
            "Cannot open '%s' at %d baud: %s; retrying every %.1f s",
            device_.c_str(),
            baudrate_,
            error.c_str(),
            reconnect_interval_s_
        );
        return;
    }
    port_ready_.store(true);
    RCLCPP_INFO(this->get_logger(), "Serial port '%s' open at %d baud", device_.c_str(), baudrate_);
}

bool SerialNode::reopen_port() {
    std::string error;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (port_.open(device_, baudrate_, &error)) {
            parser_->reset();
            port_ready_.store(true);
            RCLCPP_INFO(
                this->get_logger(),
                "Serial port '%s' reopened at %d baud",
                device_.c_str(),
                baudrate_
            );
            return true;
        }
    }
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for '%s': %s",
        device_.c_str(),
        error.c_str()
    );
    return false;
}

void SerialNode::start_reader() {
    stop_.store(false);
    reader_thread_ = std::thread(&SerialNode::reader_loop, this);
}

void SerialNode::stop_reader() {
    stop_.store(true);
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
}

void SerialNode::reader_loop() {
    std::uint8_t buffer[256];
    while (!stop_.load()) {
        if (!port_ready_.load()) {
            if (!reopen_port()) {
                sleep_interruptible(reconnect_interval_s_);
            }
            continue;
        }

        std::size_t bytes_read = 0;
        std::string error;
        const auto result = port_.read(buffer, sizeof(buffer), kReadTimeoutMs, &bytes_read, &error);
        if (result == autoaim_serial::SerialPort::ReadResult::kOk) {
            parser_->push(buffer, bytes_read);
            continue;
        }
        if (result == autoaim_serial::SerialPort::ReadResult::kTimeout) {
            continue;
        }

        port_ready_.store(false);
        {
            std::lock_guard<std::mutex> lock(port_mutex_);
            port_.close();
        }
        RCLCPP_ERROR(
            this->get_logger(),
            "Serial link lost (%s); retrying every %.1f s",
            error.c_str(),
            reconnect_interval_s_
        );
    }
}

void SerialNode::sleep_interruptible(double seconds) {
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(seconds)
        );
    while (!stop_.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kReconnectSleepSliceMs));
    }
}

void SerialNode::handle_feedback(const autoaim_serial::Feedback& feedback) {
    rx_frames_.fetch_add(1);
    last_feedback_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count());

    autoaim_interfaces::msg::GimbalState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.seq = feedback.seq;
    msg.yaw = feedback.yaw;
    msg.pitch = feedback.pitch;
    feedback_pub_->publish(msg);
}

void SerialNode::handle_parse_error(const autoaim_serial::ParseErrorInfo& error) {
    const std::string dump = hex_bytes(error.header, error.size);
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Unparseable serial bytes (%s, %zu B): %s",
        parse_error_name(error.kind),
        error.size,
        dump.c_str()
    );
}

void SerialNode::on_target(const autoaim_interfaces::msg::TrackedTarget::ConstSharedPtr& msg) {
    const auto arrival = std::chrono::steady_clock::now();
    if (has_target_time_) {
        const double interval = std::chrono::duration<double>(arrival - last_target_time_).count();
        max_target_interval_s_ = std::max(max_target_interval_s_, interval);
    }
    last_target_time_ = arrival;
    has_target_time_ = true;

    autoaim_serial::Command command;
    command.status = msg->status;
    command.yaw_target = msg->yaw_target;
    command.pitch_target = msg->pitch_target;
    command.yaw_rate = msg->yaw_rate;
    command.pitch_rate = msg->pitch_rate;

    const std::size_t sanitized = autoaim_serial::sanitize_command(&command);
    if (sanitized > 0) {
        sanitized_fields_.fetch_add(sanitized);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Replaced %zu non-finite command field(s) with 0",
            sanitized
        );
    }

    std::uint8_t frame[autoaim_serial::kCommandFrameSize];
    const std::size_t size = autoaim_serial::encode_command(command, tx_seq_, frame, sizeof(frame));
    ++tx_seq_;

    bool written = false;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (port_.is_open()) {
            std::string error;
            written = port_.write_all(frame, size, kWriteTimeoutMs, &error);
            if (!written) {
                tx_errors_.fetch_add(1);
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Serial write failed: %s",
                    error.c_str()
                );
            }
        }
    }
    if (written) {
        tx_frames_.fetch_add(1);
    } else {
        tx_dropped_.fetch_add(1);
    }

    // Mirror of the command the lower board just received (or would have
    // received when the link is down), so bags record what was attempted.
    autoaim_interfaces::msg::GimbalCommand out;
    out.header = msg->header;
    out.status = command.status;
    out.yaw_target = command.yaw_target;
    out.pitch_target = command.pitch_target;
    out.yaw_rate = command.yaw_rate;
    out.pitch_rate = command.pitch_rate;
    command_pub_->publish(out);
}

void SerialNode::on_statistics() {
    const std::size_t tx_frames = tx_frames_.load();
    const std::size_t tx_errors = tx_errors_.load();
    const std::size_t tx_dropped = tx_dropped_.load();
    const std::size_t rx_frames = rx_frames_.load();
    const std::size_t crc_errors = parser_->crc_errors();
    const std::size_t ignored_frames = parser_->ignored_frames();
    const std::size_t resyncs = parser_->resyncs();
    const std::size_t sanitized = sanitized_fields_.load();

    const std::size_t tx_delta = tx_frames - last_tx_frames_;
    const std::size_t tx_error_delta = tx_errors - last_tx_errors_;
    const std::size_t tx_drop_delta = tx_dropped - last_tx_dropped_;
    const std::size_t rx_delta = rx_frames - last_rx_frames_;
    const std::size_t crc_delta = crc_errors - last_crc_errors_;
    const std::size_t ignored_delta = ignored_frames - last_ignored_frames_;
    const std::size_t resync_delta = resyncs - last_resyncs_;
    const std::size_t sanitized_delta = sanitized - last_sanitized_;

    RCLCPP_INFO(
        this->get_logger(),
        "serial stats: tx %zu (%.1f Hz, err %zu, dropped %zu) | rx %zu (%.1f Hz, crc %zu, "
        "ignored %zu, resync %zu) | sanitized %zu | max target gap %.1f ms",
        tx_delta,
        static_cast<double>(tx_delta) / statistics_period_s_,
        tx_error_delta,
        tx_drop_delta,
        rx_delta,
        static_cast<double>(rx_delta) / statistics_period_s_,
        crc_delta,
        ignored_delta,
        resync_delta,
        sanitized_delta,
        max_target_interval_s_ * 1000.0
    );

    if (tx_delta == 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "No TrackedTarget on '%s' in the last %.1f s",
            target_topic_.c_str(),
            statistics_period_s_
        );
    }
    if (rx_delta == 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "No serial feedback on '%s' in the last %.1f s",
            feedback_topic_.c_str(),
            statistics_period_s_
        );
    }
    const std::int64_t last_feedback = last_feedback_ns_.load();
    if (last_feedback != 0) {
        const std::int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        const double feedback_age_s = static_cast<double>(now_ns - last_feedback) / 1e9;
        if (feedback_age_s > feedback_timeout_s_) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Serial feedback is stale: %.1f ms since the last frame",
                feedback_age_s * 1000.0
            );
        }
    }
    if (max_target_interval_s_ > kWarnTargetIntervalS) {
        RCLCPP_WARN(
            this->get_logger(),
            "Largest gap between commands was %.1f ms; is the pipeline keeping up?",
            max_target_interval_s_ * 1000.0
        );
    }

    last_tx_frames_ = tx_frames;
    last_tx_errors_ = tx_errors;
    last_tx_dropped_ = tx_dropped;
    last_rx_frames_ = rx_frames;
    last_crc_errors_ = crc_errors;
    last_ignored_frames_ = ignored_frames;
    last_resyncs_ = resyncs;
    last_sanitized_ = sanitized;
    max_target_interval_s_ = 0.0;
}

} // namespace SerialNode

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SerialNode::SerialNode)
