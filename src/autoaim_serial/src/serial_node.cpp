#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
    // How many of the first bytes after a (re)connect are dumped as hex. Enough
    // to recognise a frame header, short enough to stay on one log line.
    constexpr std::size_t kRawPreviewBytes = 32;
    // Beyond this the feedback angles look like a unit or endianness mistake
    // rather than a real gimbal pose (multi-turn gimbals still stay well below).
    constexpr double kPlausibleAngleRad = 3.0 * 3.14159265358979323846;

    const char* parse_error_name(autoaim_serial::ParseError kind) {
        switch (kind) {
            case autoaim_serial::ParseError::kBadLength:
                return "bad length";
            case autoaim_serial::ParseError::kBadCrc:
                return "bad CRC";
            case autoaim_serial::ParseError::kBadFeedbackLength:
                return "bad feedback length";
            case autoaim_serial::ParseError::kBadFeedbackValue:
                return "non-finite feedback angle";
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
    double min_feedback_rate_hz_ = 0.0;

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
    std::atomic<std::size_t> rx_bytes_ {0};
    std::atomic<std::size_t> reconnects_ {0};
    std::atomic<std::size_t> sanitized_fields_ {0};
    // Feedback continuity, so a link that "works" at a few Hz is still visible.
    std::atomic<std::size_t> feedback_seq_gaps_ {0};
    std::atomic<std::size_t> feedback_missing_ {0};
    std::atomic<double> max_feedback_gap_s_ {0.0};
    std::atomic<double> last_yaw_ {0.0};
    std::atomic<double> last_pitch_ {0.0};
    std::size_t last_tx_frames_ = 0;
    std::size_t last_tx_errors_ = 0;
    std::size_t last_tx_dropped_ = 0;
    std::size_t last_rx_frames_ = 0;
    std::size_t last_rx_bytes_ = 0;
    std::size_t last_reconnects_ = 0;
    std::size_t last_bad_feedback_length_ = 0;
    std::size_t last_bad_feedback_value_ = 0;
    std::size_t last_crc_errors_ = 0;
    std::size_t last_ignored_frames_ = 0;
    std::size_t last_resyncs_ = 0;
    std::size_t last_seq_gaps_ = 0;
    std::size_t last_missing_ = 0;
    std::size_t last_sanitized_ = 0;

    std::chrono::steady_clock::time_point last_target_time_ {};
    bool has_target_time_ = false;
    double max_target_interval_s_ = 0.0;
    std::atomic<std::int64_t> last_feedback_ns_ {0};

    // Reader-thread only: sequence tracking and the once-per-connection logs.
    std::uint8_t prev_feedback_seq_ = 0;
    bool has_feedback_seq_ = false;
    bool logged_first_feedback_ = false;
    // Per-kind rate limit for parse-error logs. A single shared throttle lets a
    // garbage flood hide a protocol mismatch on the 0x02 frames behind it.
    std::array<
        std::chrono::steady_clock::time_point,
        static_cast<std::size_t>(autoaim_serial::ParseError::kBadFeedbackValue) + 1>
        last_parse_error_log_ {};
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
    min_feedback_rate_hz_ = declare_parameter<double>("min_feedback_rate_hz");

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
    if (min_feedback_rate_hz_ <= 0.0) {
        throw std::invalid_argument("min_feedback_rate_hz must be positive");
    }
}

void SerialNode::open_port() {
    std::string error;
    if (!port_.open(device_, baudrate_, &error)) {
        // Not fatal: the reader thread keeps retrying and picks the device up
        // as soon as it shows up. It also owns the (throttled) retry log, so
        // this path stays quiet to avoid printing the same failure twice.
        port_ready_.store(false);
        return;
    }
    port_ready_.store(true);
    has_feedback_seq_ = false;
    logged_first_feedback_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "Serial port '%s' open at %d baud; waiting for 0x02 feedback frames",
        device_.c_str(),
        baudrate_
    );
}

bool SerialNode::reopen_port() {
    std::string error;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (port_.open(device_, baudrate_, &error)) {
            parser_->reset();
            port_ready_.store(true);
            has_feedback_seq_ = false;
            logged_first_feedback_ = false;
            const std::size_t attempt = reconnects_.fetch_add(1) + 1;
            RCLCPP_INFO(
                this->get_logger(),
                "Serial port '%s' reopened at %d baud (reconnect #%zu); waiting for 0x02 "
                "feedback frames",
                device_.c_str(),
                baudrate_,
                attempt
            );
            return true;
        }
    }
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for '%s' at %d baud: %s; retrying every %.1f s",
        device_.c_str(),
        baudrate_,
        error.c_str(),
        reconnect_interval_s_
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
    bool dumped_first_bytes = false;
    while (!stop_.load()) {
        if (!port_ready_.load()) {
            if (!reopen_port()) {
                sleep_interruptible(reconnect_interval_s_);
            } else {
                dumped_first_bytes = false;
            }
            continue;
        }

        std::size_t bytes_read = 0;
        std::string error;
        const auto result = port_.read(buffer, sizeof(buffer), kReadTimeoutMs, &bytes_read, &error);
        if (result == autoaim_serial::SerialPort::ReadResult::kOk) {
            rx_bytes_.fetch_add(bytes_read);
            if (!dumped_first_bytes) {
                dumped_first_bytes = true;
                // First chunk after a connect tells apart "board is silent",
                // "wrong baudrate" and "frame layout changed" far quicker than
                // any counter can: at a wrong baudrate the bytes look random.
                const std::size_t preview = std::min(bytes_read, kRawPreviewBytes);
                RCLCPP_INFO(
                    this->get_logger(),
                    "First %zu raw byte(s) from '%s': %s%s",
                    bytes_read,
                    device_.c_str(),
                    hex_bytes(buffer, preview).c_str(),
                    bytes_read > preview ? " ..." : ""
                );
            }
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
            "Serial link lost (%s) after %zu feedback frame(s) and %zu byte(s); retrying every "
            "%.1f s",
            error.c_str(),
            rx_frames_.load(),
            rx_bytes_.load(),
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
    const auto arrival = std::chrono::steady_clock::now();
    const std::int64_t arrival_ns = arrival.time_since_epoch().count();
    const std::int64_t previous_ns = last_feedback_ns_.exchange(arrival_ns);
    if (previous_ns != 0) {
        const double gap_s = static_cast<double>(arrival_ns - previous_ns) / 1e9;
        if (gap_s > max_feedback_gap_s_.load(std::memory_order_relaxed)) {
            max_feedback_gap_s_.store(gap_s, std::memory_order_relaxed);
        }
    }

    if (!logged_first_feedback_) {
        logged_first_feedback_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "First feedback frame decoded: seq=%u yaw=%.4f rad pitch=%.4f rad",
            static_cast<unsigned>(feedback.seq),
            static_cast<double>(feedback.yaw),
            static_cast<double>(feedback.pitch)
        );
    }

    if (has_feedback_seq_) {
        // seq is a wrapping uint8, so compare modulo 256: 1 is consecutive,
        // 0 is a duplicate, small values imply dropped frames.
        const int delta =
            static_cast<int>(static_cast<std::uint8_t>(feedback.seq - prev_feedback_seq_));
        if (delta == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Duplicate feedback seq %u (a frame was decoded twice)",
                static_cast<unsigned>(feedback.seq)
            );
        } else if (delta <= 128) {
            if (delta > 1) {
                // The lower board numbers every frame, so a jump means frames
                // were lost or never sent. This is what starves the tracker's
                // attitude search even though "some" feedback is arriving.
                const auto missing = static_cast<std::size_t>(delta - 1);
                feedback_seq_gaps_.fetch_add(1);
                feedback_missing_.fetch_add(missing);
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Feedback seq jumped %u -> %u: %zu frame(s) missing (RX overrun, flow "
                    "control or lower-board dropout)",
                    static_cast<unsigned>(prev_feedback_seq_),
                    static_cast<unsigned>(feedback.seq),
                    missing
                );
            }
        } else {
            // Backwards by more than a duplicate: the board most likely
            // restarted, so the gap count would be meaningless.
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Feedback seq went %u -> %u (backwards); assuming the lower board restarted",
                static_cast<unsigned>(prev_feedback_seq_),
                static_cast<unsigned>(feedback.seq)
            );
        }
    }
    prev_feedback_seq_ = feedback.seq;
    has_feedback_seq_ = true;

    if (std::abs(feedback.yaw) > kPlausibleAngleRad
        || std::abs(feedback.pitch) > kPlausibleAngleRad) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Feedback angles look implausible (yaw=%.3f pitch=%.3f rad): check units "
            "(rad vs deg) and endianness on the lower board",
            static_cast<double>(feedback.yaw),
            static_cast<double>(feedback.pitch)
        );
    }

    rx_frames_.fetch_add(1);
    last_yaw_.store(static_cast<double>(feedback.yaw));
    last_pitch_.store(static_cast<double>(feedback.pitch));

    autoaim_interfaces::msg::GimbalState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.seq = feedback.seq;
    msg.yaw = feedback.yaw;
    msg.pitch = feedback.pitch;
    feedback_pub_->publish(msg);
}

void SerialNode::handle_parse_error(const autoaim_serial::ParseErrorInfo& error) {
    const std::size_t kind = static_cast<std::size_t>(error.kind);
    if (kind >= last_parse_error_log_.size()) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    auto& last_logged = last_parse_error_log_[kind];
    if (now - last_logged < std::chrono::seconds(1)) {
        return;
    }
    last_logged = now;

    const std::string dump = hex_bytes(error.header, error.size);
    RCLCPP_WARN(
        this->get_logger(),
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
    bool link_open = false;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        link_open = port_.is_open();
        if (link_open) {
            std::string error;
            written = port_.write_all(frame, size, kWriteTimeoutMs, &error);
            if (!written) {
                tx_errors_.fetch_add(1);
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Serial write failed (status=%u): %s",
                    static_cast<unsigned>(command.status),
                    error.c_str()
                );
            }
        }
    }
    if (written) {
        tx_frames_.fetch_add(1);
    } else {
        const std::size_t dropped = tx_dropped_.fetch_add(1) + 1;
        if (!link_open) {
            // The device is unplugged or still reconnecting; say so explicitly
            // instead of only surfacing the total in the statistics line.
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Serial port '%s' is not open; dropping command frames (status=%u, %zu dropped "
                "so far)",
                device_.c_str(),
                static_cast<unsigned>(command.status),
                dropped
            );
        }
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
    const std::size_t rx_bytes = rx_bytes_.load();
    const std::size_t reconnects = reconnects_.load();
    const std::size_t bad_feedback_length = parser_->bad_feedback_length_frames();
    const std::size_t bad_feedback_value = parser_->bad_feedback_value_frames();
    const std::size_t crc_errors = parser_->crc_errors();
    const std::size_t ignored_frames = parser_->ignored_frames();
    const std::size_t resyncs = parser_->resyncs();
    const std::size_t seq_gaps = feedback_seq_gaps_.load();
    const std::size_t missing = feedback_missing_.load();
    const std::size_t sanitized = sanitized_fields_.load();
    const bool link_open = port_ready_.load();

    const std::size_t tx_delta = tx_frames - last_tx_frames_;
    const std::size_t tx_error_delta = tx_errors - last_tx_errors_;
    const std::size_t tx_drop_delta = tx_dropped - last_tx_dropped_;
    const std::size_t rx_delta = rx_frames - last_rx_frames_;
    const std::size_t rx_byte_delta = rx_bytes - last_rx_bytes_;
    const std::size_t reconnect_delta = reconnects - last_reconnects_;
    const std::size_t bad_feedback_length_delta = bad_feedback_length - last_bad_feedback_length_;
    const std::size_t bad_feedback_value_delta = bad_feedback_value - last_bad_feedback_value_;
    const std::size_t crc_delta = crc_errors - last_crc_errors_;
    const std::size_t ignored_delta = ignored_frames - last_ignored_frames_;
    const std::size_t resync_delta = resyncs - last_resyncs_;
    const std::size_t seq_gap_delta = seq_gaps - last_seq_gaps_;
    const std::size_t missing_delta = missing - last_missing_;
    const std::size_t sanitized_delta = sanitized - last_sanitized_;

    const double rx_rate_hz = static_cast<double>(rx_delta) / statistics_period_s_;
    const double max_gap_ms = max_feedback_gap_s_.load(std::memory_order_relaxed) * 1000.0;

    RCLCPP_INFO(
        this->get_logger(),
        "serial stats: link %s | tx %zu (%.1f Hz, err %zu, dropped %zu) | rx %zu (%.1f Hz, "
        "%zu B, crc %zu, ignored %zu, bad len %zu, bad value %zu, resync %zu, seq gaps "
        "%zu/%zu frames) | reconnects %zu | sanitized %zu | max gap: feedback %.1f ms, "
        "target %.1f ms | yaw %.4f pitch %.4f rad",
        link_open ? "open" : "DOWN",
        tx_delta,
        static_cast<double>(tx_delta) / statistics_period_s_,
        tx_error_delta,
        tx_drop_delta,
        rx_delta,
        rx_rate_hz,
        rx_byte_delta,
        crc_delta,
        ignored_delta,
        bad_feedback_length_delta,
        bad_feedback_value_delta,
        resync_delta,
        seq_gap_delta,
        missing_delta,
        reconnect_delta,
        sanitized_delta,
        max_gap_ms,
        max_target_interval_s_ * 1000.0,
        last_yaw_.load(),
        last_pitch_.load()
    );

    if (!link_open) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Serial link on '%s' is down (%zu reconnect(s) so far); retrying every %.1f s",
            device_.c_str(),
            reconnects,
            reconnect_interval_s_
        );
    }

    if (tx_delta == 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "No TrackedTarget on '%s' in the last %.1f s (tracker also drops frames while it "
            "has no gimbal attitude)",
            target_topic_.c_str(),
            statistics_period_s_
        );
    }

    if (link_open && rx_delta == 0) {
        // Distinguish a silent link from a decoding failure: both look like
        // "no feedback", but the fixes are completely different.
        if (rx_byte_delta > 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "%zu raw byte(s) arrived on '%s' but none decoded as a feedback frame: check "
                "baudrate, frame layout and the 'Unparseable serial bytes' warnings above",
                rx_byte_delta,
                device_.c_str()
            );
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "No bytes at all from '%s' in the last %.1f s: the lower board is silent or its "
                "TX is not wired to this RX",
                device_.c_str(),
                statistics_period_s_
            );
        }
    } else if (rx_delta > 0 && rx_rate_hz < min_feedback_rate_hz_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Feedback rate %.1f Hz is below %.1f Hz (largest gap %.1f ms): the tracker rejects "
            "observations whose nearest attitude is older than its attitude_timeout_s (50 ms by "
            "default), so most TRACKING frames will be dropped",
            rx_rate_hz,
            min_feedback_rate_hz_,
            max_gap_ms
        );
    }

    if (bad_feedback_length_delta > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Rejected %zu feedback frame(s) with the wrong payload length (last %u, expected "
            "%zu): this build and the lower board disagree about the 0x02 layout",
            bad_feedback_length_delta,
            static_cast<unsigned>(parser_->last_bad_feedback_length()),
            autoaim_serial::kFeedbackPayloadSize
        );
    }
    if (bad_feedback_value_delta > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Rejected %zu feedback frame(s) with a non-finite angle: the lower board sent NaN/Inf "
            "as its pose",
            bad_feedback_value_delta
        );
    }
    if (ignored_delta > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignored %zu frame(s) with an unexpected msg_id (last 0x%02X); 0x01 means our own "
            "commands are being echoed back",
            ignored_delta,
            static_cast<unsigned>(parser_->last_ignored_msg_id())
        );
    }
    if (crc_delta > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "%zu CRC error(s) / %zu resync(s) in the last %.1f s: check baudrate, wiring and "
            "common ground",
            crc_delta,
            resync_delta,
            statistics_period_s_
        );
    }
    if (seq_gap_delta > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "%zu feedback seq gap(s), %zu frame(s) missing: RX overrun, or the lower board is "
            "skipping sends",
            seq_gap_delta,
            missing_delta
        );
    }

    const std::int64_t last_feedback = last_feedback_ns_.load();
    if (last_feedback != 0) {
        const std::int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        const double feedback_age_s = static_cast<double>(now_ns - last_feedback) / 1e9;
        if (feedback_age_s > feedback_timeout_s_) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Serial feedback is stale: %.1f ms since the last frame (timeout %.1f ms)",
                feedback_age_s * 1000.0,
                feedback_timeout_s_ * 1000.0
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
    if (reconnect_delta > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Serial link reconnected %zu time(s) in the last %.1f s; check the cable and USB "
            "power",
            reconnect_delta,
            statistics_period_s_
        );
    }

    last_tx_frames_ = tx_frames;
    last_tx_errors_ = tx_errors;
    last_tx_dropped_ = tx_dropped;
    last_rx_frames_ = rx_frames;
    last_rx_bytes_ = rx_bytes;
    last_reconnects_ = reconnects;
    last_bad_feedback_length_ = bad_feedback_length;
    last_bad_feedback_value_ = bad_feedback_value;
    last_crc_errors_ = crc_errors;
    last_ignored_frames_ = ignored_frames;
    last_resyncs_ = resyncs;
    last_seq_gaps_ = seq_gaps;
    last_missing_ = missing;
    last_sanitized_ = sanitized;
    max_target_interval_s_ = 0.0;
    max_feedback_gap_s_.store(0.0, std::memory_order_relaxed);
}

} // namespace SerialNode

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SerialNode::SerialNode)
