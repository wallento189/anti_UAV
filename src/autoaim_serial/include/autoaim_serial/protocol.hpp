#ifndef AUTOAIM_SERIAL__PROTOCOL_HPP_
#define AUTOAIM_SERIAL__PROTOCOL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace autoaim_serial {

// Wire framing shared with the lower board. Keep docs/serial_protocol.md in sync.
inline constexpr std::uint8_t kSof0 = 0xAA;
inline constexpr std::uint8_t kSof1 = 0x55;
inline constexpr std::uint8_t kMsgIdCommand = 0x01;
inline constexpr std::uint8_t kMsgIdFeedback = 0x02;

inline constexpr std::size_t kCommandPayloadSize = 17; // status + 4 x float32
inline constexpr std::size_t kFeedbackPayloadSize = 8; // 2 x float32
// Upper bound the parser accepts. It is a safety guard, not a wire constant.
inline constexpr std::size_t kMaxPayloadSize = 32;

inline constexpr std::size_t kCommandFrameSize = 2 + 1 + 1 + 1 + kCommandPayloadSize + 2;
inline constexpr std::size_t kFeedbackFrameSize = 2 + 1 + 1 + 1 + kFeedbackPayloadSize + 2;

// Command as carried on the wire: a one-to-one projection of TrackedTarget.
struct Command {
    std::uint8_t status = 0;
    float yaw_error = 0.0F;
    float pitch_error = 0.0F;
    float yaw_rate = 0.0F;
    float pitch_rate = 0.0F;
};

struct Feedback {
    std::uint8_t seq = 0;
    float yaw = 0.0F;
    float pitch = 0.0F;
};

using FeedbackCallback = std::function<void(const Feedback&)>;

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t size);

// Replaces non-finite floats with zero so a broken tracker value cannot poison
// the lower board's control loop. Returns the number of replaced fields.
std::size_t sanitize_command(Command* command);

// Returns the number of bytes written, or 0 when the buffer is too small.
// Non-finite floats are sanitized before encoding; `sanitized_fields` reports
// how many were replaced.
std::size_t encode_command(
    const Command& command,
    std::uint8_t seq,
    std::uint8_t* buffer,
    std::size_t capacity,
    std::size_t* sanitized_fields = nullptr
);

std::size_t encode_feedback(const Feedback& feedback, std::uint8_t* buffer, std::size_t capacity);

// Decodes a feedback payload. Returns false for malformed or non-finite values.
bool decode_feedback(const std::uint8_t* payload, std::size_t size, Feedback* feedback);

// Resynchronizing byte-stream parser. It only emits feedback frames; command
// frames (e.g. echoed back during a hardware loopback test) are counted and
// dropped, as are unknown message ids.
class FrameParser {
public:
    explicit FrameParser(FeedbackCallback on_feedback);

    // Feeds raw serial bytes. Complete, valid frames are delivered through the
    // callback while this call is running.
    void push(const std::uint8_t* data, std::size_t size);
    void reset();

    std::size_t crc_errors() const;
    std::size_t ignored_frames() const;
    std::size_t resyncs() const;

private:
    void process();
    std::size_t find_sof() const;

    static constexpr std::size_t kNotFound = static_cast<std::size_t>(-1);

    std::vector<std::uint8_t> buffer_;
    FeedbackCallback on_feedback_;
    // Read by the statistics timer while the reader thread feeds the parser.
    std::atomic<std::size_t> crc_errors_ {0};
    std::atomic<std::size_t> ignored_frames_ {0};
    std::atomic<std::size_t> resyncs_ {0};
};

} // namespace autoaim_serial

#endif // AUTOAIM_SERIAL__PROTOCOL_HPP_
