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
// yaw_target / pitch_target are absolute angles in the gimbal frame, not
// offsets from the optical axis, so applying the same frame twice is harmless.
struct Command {
    std::uint8_t status = 0;
    float yaw_target = 0.0F;
    float pitch_target = 0.0F;
    float yaw_rate = 0.0F;
    float pitch_rate = 0.0F;
};

struct Feedback {
    std::uint8_t seq = 0;
    float yaw = 0.0F;
    float pitch = 0.0F;
};

// Why the parser could not turn a byte range into a frame. Reported through
// ParseErrorCallback so the node can log the offending bytes instead of only
// bumping a counter.
enum class ParseError : std::uint8_t {
    kGarbage = 0, // bytes discarded while scanning for the SOF
    kBadLength = 1, // payload length outside the accepted range
    kBadCrc = 2, // frame delimited correctly but the CRC did not match
    // A 0x02 frame whose payload is not exactly 8 bytes: the lower board and
    // this build disagree about the layout. Must not be lumped into the
    // "unknown msg_id" bucket where it would be invisible.
    kBadFeedbackLength = 3,
    // A well-formed 0x02 frame carrying a NaN/Inf angle: the lower board's
    // control loop produced a non-finite pose.
    kBadFeedbackValue = 4,
};

struct ParseErrorInfo {
    ParseError kind = ParseError::kGarbage;
    // First bytes at the offending position, for logging. The parser owns the
    // storage; copy them if they are needed after the callback returns.
    std::uint8_t header[8] {};
    std::size_t size = 0;
};

using FeedbackCallback = std::function<void(const Feedback&)>;
using ParseErrorCallback = std::function<void(const ParseErrorInfo&)>;

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
    explicit FrameParser(FeedbackCallback on_feedback, ParseErrorCallback on_error = {});

    // Feeds raw serial bytes. Complete, valid frames are delivered through the
    // callback while this call is running.
    void push(const std::uint8_t* data, std::size_t size);
    void reset();

    std::size_t crc_errors() const;
    // Well-formed frames with a msg_id this node does not consume: the command
    // frames echoed back during a loopback test fall here too.
    std::size_t ignored_frames() const;
    // 0x02 frames that could not be decoded, split by cause so the node can
    // report the right one. Kept apart from ignored_frames so a firmware or
    // protocol mismatch shows up as an error instead of a silent count.
    std::size_t bad_feedback_length_frames() const; // payload size != 8
    std::size_t bad_feedback_value_frames() const; // NaN/Inf angle
    std::size_t resyncs() const;
    // Last offending msg_id / payload length, for log messages.
    std::uint8_t last_ignored_msg_id() const;
    std::uint8_t last_bad_feedback_length() const;

private:
    void process();
    std::size_t find_sof() const;
    void report_error(ParseError kind, const std::uint8_t* data, std::size_t size);

    static constexpr std::size_t kNotFound = static_cast<std::size_t>(-1);

    std::vector<std::uint8_t> buffer_;
    FeedbackCallback on_feedback_;
    ParseErrorCallback on_error_;
    // Read by the statistics timer while the reader thread feeds the parser.
    std::atomic<std::size_t> crc_errors_ {0};
    std::atomic<std::size_t> ignored_frames_ {0};
    std::atomic<std::size_t> bad_feedback_length_frames_ {0};
    std::atomic<std::size_t> bad_feedback_value_frames_ {0};
    std::atomic<std::size_t> resyncs_ {0};
    std::atomic<std::uint8_t> last_ignored_msg_id_ {0};
    std::atomic<std::uint8_t> last_bad_feedback_length_ {0};
};

} // namespace autoaim_serial

#endif // AUTOAIM_SERIAL__PROTOCOL_HPP_
