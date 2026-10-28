#include "autoaim_serial/protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace autoaim_serial {

static_assert(sizeof(float) == 4, "wire protocol assumes 32-bit float");
static_assert(
    std::numeric_limits<float>::is_iec559,
    "wire protocol assumes IEEE-754 binary32 float"
);

namespace {
    void write_u16(std::uint8_t* out, std::uint16_t value) {
        out[0] = static_cast<std::uint8_t>(value & 0xFFU);
        out[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    }

    std::uint16_t read_u16(const std::uint8_t* in) {
        return static_cast<std::uint16_t>(in[0])
            | static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[1]) << 8U);
    }

    void write_f32(std::uint8_t* out, float value) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        out[0] = static_cast<std::uint8_t>(bits & 0xFFU);
        out[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
        out[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xFFU);
        out[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xFFU);
    }

    float read_f32(const std::uint8_t* in) {
        const std::uint32_t bits = static_cast<std::uint32_t>(in[0])
            | (static_cast<std::uint32_t>(in[1]) << 8U) | (static_cast<std::uint32_t>(in[2]) << 16U)
            | (static_cast<std::uint32_t>(in[3]) << 24U);
        float value = 0.0F;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    bool is_finite(float value) {
        return std::isfinite(value);
    }
} // namespace

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[i]) << 8U);
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

std::size_t sanitize_command(Command* command) {
    if (command == nullptr) {
        return 0;
    }
    std::size_t sanitized = 0;
    const auto fix = [&sanitized](float& value) {
        if (!is_finite(value)) {
            value = 0.0F;
            ++sanitized;
        }
    };
    fix(command->yaw_error);
    fix(command->pitch_error);
    fix(command->yaw_rate);
    fix(command->pitch_rate);
    return sanitized;
}

std::size_t encode_command(
    const Command& command,
    std::uint8_t seq,
    std::uint8_t* buffer,
    std::size_t capacity,
    std::size_t* sanitized_fields
) {
    if (buffer == nullptr || capacity < kCommandFrameSize) {
        return 0;
    }
    Command safe = command;
    const std::size_t sanitized = sanitize_command(&safe);
    if (sanitized_fields != nullptr) {
        *sanitized_fields = sanitized;
    }

    buffer[0] = kSof0;
    buffer[1] = kSof1;
    buffer[2] = kMsgIdCommand;
    buffer[3] = seq;
    buffer[4] = static_cast<std::uint8_t>(kCommandPayloadSize);
    buffer[5] = safe.status;
    write_f32(buffer + 6, safe.yaw_error);
    write_f32(buffer + 10, safe.pitch_error);
    write_f32(buffer + 14, safe.yaw_rate);
    write_f32(buffer + 18, safe.pitch_rate);
    write_u16(buffer + kCommandFrameSize - 2, crc16_ccitt(buffer + 2, kCommandPayloadSize + 3));
    return kCommandFrameSize;
}

std::size_t encode_feedback(const Feedback& feedback, std::uint8_t* buffer, std::size_t capacity) {
    if (buffer == nullptr || capacity < kFeedbackFrameSize) {
        return 0;
    }
    const float yaw = is_finite(feedback.yaw) ? feedback.yaw : 0.0F;
    const float pitch = is_finite(feedback.pitch) ? feedback.pitch : 0.0F;

    buffer[0] = kSof0;
    buffer[1] = kSof1;
    buffer[2] = kMsgIdFeedback;
    buffer[3] = feedback.seq;
    buffer[4] = static_cast<std::uint8_t>(kFeedbackPayloadSize);
    write_f32(buffer + 5, yaw);
    write_f32(buffer + 9, pitch);
    write_u16(buffer + kFeedbackFrameSize - 2, crc16_ccitt(buffer + 2, kFeedbackPayloadSize + 3));
    return kFeedbackFrameSize;
}

bool decode_feedback(const std::uint8_t* payload, std::size_t size, Feedback* feedback) {
    if (payload == nullptr || feedback == nullptr || size != kFeedbackPayloadSize) {
        return false;
    }
    const float yaw = read_f32(payload);
    const float pitch = read_f32(payload + 4);
    if (!is_finite(yaw) || !is_finite(pitch)) {
        return false;
    }
    feedback->yaw = yaw;
    feedback->pitch = pitch;
    return true;
}

FrameParser::FrameParser(FeedbackCallback on_feedback): on_feedback_(std::move(on_feedback)) {
    buffer_.reserve(kMaxPayloadSize + 7);
}

void FrameParser::push(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
        return;
    }
    buffer_.insert(buffer_.end(), data, data + size);
    process();
}

void FrameParser::reset() {
    buffer_.clear();
}

std::size_t FrameParser::crc_errors() const {
    return crc_errors_.load();
}

std::size_t FrameParser::ignored_frames() const {
    return ignored_frames_.load();
}

std::size_t FrameParser::resyncs() const {
    return resyncs_.load();
}

std::size_t FrameParser::find_sof() const {
    if (buffer_.size() < 2) {
        return kNotFound;
    }
    for (std::size_t i = 0; i + 1 < buffer_.size(); ++i) {
        if (buffer_[i] == kSof0 && buffer_[i + 1] == kSof1) {
            return i;
        }
    }
    return kNotFound;
}

void FrameParser::process() {
    while (true) {
        const std::size_t sof = find_sof();
        if (sof == kNotFound) {
            // Keep a trailing SOF0: the matching SOF1 may arrive in the next read.
            if (!buffer_.empty() && buffer_.back() == kSof0) {
                const std::uint8_t tail = buffer_.back();
                buffer_.clear();
                buffer_.push_back(tail);
            } else {
                buffer_.clear();
            }
            return;
        }
        if (sof > 0) {
            ++resyncs_;
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(sof));
        }
        if (buffer_.size() < 5) {
            return;
        }

        const std::uint8_t msg_id = buffer_[2];
        const std::size_t payload_size = buffer_[4];
        if (payload_size == 0 || payload_size > kMaxPayloadSize) {
            ++resyncs_;
            buffer_.erase(buffer_.begin());
            continue;
        }

        const std::size_t frame_size = 5 + payload_size + 2;
        if (buffer_.size() < frame_size) {
            return;
        }

        const std::uint16_t received = read_u16(buffer_.data() + 5 + payload_size);
        const std::uint16_t computed = crc16_ccitt(buffer_.data() + 2, payload_size + 3);
        if (received != computed) {
            ++crc_errors_;
            ++resyncs_;
            // Drop one byte and rescan: a valid frame may start inside the
            // discarded payload (e.g. after a truncated frame or line noise).
            buffer_.erase(buffer_.begin());
            continue;
        }

        if (msg_id == kMsgIdFeedback && payload_size == kFeedbackPayloadSize) {
            Feedback feedback;
            feedback.seq = buffer_[3];
            if (decode_feedback(buffer_.data() + 5, payload_size, &feedback)) {
                if (on_feedback_) {
                    on_feedback_(feedback);
                }
            } else {
                ++ignored_frames_;
            }
        } else {
            ++ignored_frames_;
        }

        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    }
}

} // namespace autoaim_serial
