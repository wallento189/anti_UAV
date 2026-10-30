#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "autoaim_serial/protocol.hpp"

namespace {

using autoaim_serial::Command;
using autoaim_serial::Feedback;
using autoaim_serial::FrameParser;
using autoaim_serial::ParseError;
using autoaim_serial::ParseErrorInfo;

// clang-format off
const std::uint8_t kCommandFrame[] = {
    0xAA, 0x55, 0x01, 0x00, 0x11, 0x01, 0xCD, 0xCC, 0x4C, 0x3D, 0x0A, 0xD7,
    0xA3, 0xBC, 0x9A, 0x99, 0x99, 0x3E, 0xCD, 0xCC, 0xCC, 0xBD, 0xCF, 0x82,
};

const std::uint8_t kFeedbackFrame[] = {
    0xAA, 0x55, 0x02, 0x07, 0x08, 0xCD, 0xCC, 0xCC, 0xBD,
    0xCD, 0xCC, 0x4C, 0x3D, 0x8E, 0xF4,
};
// clang-format on

std::vector<std::uint8_t> to_vector(const std::uint8_t* data, std::size_t size) {
    return std::vector<std::uint8_t>(data, data + size);
}

void rewrite_crc(std::vector<std::uint8_t>& frame) {
    ASSERT_GE(frame.size(), 7U);
    const std::size_t payload_size = frame[4];
    ASSERT_EQ(frame.size(), 5 + payload_size + 2);
    const std::uint16_t crc = autoaim_serial::crc16_ccitt(frame.data() + 2, payload_size + 3);
    frame[5 + payload_size] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame[5 + payload_size + 1] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
}

Command sample_command() {
    Command command;
    command.status = 1;
    command.yaw_target = 0.05F;
    command.pitch_target = -0.02F;
    command.yaw_rate = 0.3F;
    command.pitch_rate = -0.1F;
    return command;
}

} // namespace

TEST(Crc16, KnownVector) {
    const std::uint8_t data[] = "123456789";
    EXPECT_EQ(autoaim_serial::crc16_ccitt(data, 9), 0x29B1U);
}

TEST(Encoding, CommandFrameIsStable) {
    std::uint8_t frame[autoaim_serial::kCommandFrameSize + 1];
    const std::size_t size =
        autoaim_serial::encode_command(sample_command(), 0, frame, sizeof(frame));
    ASSERT_EQ(size, autoaim_serial::kCommandFrameSize);
    EXPECT_EQ(to_vector(frame, size), to_vector(kCommandFrame, sizeof(kCommandFrame)));
}

TEST(Encoding, FeedbackFrameIsStable) {
    Feedback feedback;
    feedback.seq = 7;
    feedback.yaw = -0.1F;
    feedback.pitch = 0.05F;

    std::uint8_t frame[autoaim_serial::kFeedbackFrameSize + 1];
    const std::size_t size = autoaim_serial::encode_feedback(feedback, frame, sizeof(frame));
    ASSERT_EQ(size, autoaim_serial::kFeedbackFrameSize);
    EXPECT_EQ(to_vector(frame, size), to_vector(kFeedbackFrame, sizeof(kFeedbackFrame)));
}

TEST(Encoding, RejectsShortBuffer) {
    Command command = sample_command();
    std::uint8_t frame[autoaim_serial::kCommandFrameSize - 1];
    EXPECT_EQ(autoaim_serial::encode_command(command, 0, frame, sizeof(frame)), 0U);
}

TEST(Encoding, SanitizesNonFiniteCommand) {
    Command dirty = sample_command();
    dirty.yaw_target = std::numeric_limits<float>::quiet_NaN();
    dirty.pitch_rate = std::numeric_limits<float>::infinity();

    std::uint8_t dirty_frame[autoaim_serial::kCommandFrameSize];
    std::size_t sanitized = 0;
    const std::size_t dirty_size =
        autoaim_serial::encode_command(dirty, 0, dirty_frame, sizeof(dirty_frame), &sanitized);
    ASSERT_EQ(dirty_size, autoaim_serial::kCommandFrameSize);
    EXPECT_EQ(sanitized, 2U);

    Command clean = sample_command();
    clean.yaw_target = 0.0F;
    clean.pitch_rate = 0.0F;
    std::uint8_t clean_frame[autoaim_serial::kCommandFrameSize];
    ASSERT_EQ(
        autoaim_serial::encode_command(clean, 0, clean_frame, sizeof(clean_frame)),
        autoaim_serial::kCommandFrameSize
    );
    EXPECT_EQ(to_vector(dirty_frame, dirty_size), to_vector(clean_frame, sizeof(clean_frame)));
}

TEST(Decoding, FeedbackRoundTrip) {
    Feedback in;
    in.yaw = 0.123F;
    in.pitch = -0.456F;

    std::uint8_t frame[autoaim_serial::kFeedbackFrameSize];
    ASSERT_EQ(
        autoaim_serial::encode_feedback(in, frame, sizeof(frame)),
        autoaim_serial::kFeedbackFrameSize
    );

    Feedback out;
    ASSERT_TRUE(
        autoaim_serial::decode_feedback(frame + 5, autoaim_serial::kFeedbackPayloadSize, &out)
    );
    EXPECT_FLOAT_EQ(out.yaw, in.yaw);
    EXPECT_FLOAT_EQ(out.pitch, in.pitch);
}

TEST(Decoding, RejectsWrongPayloadSize) {
    Feedback out;
    const std::uint8_t payload[4] = {0, 0, 0, 0};
    EXPECT_FALSE(autoaim_serial::decode_feedback(payload, 4, &out));
}

TEST(Parser, DeliversFeedbackAcrossChunks) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    parser.push(kFeedbackFrame, 1);
    parser.push(kFeedbackFrame + 1, 3);
    parser.push(kFeedbackFrame + 4, sizeof(kFeedbackFrame) - 4);

    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0].seq, 7U);
    EXPECT_FLOAT_EQ(received[0].yaw, -0.1F);
    EXPECT_FLOAT_EQ(received[0].pitch, 0.05F);
    EXPECT_EQ(parser.crc_errors(), 0U);
}

TEST(Parser, ResyncsAfterGarbage) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    const std::uint8_t garbage[] = {0x00, 0xFF, 0xAA, 0x00, 0x55, 0xAA};
    parser.push(garbage, sizeof(garbage));
    parser.push(kFeedbackFrame, sizeof(kFeedbackFrame));

    ASSERT_EQ(received.size(), 1U);
    EXPECT_GE(parser.resyncs(), 1U);
}

TEST(Parser, RejectsBadCrcAndRecovers) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    std::vector<std::uint8_t> corrupt = to_vector(kFeedbackFrame, sizeof(kFeedbackFrame));
    corrupt[6] ^= 0x01; // flip a payload bit, leave the transmitted CRC untouched
    parser.push(corrupt.data(), corrupt.size());
    parser.push(kFeedbackFrame, sizeof(kFeedbackFrame));

    EXPECT_EQ(parser.crc_errors(), 1U);
    ASSERT_EQ(received.size(), 1U);
    EXPECT_FLOAT_EQ(received[0].yaw, -0.1F);
}

TEST(Parser, RecoversFrameInsideCorruptedPayload) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    // A claim of a 32-byte payload that contains a complete valid frame. The
    // outer CRC is wrong, so the parser must drop one byte and rescan inward.
    std::vector<std::uint8_t> wrapper = {0xAA, 0x55, 0x02, 0x00, 0x20};
    wrapper.insert(wrapper.end(), kFeedbackFrame, kFeedbackFrame + sizeof(kFeedbackFrame));
    wrapper.resize(5 + 32, 0x00);
    wrapper.push_back(0x00);
    wrapper.push_back(0x00);

    parser.push(wrapper.data(), wrapper.size());

    ASSERT_EQ(received.size(), 1U);
    EXPECT_FLOAT_EQ(received[0].yaw, -0.1F);
    EXPECT_GE(parser.crc_errors(), 1U);
}

TEST(Parser, RejectsOversizeLengthAndRecovers) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    const std::uint8_t oversized[] = {0xAA, 0x55, 0x02, 0x00, 0xFF, 0x01, 0x02};
    parser.push(oversized, sizeof(oversized));
    parser.push(kFeedbackFrame, sizeof(kFeedbackFrame));

    ASSERT_EQ(received.size(), 1U);
    EXPECT_GE(parser.resyncs(), 1U);
}

TEST(Parser, IgnoresCommandFrames) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    parser.push(kCommandFrame, sizeof(kCommandFrame));
    parser.push(kFeedbackFrame, sizeof(kFeedbackFrame));

    EXPECT_EQ(parser.ignored_frames(), 1U);
    ASSERT_EQ(received.size(), 1U);
}

TEST(Parser, RejectsNonFiniteFeedback) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    std::vector<std::uint8_t> frame = to_vector(kFeedbackFrame, sizeof(kFeedbackFrame));
    const std::uint32_t nan_bits = 0x7FC00000U;
    frame[5] = static_cast<std::uint8_t>(nan_bits & 0xFFU);
    frame[6] = static_cast<std::uint8_t>((nan_bits >> 8U) & 0xFFU);
    frame[7] = static_cast<std::uint8_t>((nan_bits >> 16U) & 0xFFU);
    frame[8] = static_cast<std::uint8_t>((nan_bits >> 24U) & 0xFFU);
    rewrite_crc(frame);

    parser.push(frame.data(), frame.size());

    EXPECT_TRUE(received.empty());
    EXPECT_EQ(parser.bad_feedback_value_frames(), 1U);
    EXPECT_EQ(parser.ignored_frames(), 0U);
}

TEST(Parser, RejectsFeedbackWithUnexpectedLength) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    // 0x02 with a 12-byte payload: a firmware/protocol mismatch, not an
    // unknown message id. It must be counted as a bad feedback frame.
    std::vector<std::uint8_t> frame = to_vector(kFeedbackFrame, sizeof(kFeedbackFrame));
    frame[4] = 12;
    for (int i = 0; i < 4; ++i) {
        frame.insert(frame.end() - 2, 0x00);
    }
    rewrite_crc(frame);
    ASSERT_EQ(frame.size(), 19U);

    parser.push(frame.data(), frame.size());

    EXPECT_TRUE(received.empty());
    EXPECT_EQ(parser.bad_feedback_length_frames(), 1U);
    EXPECT_EQ(parser.bad_feedback_value_frames(), 0U);
    EXPECT_EQ(parser.ignored_frames(), 0U);
    EXPECT_EQ(parser.last_bad_feedback_length(), 12U);
}

TEST(Parser, KeepsTrailingSofBetweenReads) {
    std::vector<Feedback> received;
    FrameParser parser([&received](const Feedback& feedback) { received.push_back(feedback); });

    parser.push(kFeedbackFrame, 2);
    parser.push(kFeedbackFrame + 2, sizeof(kFeedbackFrame) - 2);

    ASSERT_EQ(received.size(), 1U);
}

TEST(Parser, ReportsGarbageWithTheDiscardedBytes) {
    std::vector<ParseErrorInfo> errors;
    FrameParser parser(
        [](const Feedback&) {},
        [&errors](const ParseErrorInfo& info) { errors.push_back(info); }
    );

    const std::uint8_t garbage[] = {0x00, 0x11, 0x22, 0x33};
    parser.push(garbage, sizeof(garbage));

    ASSERT_EQ(errors.size(), 1U);
    EXPECT_EQ(errors[0].kind, ParseError::kGarbage);
    ASSERT_EQ(errors[0].size, sizeof(garbage));
    EXPECT_EQ(errors[0].header[0], 0x00U);
    EXPECT_EQ(errors[0].header[3], 0x33U);
}

TEST(Parser, ReportsBadCrcFrameHeader) {
    std::vector<Feedback> received;
    std::vector<ParseErrorInfo> errors;
    FrameParser parser(
        [&received](const Feedback& feedback) { received.push_back(feedback); },
        [&errors](const ParseErrorInfo& info) { errors.push_back(info); }
    );

    std::vector<std::uint8_t> corrupt = to_vector(kFeedbackFrame, sizeof(kFeedbackFrame));
    corrupt[6] ^= 0x01;
    parser.push(corrupt.data(), corrupt.size());

    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(errors[0].kind, ParseError::kBadCrc);
    ASSERT_GE(errors[0].size, 5U);
    EXPECT_EQ(errors[0].header[0], 0xAAU);
    EXPECT_EQ(errors[0].header[1], 0x55U);
    EXPECT_EQ(errors[0].header[2], 0x02U);
}

TEST(Parser, ReportsBadLength) {
    std::vector<ParseErrorInfo> errors;
    FrameParser parser(
        [](const Feedback&) {},
        [&errors](const ParseErrorInfo& info) { errors.push_back(info); }
    );

    const std::uint8_t oversized[] = {0xAA, 0x55, 0x02, 0x00, 0xFF, 0x01, 0x02};
    parser.push(oversized, sizeof(oversized));

    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(errors[0].kind, ParseError::kBadLength);
    ASSERT_GE(errors[0].size, 5U);
    EXPECT_EQ(errors[0].header[4], 0xFFU);
}
