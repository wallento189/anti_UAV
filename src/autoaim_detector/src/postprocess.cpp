#include "autoaim_detector/postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace autoaim_detector {

namespace {

    float sigmoid(float x) {
        return 1.0f / (1.0f + std::exp(-x));
    }

    float dfl_expectation(const float* logits, int reg_max) {
        float max_logit = logits[0];
        for (int i = 1; i < reg_max; ++i) {
            max_logit = std::max(max_logit, logits[i]);
        }
        float sum = 0.0f;
        float weighted = 0.0f;
        for (int i = 0; i < reg_max; ++i) {
            const float p = std::exp(logits[i] - max_logit);
            sum += p;
            weighted += p * static_cast<float>(i);
        }
        return weighted / sum;
    }

} // namespace

Detection decode_detection(
    const std::vector<RawTensor>& outputs,
    const LetterboxTransform& transform,
    float confidence_thres
) {
    float best_score = -1.0f;
    const RawTensor* best_cls = nullptr;
    int best_row = 0;
    int best_col = 0;
    for (const RawTensor& tensor: outputs) {
        if (tensor.is_box) {
            continue;
        }
        for (int y = 0; y < tensor.height; ++y) {
            for (int x = 0; x < tensor.width; ++x) {
                // cls 的通道可能被 BPU 补齐到 4，只取第 0 通道。
                const float score = sigmoid(
                    tensor.data[(static_cast<size_t>(y) * tensor.width + x) * tensor.channels]
                );
                if (score > best_score) {
                    best_score = score;
                    best_cls = &tensor;
                    best_row = y;
                    best_col = x;
                }
            }
        }
    }

    Detection detection;
    if (best_cls == nullptr || best_score < confidence_thres) {
        return detection;
    }

    const RawTensor* box = nullptr;
    for (const RawTensor& tensor: outputs) {
        if (tensor.is_box && tensor.stride == best_cls->stride) {
            box = &tensor;
            break;
        }
    }
    if (box == nullptr) {
        throw std::runtime_error(
            "missing box tensor for stride " + std::to_string(best_cls->stride) + " in postprocess"
        );
    }

    const float* logits =
        box->data.data() + (static_cast<size_t>(best_row) * box->width + best_col) * box->channels;
    const float l = dfl_expectation(logits + 0 * kRegMax, kRegMax);
    const float t = dfl_expectation(logits + 1 * kRegMax, kRegMax);
    const float r = dfl_expectation(logits + 2 * kRegMax, kRegMax);
    const float b = dfl_expectation(logits + 3 * kRegMax, kRegMax);

    const float stride = static_cast<float>(box->stride);
    const float cx = (static_cast<float>(best_col) + 0.5f) * stride;
    const float cy = (static_cast<float>(best_row) + 0.5f) * stride;

    const auto to_original_x = [&](float value) {
        return std::clamp(
            (value - transform.pad_x) / transform.scale,
            0.0f,
            static_cast<float>(transform.orig_width)
        );
    };
    const auto to_original_y = [&](float value) {
        return std::clamp(
            (value - transform.pad_y) / transform.scale,
            0.0f,
            static_cast<float>(transform.orig_height)
        );
    };

    detection.detected = true;
    detection.score = best_score;
    detection.x1 = to_original_x(cx - l * stride);
    detection.y1 = to_original_y(cy - t * stride);
    detection.x2 = to_original_x(cx + r * stride);
    detection.y2 = to_original_y(cy + b * stride);
    detection.x = 0.5f * (detection.x1 + detection.x2);
    detection.y = 0.5f * (detection.y1 + detection.y2);
    return detection;
}

} // namespace autoaim_detector
