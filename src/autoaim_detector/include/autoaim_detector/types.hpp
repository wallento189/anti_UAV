#ifndef AUTOAIM_DETECTOR__TYPES_HPP_
#define AUTOAIM_DETECTOR__TYPES_HPP_

#include <vector>

namespace autoaim_detector {

inline constexpr int kInputSize = 640;
inline constexpr int kRegMax = 16;

struct RawTensor {
    int height = 0;
    int width = 0;
    int channels = 0;
    int stride = 0;
    bool is_box = false;
    std::vector<float> data;
};

struct LetterboxTransform {
    float scale = 1.0f;
    float pad_x = 0.0f;
    float pad_y = 0.0f;
    int orig_width = 0;
    int orig_height = 0;
};

struct Detection {
    bool detected = false;
    float score = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
};

} // namespace autoaim_detector

#endif // AUTOAIM_DETECTOR__TYPES_HPP_
