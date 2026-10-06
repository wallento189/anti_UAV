#include "autoaim_detector/preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace autoaim_detector {

namespace {
    inline constexpr int kPadValue = 114;
} // namespace

LetterboxTransform make_letterbox_transform(int width, int height) {
    LetterboxTransform transform;
    transform.orig_width = width;
    transform.orig_height = height;
    transform.scale = std::min(
        static_cast<float>(kInputSize) / static_cast<float>(width),
        static_cast<float>(kInputSize) / static_cast<float>(height)
    );
    const int new_width = static_cast<int>(std::lround(width * transform.scale));
    const int new_height = static_cast<int>(std::lround(height * transform.scale));
    transform.pad_x = static_cast<float>(kInputSize - new_width) / 2.0f;
    transform.pad_y = static_cast<float>(kInputSize - new_height) / 2.0f;
    return transform;
}

cv::Mat letterbox_image(const cv::Mat& bgr, const LetterboxTransform& transform) {
    const int new_width = static_cast<int>(std::lround(transform.orig_width * transform.scale));
    const int new_height = static_cast<int>(std::lround(transform.orig_height * transform.scale));
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(new_width, new_height), 0.0, 0.0, cv::INTER_LINEAR);

    const int pad_left = static_cast<int>(std::lround(transform.pad_x));
    const int pad_top = static_cast<int>(std::lround(transform.pad_y));
    const int pad_right = kInputSize - new_width - pad_left;
    const int pad_bottom = kInputSize - new_height - pad_top;

    cv::Mat letterboxed;
    cv::copyMakeBorder(
        resized,
        letterboxed,
        pad_top,
        pad_bottom,
        pad_left,
        pad_right,
        cv::BORDER_CONSTANT,
        cv::Scalar(kPadValue, kPadValue, kPadValue)
    );
    return letterboxed;
}

std::vector<float> to_rgb_nchw_blob(const cv::Mat& letterboxed_bgr) {
    cv::Mat rgb;
    cv::cvtColor(letterboxed_bgr, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    std::vector<float> blob(static_cast<size_t>(3) * kInputSize * kInputSize);
    std::vector<cv::Mat> channels(3);
    for (int c = 0; c < 3; ++c) {
        channels[c] = cv::Mat(
            kInputSize,
            kInputSize,
            CV_32F,
            blob.data() + static_cast<size_t>(c) * kInputSize * kInputSize
        );
    }
    cv::split(rgb, channels);
    return blob;
}

cv::Mat to_nv12(const cv::Mat& letterboxed_bgr) {
    cv::Mat i420;
    cv::cvtColor(letterboxed_bgr, i420, cv::COLOR_BGR2YUV_I420);

    const int width = letterboxed_bgr.cols;
    const int height = letterboxed_bgr.rows;
    const size_t y_size = static_cast<size_t>(width) * height;
    const size_t uv_size = y_size / 4;

    cv::Mat nv12(height * 3 / 2, width, CV_8UC1);
    std::copy(i420.ptr<uint8_t>(), i420.ptr<uint8_t>() + y_size, nv12.ptr<uint8_t>());

    const uint8_t* u = i420.ptr<uint8_t>() + y_size;
    const uint8_t* v = u + uv_size;
    uint8_t* uv = nv12.ptr<uint8_t>() + y_size;
    for (size_t i = 0; i < uv_size; ++i) {
        uv[2 * i] = u[i];
        uv[2 * i + 1] = v[i];
    }
    return nv12;
}

} // namespace autoaim_detector
