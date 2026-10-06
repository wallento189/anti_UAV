#ifndef AUTOAIM_DETECTOR__PREPROCESS_HPP_
#define AUTOAIM_DETECTOR__PREPROCESS_HPP_

#include <vector>

#include <opencv2/core.hpp>

#include "autoaim_detector/types.hpp"

namespace autoaim_detector {

LetterboxTransform make_letterbox_transform(int width, int height);

cv::Mat letterbox_image(const cv::Mat& bgr, const LetterboxTransform& transform);

std::vector<float> to_rgb_nchw_blob(const cv::Mat& letterboxed_bgr);

cv::Mat to_nv12(const cv::Mat& letterboxed_bgr);

} // namespace autoaim_detector

#endif // AUTOAIM_DETECTOR__PREPROCESS_HPP_
