#ifndef AUTOAIM_DETECTOR__POSTPROCESS_HPP_
#define AUTOAIM_DETECTOR__POSTPROCESS_HPP_

#include <vector>

#include "autoaim_detector/types.hpp"

namespace autoaim_detector {

Detection decode_detection(
    const std::vector<RawTensor>& outputs,
    const LetterboxTransform& transform,
    float confidence_thres
);

} // namespace autoaim_detector

#endif // AUTOAIM_DETECTOR__POSTPROCESS_HPP_
