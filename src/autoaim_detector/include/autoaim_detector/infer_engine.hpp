#ifndef AUTOAIM_DETECTOR__INFER_ENGINE_HPP_
#define AUTOAIM_DETECTOR__INFER_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "autoaim_detector/types.hpp"

namespace autoaim_detector {

class InferEngine {
public:
    virtual ~InferEngine() = default;

    virtual void load(const std::string& model_path) = 0;
    virtual std::vector<RawTensor> infer(const cv::Mat& bgr) = 0;
    virtual const char* name() const = 0;
};

std::unique_ptr<InferEngine>
create_engine(bool enable_gpu, bool enable_bpu, const std::string& model_path);

} // namespace autoaim_detector

#endif // AUTOAIM_DETECTOR__INFER_ENGINE_HPP_
