#ifndef AUTOAIM_DETECTOR__BACKEND_OPENVINO_HPP_
#define AUTOAIM_DETECTOR__BACKEND_OPENVINO_HPP_

#include <memory>

#include "autoaim_detector/infer_engine.hpp"

namespace autoaim_detector {

std::unique_ptr<InferEngine> make_openvino_engine(bool enable_gpu);

} // namespace autoaim_detector

#endif // AUTOAIM_DETECTOR__BACKEND_OPENVINO_HPP_
