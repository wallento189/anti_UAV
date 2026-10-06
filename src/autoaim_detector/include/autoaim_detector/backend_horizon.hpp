#ifndef AUTOAIM_DETECTOR__BACKEND_HORIZON_HPP_
#define AUTOAIM_DETECTOR__BACKEND_HORIZON_HPP_

#include <memory>

#include "autoaim_detector/infer_engine.hpp"

namespace autoaim_detector {

std::unique_ptr<InferEngine> make_horizon_engine();

} // namespace autoaim_detector

#endif // AUTOAIM_DETECTOR__BACKEND_HORIZON_HPP_
