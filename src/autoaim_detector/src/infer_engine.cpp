#include "autoaim_detector/infer_engine.hpp"

#include <stdexcept>

#if defined(AUTOAIM_WITH_OPENVINO)
    #include "autoaim_detector/backend_openvino.hpp"
#endif
#if defined(AUTOAIM_WITH_HORIZON)
    #include "autoaim_detector/backend_horizon.hpp"
#endif

namespace autoaim_detector {

std::unique_ptr<InferEngine>
create_engine(bool enable_gpu, bool enable_bpu, const std::string& model_path) {
    if (enable_bpu && enable_gpu) {
        throw std::runtime_error("enable_bpu and enable_gpu are mutually exclusive");
    }

    if (enable_bpu) {
#if defined(AUTOAIM_WITH_HORIZON)
        std::unique_ptr<InferEngine> engine = make_horizon_engine();
        engine->load(model_path);
        return engine;
#else
        (void)model_path;
        throw std::runtime_error(
            "BPU backend not compiled in (Horizon SDK not found at build time); "
            "rebuild with -DAUTOAIM_WITH_HORIZON=ON or set enable_bpu:=false"
        );
#endif
    }

#if defined(AUTOAIM_WITH_OPENVINO)
    std::unique_ptr<InferEngine> engine = make_openvino_engine(enable_gpu);
    engine->load(model_path);
    return engine;
#else
    (void)enable_gpu;
    (void)model_path;
    throw std::runtime_error(
        "ONNX backend not compiled in (OpenVINO not found at build time); "
        "rebuild with -DAUTOAIM_WITH_OPENVINO=ON"
    );
#endif
}

} // namespace autoaim_detector
