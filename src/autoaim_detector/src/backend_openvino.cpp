#include "autoaim_detector/backend_openvino.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <openvino/openvino.hpp>
#include <opencv2/core.hpp>

#include "autoaim_detector/preprocess.hpp"

namespace autoaim_detector {

namespace {

    struct OutputSlot {
        std::string name;
        int stride = 0;
        bool is_box = false;
    };

    class OpenVinoEngine final: public InferEngine {
    public:
        explicit OpenVinoEngine(bool enable_gpu): device_(enable_gpu ? "GPU" : "CPU") {}

        void load(const std::string& model_path) override {
            model_path_ = model_path;
            try {
                compiled_model_ = core_.compile_model(model_path, device_);
            } catch (const std::exception& e) {
                throw std::runtime_error(
                    "OpenVINO compile_model failed on device '" + device_ + "' for '" + model_path
                    + "': " + e.what() + " (try enable_gpu:=false to use the CPU device)"
                );
            }
            request_ = compiled_model_.create_infer_request();

            for (const ov::Output<const ov::Node>& output: compiled_model_.outputs()) {
                const ov::Shape shape = output.get_shape();
                if (shape.size() != 4 || shape[0] != 1) {
                    throw std::runtime_error("unexpected output rank/shape from OpenVINO model");
                }
                const int height = static_cast<int>(shape[1]);
                const int width = static_cast<int>(shape[2]);
                const int channels = static_cast<int>(shape[3]);
                if (height != width || (channels != 1 && channels != 4 * kRegMax)) {
                    throw std::runtime_error("unexpected output shape from OpenVINO model");
                }
                const int stride = kInputSize / height;
                if (stride != 4 && stride != 8 && stride != 16 && stride != 32) {
                    throw std::runtime_error("unexpected output stride from OpenVINO model");
                }
                slots_.push_back({output.get_any_name(), stride, channels == 4 * kRegMax});
            }
            if (slots_.empty() || slots_.size() % 2 != 0) {
                throw std::runtime_error(
                    "expected an even number of cls/box outputs from OpenVINO model, got "
                    + std::to_string(slots_.size())
                );
            }
        }

        std::vector<RawTensor> infer(const cv::Mat& bgr) override {
            const LetterboxTransform transform = make_letterbox_transform(bgr.cols, bgr.rows);
            const cv::Mat letterboxed = letterbox_image(bgr, transform);
            const std::vector<float> blob = to_rgb_nchw_blob(letterboxed);

            ov::Tensor input_tensor(
                ov::element::f32,
                ov::Shape {
                    1,
                    3,
                    static_cast<size_t>(kInputSize),
                    static_cast<size_t>(kInputSize),
                }
            );
            std::copy(blob.begin(), blob.end(), input_tensor.data<float>());
            request_.set_input_tensor(input_tensor);
            request_.infer();

            std::vector<RawTensor> outputs;
            outputs.reserve(slots_.size());
            for (const OutputSlot& slot: slots_) {
                outputs.push_back(read_output(slot));
            }
            return outputs;
        }

        const char* name() const override {
            return device_ == "GPU" ? "openvino-gpu" : "openvino-cpu";
        }

    private:
        RawTensor read_output(const OutputSlot& slot) {
            const ov::Tensor tensor = request_.get_tensor(slot.name);
            const ov::Shape shape = tensor.get_shape();

            RawTensor out;
            out.height = static_cast<int>(shape[1]);
            out.width = static_cast<int>(shape[2]);
            out.channels = static_cast<int>(shape[3]);
            out.stride = slot.stride;
            out.is_box = slot.is_box;

            const size_t count = tensor.get_size();
            out.data.resize(count);
            if (tensor.get_element_type() == ov::element::f32) {
                const float* src = tensor.data<float>();
                std::copy(src, src + count, out.data.begin());
            } else if (tensor.get_element_type() == ov::element::f16) {
                const ov::float16* src = tensor.data<ov::float16>();
                std::transform(src, src + count, out.data.begin(), [](ov::float16 value) {
                    return static_cast<float>(value);
                });
            } else {
                throw std::runtime_error("unsupported OpenVINO output element type");
            }
            return out;
        }

        std::string device_;
        std::string model_path_;
        ov::Core core_;
        ov::CompiledModel compiled_model_;
        ov::InferRequest request_;
        std::vector<OutputSlot> slots_;
    };

} // namespace

std::unique_ptr<InferEngine> make_openvino_engine(bool enable_gpu) {
    return std::make_unique<OpenVinoEngine>(enable_gpu);
}

} // namespace autoaim_detector
