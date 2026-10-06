#include "autoaim_detector/backend_horizon.hpp"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

#include "autoaim_detector/preprocess.hpp"

namespace autoaim_detector {

namespace {

    struct OutputSlot {
        int index = 0;
        int stride = 0;
        bool is_box = false;
    };

    float dequantize(int32_t value, const hbDNNTensorProperties& properties, int channel) {
        if (properties.quantiType == SCALE) {
            if (properties.scale.scaleLen <= 0 || properties.scale.scaleData == nullptr) {
                return static_cast<float>(value);
            }
            const int index = properties.scale.scaleLen == 1 ? 0 : channel;
            return static_cast<float>(value) * properties.scale.scaleData[index];
        }
        if (properties.quantiType == SHIFT) {
            if (properties.shift.shiftLen <= 0 || properties.shift.shiftData == nullptr) {
                return static_cast<float>(value);
            }
            const int index = properties.shift.shiftLen == 1 ? 0 : channel;
            return static_cast<float>(value)
                / static_cast<float>(1u << properties.shift.shiftData[index]);
        }
        return static_cast<float>(value);
    }

    class HorizonEngine final: public InferEngine {
    public:
        ~HorizonEngine() override {
            release();
        }

        void load(const std::string& model_path) override {
            const char* model_file = model_path.c_str();
            if (hbDNNInitializeFromFiles(&packed_handle_, &model_file, 1) != 0) {
                throw std::runtime_error(
                    "hbDNNInitializeFromFiles failed for '" + model_path + "'"
                );
            }

            const char** model_names = nullptr;
            int model_count = 0;
            if (hbDNNGetModelNameList(&model_names, &model_count, packed_handle_) != 0
                || model_count < 1) {
                throw std::runtime_error("hbDNNGetModelNameList failed");
            }
            if (hbDNNGetModelHandle(&dnn_handle_, packed_handle_, model_names[0]) != 0) {
                throw std::runtime_error("hbDNNGetModelHandle failed");
            }

            int input_count = 0;
            if (hbDNNGetInputCount(&input_count, dnn_handle_) != 0 || input_count != 1) {
                throw std::runtime_error("Horizon model must have exactly 1 input");
            }
            hbDNNGetInputTensorProperties(&input_properties_, dnn_handle_, 0);
            if (input_properties_.tensorType != HB_DNN_IMG_TYPE_NV12
                || input_properties_.tensorLayout != HB_DNN_LAYOUT_NCHW)
            {
                throw std::runtime_error(
                    "Horizon model input must be NV12/NCHW, got tensorType="
                    + std::to_string(input_properties_.tensorType)
                    + " layout=" + std::to_string(input_properties_.tensorLayout)
                );
            }
            if (input_properties_.validShape.numDimensions != 4
                || input_properties_.validShape.dimensionSize[0] != 1
                || input_properties_.validShape.dimensionSize[1] != 3
                || input_properties_.validShape.dimensionSize[2] != kInputSize
                || input_properties_.validShape.dimensionSize[3] != kInputSize)
            {
                throw std::runtime_error("Horizon model input validShape must be (1,3,640,640)");
            }

            int output_count = 0;
            if (hbDNNGetOutputCount(&output_count, dnn_handle_) != 0 || output_count < 2
                || output_count % 2 != 0)
            {
                throw std::runtime_error("Horizon model must have an even number of cls/box outputs"
                );
            }
            outputs_.resize(output_count);
            for (int i = 0; i < output_count; ++i) {
                hbDNNTensorProperties properties;
                hbDNNGetOutputTensorProperties(&properties, dnn_handle_, i);
                if (properties.validShape.numDimensions != 4) {
                    throw std::runtime_error("Horizon output must be 4-dimensional");
                }
                if (properties.quantiType != SCALE && properties.quantiType != SHIFT) {
                    throw std::runtime_error("Horizon output must be quantized (SCALE/SHIFT)");
                }
                const int height = properties.validShape.dimensionSize[1];
                const int channels = properties.validShape.dimensionSize[3];
                if (channels != 1 && channels != 4 * kRegMax) {
                    throw std::runtime_error("unexpected Horizon output channels");
                }
                const int stride = kInputSize / height;
                if (stride != 4 && stride != 8 && stride != 16 && stride != 32) {
                    throw std::runtime_error("unexpected Horizon output stride");
                }
                slots_.push_back({i, stride, channels == 4 * kRegMax});
            }

            input_.properties = input_properties_;
            hbSysAllocCachedMem(&input_.sysMem[0], input_properties_.alignedByteSize);
            for (int i = 0; i < output_count; ++i) {
                hbDNNGetOutputTensorProperties(&outputs_[i].properties, dnn_handle_, i);
                hbSysAllocCachedMem(&outputs_[i].sysMem[0], outputs_[i].properties.alignedByteSize);
            }
        }

        std::vector<RawTensor> infer(const cv::Mat& bgr) override {
            const LetterboxTransform transform = make_letterbox_transform(bgr.cols, bgr.rows);
            const cv::Mat letterboxed = letterbox_image(bgr, transform);
            const cv::Mat nv12 = to_nv12(letterboxed);

            if (nv12.total() > static_cast<size_t>(input_properties_.alignedByteSize)) {
                throw std::runtime_error("NV12 buffer larger than the Horizon input tensor");
            }
            std::memcpy(input_.sysMem[0].virAddr, nv12.data, nv12.total());
            hbSysFlushMem(&input_.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

            hbDNNInferCtrlParam control;
            HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
            hbDNNTensor* output_ptr = outputs_.data();
            if (hbDNNInfer(&task_handle_, &output_ptr, &input_, dnn_handle_, &control) != 0) {
                throw std::runtime_error("hbDNNInfer failed");
            }
            hbDNNWaitTaskDone(task_handle_, 0);

            std::vector<RawTensor> result;
            result.reserve(slots_.size());
            for (const OutputSlot& slot: slots_) {
                hbDNNTensor& tensor = outputs_[slot.index];
                hbSysFlushMem(&tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
                result.push_back(read_output(tensor, slot));
            }

            hbDNNReleaseTask(task_handle_);
            task_handle_ = nullptr;
            return result;
        }

        const char* name() const override {
            return "horizon-bpu";
        }

    private:
        static RawTensor read_output(const hbDNNTensor& tensor, const OutputSlot& slot) {
            const hbDNNTensorProperties& properties = tensor.properties;
            const int height = properties.validShape.dimensionSize[1];
            const int width = properties.validShape.dimensionSize[2];
            const int channels = properties.validShape.dimensionSize[3];
            const int aligned_width = properties.alignedShape.dimensionSize[2];
            const int aligned_channels = properties.alignedShape.dimensionSize[3];
            const int32_t* src = reinterpret_cast<const int32_t*>(tensor.sysMem[0].virAddr);

            RawTensor out;
            out.height = height;
            out.width = width;
            out.channels = channels;
            out.stride = slot.stride;
            out.is_box = slot.is_box;
            out.data.resize(static_cast<size_t>(height) * width * channels);
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    for (int c = 0; c < channels; ++c) {
                        const int32_t value =
                            src[(static_cast<size_t>(y) * aligned_width + x) * aligned_channels
                                + c];
                        out.data[(static_cast<size_t>(y) * width + x) * channels + c] =
                            dequantize(value, properties, c);
                    }
                }
            }
            return out;
        }

        void release() {
            if (task_handle_ != nullptr) {
                hbDNNReleaseTask(task_handle_);
                task_handle_ = nullptr;
            }
            for (hbDNNTensor& output: outputs_) {
                if (output.sysMem[0].virAddr != nullptr) {
                    hbSysFreeMem(&output.sysMem[0]);
                    output.sysMem[0].virAddr = nullptr;
                }
            }
            if (input_.sysMem[0].virAddr != nullptr) {
                hbSysFreeMem(&input_.sysMem[0]);
                input_.sysMem[0].virAddr = nullptr;
            }
            if (packed_handle_ != nullptr) {
                hbDNNRelease(packed_handle_);
                packed_handle_ = nullptr;
            }
        }

        hbPackedDNNHandle_t packed_handle_ = nullptr;
        hbDNNHandle_t dnn_handle_ = nullptr;
        hbDNNTensorProperties input_properties_ {};
        hbDNNTensor input_ {};
        std::vector<hbDNNTensor> outputs_;
        std::vector<OutputSlot> slots_;
        hbDNNTaskHandle_t task_handle_ = nullptr;
    };

} // namespace

std::unique_ptr<InferEngine> make_horizon_engine() {
    return std::make_unique<HorizonEngine>();
}

} // namespace autoaim_detector
