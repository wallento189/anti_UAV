#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace CameraNode {

class CameraNode: public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;

private:
    void get_parameters();
    void start();
    void stop();
    bool open_device();
    void apply_settings();
    void capture_loop();

    static std::string fourcc_to_string(int64_t code);

    static constexpr int kMaxConsecutiveFailures = 100;

    std::string device_;
    std::string fourcc_;
    unsigned int width_ = 640;
    unsigned int height_ = 480;
    double fps_ = 120.0;
    int crop_x_ = 0;
    int crop_y_ = 0;
    int crop_width_ = 640;
    int crop_height_ = 480;
    std::string frame_id_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string camera_info_url_;
    int buffer_size_ = 1;

    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    std::thread worker_;
    std::atomic<bool> running_ {false};
};

// 进程内组合时启用零拷贝投递；独立进程运行时无对端，自动回退 DDS。
CameraNode::CameraNode(const rclcpp::NodeOptions& options)
: Node("autoaim_camera", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
    get_parameters();
    start();
}

CameraNode::~CameraNode() {
    stop();
}

void CameraNode::get_parameters() {
    device_ = declare_parameter<std::string>("device");
    fourcc_ = declare_parameter<std::string>("fourcc");
    width_ = static_cast<unsigned int>(declare_parameter<int>("width"));
    height_ = static_cast<unsigned int>(declare_parameter<int>("height"));
    fps_ = declare_parameter<double>("fps");
    crop_x_ = declare_parameter<int>("crop_x");
    crop_y_ = declare_parameter<int>("crop_y");
    crop_width_ = declare_parameter<int>("crop_width");
    crop_height_ = declare_parameter<int>("crop_height");
    frame_id_ = declare_parameter<std::string>("frame_id");
    image_topic_ = declare_parameter<std::string>("image_topic");
    buffer_size_ = declare_parameter<int>("buffer_size");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic");
    camera_info_url_ = declare_parameter<std::string>("camera_info_url");

    if (fourcc_.size() != 4) {
        throw std::runtime_error("fourcc must be exactly 4 characters, got '" + fourcc_ + "'");
    }
    if (crop_x_ < 0 || crop_y_ < 0 || crop_width_ <= 0 || crop_height_ <= 0) {
        throw std::runtime_error(
            "crop window must satisfy crop_x/crop_y >= 0 and crop_width/crop_height > 0"
        );
    }
    if (crop_x_ + crop_width_ > static_cast<int>(width_)
        || crop_y_ + crop_height_ > static_cast<int>(height_))
    {
        throw std::runtime_error(
            "crop window [" + std::to_string(crop_x_) + "," + std::to_string(crop_y_) + " "
            + std::to_string(crop_width_) + "x" + std::to_string(crop_height_)
            + "] exceeds the requested capture resolution " + std::to_string(width_) + "x"
            + std::to_string(height_)
        );
    }
}

void CameraNode::start() {
    pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this,
        "autoaim_camera",
        camera_info_url_
    );
    if (!camera_info_manager_->isCalibrated()) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Failed to load camera calibration from '%s'",
            camera_info_url_.c_str()
        );
        throw std::runtime_error("failed to load camera calibration");
    }
    camera_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, rclcpp::SensorDataQoS());
    if (!open_device()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open device '%s'", device_.c_str());
        throw std::runtime_error("failed to open camera device");
    }
    const sensor_msgs::msg::CameraInfo info = camera_info_manager_->getCameraInfo();
    const int actual_width = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actual_height = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    if ((actual_width > 0 && crop_x_ + crop_width_ > actual_width)
        || (actual_height > 0 && crop_y_ + crop_height_ > actual_height))
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "Crop window [%d,%d %dx%d] exceeds the actual capture resolution %dx%d",
            crop_x_,
            crop_y_,
            crop_width_,
            crop_height_,
            actual_width,
            actual_height
        );
        throw std::runtime_error("crop window exceeds capture resolution");
    }
    if (info.width != static_cast<unsigned int>(crop_width_)
        || info.height != static_cast<unsigned int>(crop_height_))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Calibration is for %ux%u but the published crop is %dx%d; recalibrate on the "
            "cropped topic and replace '%s'",
            info.width,
            info.height,
            crop_width_,
            crop_height_,
            camera_info_url_.c_str()
        );
    }
    running_ = true;
    worker_ = std::thread(&CameraNode::capture_loop, this);
    RCLCPP_INFO(
        this->get_logger(),
        "autoaim_camera started, publishing to '%s' and '%s'",
        image_topic_.c_str(),
        camera_info_topic_.c_str()
    );
    RCLCPP_INFO(
        this->get_logger(),
        "Publishing crop [%d,%d %dx%d] of the %dx%d capture",
        crop_x_,
        crop_y_,
        crop_width_,
        crop_height_,
        width_,
        height_
    );
}

void CameraNode::stop() {
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
    if (capture_.isOpened()) {
        capture_.release();
        RCLCPP_INFO(this->get_logger(), "Device '%s' released", device_.c_str());
    }
}

bool CameraNode::open_device() {
    try {
        capture_.release();
        if (!capture_.open(device_, cv::CAP_V4L2)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open device '%s' (busy, not present, or no permission)",
                device_.c_str()
            );
            return false;
        }
        apply_settings();
        return true;
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "OpenCV exception while opening '%s': %s",
            device_.c_str(),
            e.what()
        );
        return false;
    }
}

std::string CameraNode::fourcc_to_string(int64_t code) {
    if (code < 0) {
        return "unknown";
    }
    std::string s;
    s.resize(4);
    s[0] = static_cast<char>(code & 0xFF);
    s[1] = static_cast<char>((code >> 8) & 0xFF);
    s[2] = static_cast<char>((code >> 16) & 0xFF);
    s[3] = static_cast<char>((code >> 24) & 0xFF);
    return s;
}

void CameraNode::apply_settings() {
    const int requested_fourcc =
        cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]);
    capture_.set(cv::CAP_PROP_FOURCC, static_cast<double>(requested_fourcc));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    capture_.set(cv::CAP_PROP_FPS, fps_);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(buffer_size_));

    const unsigned int actual_width =
        static_cast<unsigned int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    const unsigned int actual_height =
        static_cast<unsigned int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
    const std::string actual_fourcc =
        fourcc_to_string(static_cast<int64_t>(capture_.get(cv::CAP_PROP_FOURCC)));

    RCLCPP_INFO(
        this->get_logger(),
        "Device '%s' configured: %ux%u @ %.1f fps, fourcc=%s (requested %ux%u @ %.1f fps, fourcc=%s)",
        device_.c_str(),
        actual_width,
        actual_height,
        actual_fps,
        actual_fourcc.c_str(),
        width_,
        height_,
        fps_,
        fourcc_.c_str()
    );

    if (actual_width != width_ || actual_height != height_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Resolution mismatch: requested %ux%u, got %ux%u",
            width_,
            height_,
            actual_width,
            actual_height
        );
    }
    if (actual_fourcc != "unknown" && actual_fourcc != fourcc_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Pixel format mismatch: requested %s, got %s",
            fourcc_.c_str(),
            actual_fourcc.c_str()
        );
    }
    if (actual_fps > 0.0 && std::fabs(actual_fps - fps_) > 1.0) {
        RCLCPP_WARN(
            this->get_logger(),
            "FPS mismatch: requested %.1f, driver reports %.1f",
            fps_,
            actual_fps
        );
    }
}

void CameraNode::capture_loop() {
    int consecutive_failures = 0;
    try {
        while (running_ && rclcpp::ok()) {
            cv::Mat frame;
            try {
                if (!capture_.read(frame)) {
                    frame.release();
                }
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *get_clock(),
                    1000,
                    "Capture error: %s",
                    e.what()
                );
                frame.release();
            }

            if (frame.empty()) {
                if (++consecutive_failures >= kMaxConsecutiveFailures) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "%d consecutive capture failures, reopening '%s'",
                        consecutive_failures,
                        device_.c_str()
                    );
                    if (open_device()) {
                        consecutive_failures = 0;
                    } else {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                continue;
            }
            consecutive_failures = 0;

            if (frame.cols < crop_x_ + crop_width_ || frame.rows < crop_y_ + crop_height_) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *get_clock(),
                    1000,
                    "Captured frame %dx%d is smaller than crop window [%d,%d %dx%d]",
                    frame.cols,
                    frame.rows,
                    crop_x_,
                    crop_y_,
                    crop_width_,
                    crop_height_
                );
                continue;
            }
            // ROI 是原帧上的视图，不复制像素；发布时再按行拷入消息。
            const cv::Mat cropped = frame(cv::Rect(crop_x_, crop_y_, crop_width_, crop_height_));

            try {
                const size_t row_bytes = static_cast<size_t>(cropped.cols) * cropped.elemSize();
                // unique_ptr 发布是进程内零拷贝的前提；rclcpp 无 shared_ptr 重载，const& 会多一次整帧拷贝。
                auto msg = std::make_unique<sensor_msgs::msg::Image>();
                msg->header.stamp = now();
                msg->header.frame_id = frame_id_;
                msg->height = static_cast<sensor_msgs::msg::Image::_height_type>(cropped.rows);
                msg->width = static_cast<sensor_msgs::msg::Image::_width_type>(cropped.cols);
                msg->encoding = sensor_msgs::image_encodings::BGR8;
                msg->is_bigendian = false;
                msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(row_bytes);
                msg->data.resize(row_bytes * static_cast<size_t>(cropped.rows));
                for (int row = 0; row < cropped.rows; ++row) {
                    std::memcpy(
                        msg->data.data() + static_cast<size_t>(row) * row_bytes,
                        cropped.ptr(row),
                        row_bytes
                    );
                }
                // 标定消息与图像同 stamp，下游可按帧配对。
                sensor_msgs::msg::CameraInfo info = camera_info_manager_->getCameraInfo();
                info.header = msg->header;
                camera_info_pub_->publish(info);
                pub_->publish(std::move(msg));
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *get_clock(),
                    1000,
                    "Frame conversion error: %s",
                    e.what()
                );
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Capture loop terminated unexpectedly: %s", e.what());
        running_ = false;
    }
}

} // namespace CameraNode

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode::CameraNode)
