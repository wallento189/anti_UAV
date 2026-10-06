#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "autoaim_detector/infer_engine.hpp"
#include "autoaim_detector/postprocess.hpp"
#include "autoaim_detector/preprocess.hpp"
#include "autoaim_detector/types.hpp"
#include "autoaim_interfaces/msg/detection.hpp"

namespace DetectorNode {

class DetectorNode: public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions& options);
    ~DetectorNode() override;

private:
    void get_parameters();
    void start();
    void stop();
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void inference_loop();
    void publish_labeled_image(
        const cv::Mat& frame,
        const autoaim_detector::Detection& detection,
        const std_msgs::msg::Header& header
    );
    std::string resolve_model_path() const;
    autoaim_interfaces::msg::Detection make_message(
        const autoaim_detector::Detection& detection,
        const std_msgs::msg::Header& header
    ) const;

    std::string image_topic_;
    std::string detection_topic_;
    std::string labeled_image_topic_;
    std::string model_path_;
    bool enable_gpu_ = true;
    bool enable_bpu_ = false;
    bool enable_labeled_image_ = false;
    double confidence_thres_ = 0.25;

    std::unique_ptr<autoaim_detector::InferEngine> engine_;
    image_transport::Subscriber sub_;
    rclcpp::Publisher<autoaim_interfaces::msg::Detection>::SharedPtr pub_;
    image_transport::Publisher labeled_pub_;

    std::thread worker_;
    std::atomic<bool> running_ {false};
    std::mutex mutex_;
    std::condition_variable cv_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_;
};

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options): Node("autoaim_detector", options) {
    get_parameters();
    start();
}

DetectorNode::~DetectorNode() {
    stop();
}

void DetectorNode::get_parameters() {
    image_topic_ = declare_parameter<std::string>("image_topic");
    detection_topic_ = declare_parameter<std::string>("detection_topic");
    labeled_image_topic_ = declare_parameter<std::string>("labeled_image_topic");
    model_path_ = declare_parameter<std::string>("model_path");
    enable_gpu_ = declare_parameter<bool>("enable_gpu");
    enable_bpu_ = declare_parameter<bool>("enable_bpu");
    enable_labeled_image_ = declare_parameter<bool>("enable_labeled_image");
    confidence_thres_ = declare_parameter<double>("confidence_thres");
    model_path_ = resolve_model_path();
}

std::string DetectorNode::resolve_model_path() const {
    if (model_path_.empty()) {
        throw std::runtime_error(
            "model_path must not be empty; set it to a model file name relative to the "
            "autoaim_detector model directory"
        );
    }
    const std::string model_dir =
        ament_index_cpp::get_package_share_directory("autoaim_detector") + "/model";
    return (std::filesystem::path(model_dir) / model_path_).string();
}

void DetectorNode::start() {
    engine_ = autoaim_detector::create_engine(enable_gpu_, enable_bpu_, model_path_);
    pub_ = create_publisher<autoaim_interfaces::msg::Detection>(detection_topic_, rclcpp::QoS(10));
    if (enable_labeled_image_) {
        labeled_pub_ = image_transport::create_publisher(
            this,
            labeled_image_topic_,
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
    }
    sub_ = image_transport::create_subscription(
        this,
        image_topic_,
        std::bind(&DetectorNode::on_image, this, std::placeholders::_1),
        "raw",
        rclcpp::SensorDataQoS().get_rmw_qos_profile()
    );
    running_ = true;
    worker_ = std::thread(&DetectorNode::inference_loop, this);
    const std::string labeled_suffix =
        enable_labeled_image_ ? ", labeled image '" + labeled_image_topic_ + "'" : "";
    RCLCPP_INFO(
        this->get_logger(),
        "autoaim_detector started: backend=%s, model=%s, subscribing '%s', publishing '%s'%s",
        engine_->name(),
        model_path_.c_str(),
        image_topic_.c_str(),
        detection_topic_.c_str(),
        labeled_suffix.c_str()
    );
}

void DetectorNode::stop() {
    running_ = false;
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    engine_.reset();
}

void DetectorNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = msg;
    }
    cv_.notify_one();
}

void DetectorNode::inference_loop() {
    while (running_ && rclcpp::ok()) {
        sensor_msgs::msg::Image::ConstSharedPtr msg;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return latest_ != nullptr || !running_;
            });
            if (!running_) {
                break;
            }
            msg = latest_;
            latest_.reset();
        }
        if (!msg) {
            continue;
        }

        try {
            const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            const auto begin = std::chrono::steady_clock::now();
            const std::vector<autoaim_detector::RawTensor> outputs = engine_->infer(cv_ptr->image);
            const autoaim_detector::LetterboxTransform transform =
                autoaim_detector::make_letterbox_transform(cv_ptr->image.cols, cv_ptr->image.rows);
            const autoaim_detector::Detection detection = autoaim_detector::decode_detection(
                outputs,
                transform,
                static_cast<float>(confidence_thres_)
            );
            pub_->publish(make_message(detection, msg->header));
            if (enable_labeled_image_) {
                publish_labeled_image(cv_ptr->image, detection, msg->header);
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - begin
            )
                                     .count();
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *get_clock(),
                1000,
                "inference took %ld ms",
                elapsed
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *get_clock(),
                1000,
                "Inference failed: %s",
                e.what()
            );
        }
    }
}

void DetectorNode::publish_labeled_image(
    const cv::Mat& frame,
    const autoaim_detector::Detection& detection,
    const std_msgs::msg::Header& header
) {
    cv::Mat canvas = frame.clone();
    if (detection.detected) {
        const cv::Point p1(cvRound(detection.x1), cvRound(detection.y1));
        const cv::Point p2(cvRound(detection.x2), cvRound(detection.y2));
        const cv::Scalar green(0, 255, 0);
        cv::rectangle(canvas, p1, p2, green, 2);
        char text[32];
        snprintf(text, sizeof(text), "%.2f", detection.score);
        cv::putText(
            canvas,
            text,
            cv::Point(p1.x, std::max(p1.y - 5, 12)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            green,
            1,
            cv::LINE_AA
        );
    }
    labeled_pub_.publish(cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg());
}

autoaim_interfaces::msg::Detection DetectorNode::make_message(
    const autoaim_detector::Detection& detection,
    const std_msgs::msg::Header& header
) const {
    autoaim_interfaces::msg::Detection msg;
    msg.header = header;
    msg.detected = detection.detected;
    msg.score = detection.score;
    msg.x = detection.x;
    msg.y = detection.y;
    msg.x1 = detection.x1;
    msg.y1 = detection.y1;
    msg.x2 = detection.x2;
    msg.y2 = detection.y2;
    return msg;
}

} // namespace DetectorNode

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DetectorNode::DetectorNode)
