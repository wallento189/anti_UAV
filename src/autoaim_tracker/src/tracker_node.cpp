#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "autoaim_interfaces/msg/detection.hpp"
#include "autoaim_interfaces/msg/tracked_target.hpp"
#include "autoaim_tracker/tracker.hpp"

namespace TrackerNode {

namespace {
    const char* status_name(autoaim_tracker::Status status) {
        switch (status) {
            case autoaim_tracker::Status::kTracking:
                return "TRACKING";
            case autoaim_tracker::Status::kHolding:
                return "HOLDING";
            default:
                return "LOST";
        }
    }
} // namespace

class TrackerNode: public rclcpp::Node {
public:
    explicit TrackerNode(const rclcpp::NodeOptions& options);
    ~TrackerNode() override = default;

private:
    void get_parameters();
    void start();
    void on_detection(const autoaim_interfaces::msg::Detection::ConstSharedPtr& msg);
    void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void publish();
    void publish_debug_image(const cv::Mat& frame, const std_msgs::msg::Header& header);

    std::string detection_topic_;
    std::string camera_info_topic_;
    std::string target_topic_;
    std::string image_topic_;
    std::string debug_image_topic_;
    bool enable_debug_image_ = false;

    std::unique_ptr<autoaim_tracker::Tracker> tracker_;

    rclcpp::Subscription<autoaim_interfaces::msg::Detection>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<autoaim_interfaces::msg::TrackedTarget>::SharedPtr target_pub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_pub_;

    std_msgs::msg::Header last_header_;
    double principal_x_ = 0.0;
    double principal_y_ = 0.0;
};

// 进程内组合时启用零拷贝投递；独立进程运行时无对端，自动回退 DDS。
TrackerNode::TrackerNode(const rclcpp::NodeOptions& options):
    Node("autoaim_tracker", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
    get_parameters();
    start();
}

void TrackerNode::get_parameters() {
    detection_topic_ = declare_parameter<std::string>("detection_topic");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic");
    target_topic_ = declare_parameter<std::string>("target_topic");
    image_topic_ = declare_parameter<std::string>("image_topic");
    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic");
    enable_debug_image_ = declare_parameter<bool>("enable_debug_image");

    autoaim_tracker::TrackerConfig config;
    config.filter_alpha = declare_parameter<double>("filter_alpha");
    config.filter_beta = declare_parameter<double>("filter_beta");
    config.max_jump_px = declare_parameter<double>("max_jump_px");
    config.max_speed_px_s = declare_parameter<double>("max_speed_px_s");
    config.hold_frames = declare_parameter<int>("hold_frames");
    config.lost_frames = declare_parameter<int>("lost_frames");
    tracker_ = std::make_unique<autoaim_tracker::Tracker>(config);
}

void TrackerNode::start() {
    detection_sub_ = create_subscription<autoaim_interfaces::msg::Detection>(
        detection_topic_,
        rclcpp::QoS(10),
        std::bind(&TrackerNode::on_detection, this, std::placeholders::_1)
    );
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TrackerNode::on_camera_info, this, std::placeholders::_1)
    );
    target_pub_ =
        create_publisher<autoaim_interfaces::msg::TrackedTarget>(target_topic_, rclcpp::QoS(10));

    if (enable_debug_image_) {
        image_sub_ = image_transport::create_subscription(
            this,
            image_topic_,
            std::bind(&TrackerNode::on_image, this, std::placeholders::_1),
            "raw",
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        debug_pub_ = image_transport::create_publisher(
            this,
            debug_image_topic_,
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
    }

    last_header_.stamp = now();
    publish();
    const std::string debug_suffix =
        enable_debug_image_ ? ", debug image '" + debug_image_topic_ + "'" : "";
    RCLCPP_INFO(
        this->get_logger(),
        "autoaim_tracker started: subscribing '%s' and '%s', publishing '%s'%s",
        detection_topic_.c_str(),
        camera_info_topic_.c_str(),
        target_topic_.c_str(),
        debug_suffix.c_str()
    );
}

void TrackerNode::on_detection(const autoaim_interfaces::msg::Detection::ConstSharedPtr& msg) {
    autoaim_tracker::UpdateResult result;
    if (msg->detected) {
        if (!tracker_->has_intrinsics()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *get_clock(),
                1000,
                "Waiting for camera_info on '%s'",
                camera_info_topic_.c_str()
            );
            return;
        }
        autoaim_tracker::Measurement measurement;
        measurement.x = msg->x;
        measurement.y = msg->y;
        measurement.x1 = msg->x1;
        measurement.y1 = msg->y1;
        measurement.x2 = msg->x2;
        measurement.y2 = msg->y2;
        result = tracker_->update(measurement, rclcpp::Time(msg->header.stamp).seconds());
    } else {
        result = tracker_->miss();
    }

    if (result.observed) {
        last_header_ = msg->header;
    }
    if (result.observed || result.status_changed) {
        publish();
    }
}

void TrackerNode::on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    const bool had_intrinsics = tracker_->has_intrinsics();

    autoaim_tracker::CameraIntrinsics intrinsics;
    intrinsics.fx = msg->k[0];
    intrinsics.fy = msg->k[4];
    intrinsics.cx = msg->k[2];
    intrinsics.cy = msg->k[5];
    intrinsics.distortion.assign(msg->d.begin(), msg->d.end());
    intrinsics.valid = msg->width > 0 && msg->height > 0;
    tracker_->set_intrinsics(intrinsics);

    principal_x_ = msg->k[2];
    principal_y_ = msg->k[5];
    if (!had_intrinsics && tracker_->has_intrinsics()) {
        RCLCPP_INFO(
            this->get_logger(),
            "Camera intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            msg->k[0],
            msg->k[4],
            msg->k[2],
            msg->k[5]
        );
    }
}

void TrackerNode::publish() {
    const autoaim_tracker::TargetState state = tracker_->state();

    autoaim_interfaces::msg::TrackedTarget msg;
    msg.header = last_header_;
    msg.status = static_cast<std::uint8_t>(state.status);
    msg.x = static_cast<float>(state.x);
    msg.y = static_cast<float>(state.y);
    msg.x1 = static_cast<float>(state.x1);
    msg.y1 = static_cast<float>(state.y1);
    msg.x2 = static_cast<float>(state.x2);
    msg.y2 = static_cast<float>(state.y2);
    msg.yaw_error = static_cast<float>(state.yaw_error);
    msg.pitch_error = static_cast<float>(state.pitch_error);
    msg.yaw_rate = static_cast<float>(state.yaw_rate);
    msg.pitch_rate = static_cast<float>(state.pitch_rate);
    target_pub_->publish(msg);
}

void TrackerNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!tracker_->has_intrinsics()) {
        return;
    }
    try {
        const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        publish_debug_image(cv_ptr->image, msg->header);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *get_clock(),
            1000,
            "Failed to convert debug image: %s",
            e.what()
        );
    }
}

void TrackerNode::publish_debug_image(const cv::Mat& frame, const std_msgs::msg::Header& header) {
    const autoaim_tracker::TargetState state = tracker_->state();
    cv::Mat canvas = frame.clone();

    const cv::Point axis(cvRound(principal_x_), cvRound(principal_y_));
    cv::drawMarker(canvas, axis, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 20, 1);

    if (state.status != autoaim_tracker::Status::kLost) {
        const cv::Point p1(cvRound(state.x1), cvRound(state.y1));
        const cv::Point p2(cvRound(state.x2), cvRound(state.y2));
        const cv::Point center(cvRound(state.x), cvRound(state.y));
        const cv::Scalar green(0, 255, 0);
        const cv::Scalar yellow(0, 255, 255);
        cv::rectangle(canvas, p1, p2, green, 2);
        cv::circle(canvas, center, 3, green, -1);
        cv::arrowedLine(canvas, axis, center, yellow, 1);
    }

    char text[128];
    snprintf(
        text,
        sizeof(text),
        "%s yaw=%+.3f pitch=%+.3f",
        status_name(state.status),
        state.yaw_error,
        state.pitch_error
    );
    cv::putText(
        canvas,
        text,
        cv::Point(8, 20),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 0),
        1,
        cv::LINE_AA
    );
    debug_pub_.publish(cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg());
}

} // namespace TrackerNode

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TrackerNode::TrackerNode)
