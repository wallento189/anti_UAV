#include "autoaim_tracker/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace autoaim_tracker {

namespace {
    // Guards the velocity update against duplicate or out-of-order stamps.
    inline constexpr double kMinDt = 1e-3;
    // A gap this many times longer than the running frame interval means the
    // stream was interrupted, so the old track must not gate the new frame.
    inline constexpr double kResetDtFactor = 5.0;
    // Weight of the previous interval estimate when smoothing frame timing.
    inline constexpr double kIntervalSmoothing = 0.8;
} // namespace

Tracker::Tracker(const TrackerConfig& config): config_(config) {
    if (config_.hold_frames <= 0 || config_.lost_frames <= config_.hold_frames) {
        throw std::invalid_argument("hold_frames must be > 0 and lost_frames must be > hold_frames"
        );
    }
}

void Tracker::set_intrinsics(const CameraIntrinsics& intrinsics) {
    has_intrinsics_ = intrinsics.valid && intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
    if (!has_intrinsics_) {
        return;
    }
    fx_ = intrinsics.fx;
    fy_ = intrinsics.fy;
    cx_ = intrinsics.cx;
    cy_ = intrinsics.cy;
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    if (intrinsics.distortion.empty()) {
        distortion_ = cv::Mat();
    } else {
        distortion_ = cv::Mat(1, static_cast<int>(intrinsics.distortion.size()), CV_64F);
        for (std::size_t i = 0; i < intrinsics.distortion.size(); ++i) {
            distortion_.at<double>(0, static_cast<int>(i)) = intrinsics.distortion[i];
        }
    }
}

bool Tracker::has_intrinsics() const {
    return has_intrinsics_;
}

UpdateResult Tracker::update(const Measurement& measurement, double stamp) {
    const bool interrupted = has_interval_ && stamp - last_stamp_ > kResetDtFactor * average_dt_;
    if (!initialized_ || stamp <= last_stamp_ || interrupted
        || static_cast<int>(miss_count_) >= config_.lost_frames)
    {
        reset(measurement, stamp);
        return result(true);
    }

    const double dt = std::max(stamp - last_stamp_, kMinDt);
    const double predicted_x = x_ + vx_ * dt;
    const double predicted_y = y_ + vy_ * dt;
    const double residual_x = measurement.x - predicted_x;
    const double residual_y = measurement.y - predicted_y;

    // While the track is still alive a large jump looks like a false positive.
    if (std::hypot(residual_x, residual_y) > config_.max_jump_px) {
        ++miss_count_;
        return result(false);
    }

    x_ = predicted_x + config_.filter_alpha * residual_x;
    y_ = predicted_y + config_.filter_alpha * residual_y;
    vx_ = std::clamp(
        vx_ + config_.filter_beta * residual_x / dt,
        -config_.max_speed_px_s,
        config_.max_speed_px_s
    );
    vy_ = std::clamp(
        vy_ + config_.filter_beta * residual_y / dt,
        -config_.max_speed_px_s,
        config_.max_speed_px_s
    );

    if (!has_interval_) {
        average_dt_ = stamp - last_stamp_;
        has_interval_ = true;
    } else {
        average_dt_ =
            kIntervalSmoothing * average_dt_ + (1.0 - kIntervalSmoothing) * (stamp - last_stamp_);
    }

    miss_count_ = 0;
    last_stamp_ = stamp;
    box_x1_ = measurement.x1;
    box_y1_ = measurement.y1;
    box_x2_ = measurement.x2;
    box_y2_ = measurement.y2;
    box_center_x_ = measurement.x;
    box_center_y_ = measurement.y;
    return result(true);
}

UpdateResult Tracker::miss() {
    if (initialized_) {
        ++miss_count_;
    }
    return result(false);
}

TargetState Tracker::state() const {
    TargetState out;
    out.status = status_at();
    out.stamp = last_stamp_;
    if (out.status == Status::kLost) {
        return out;
    }

    out.x = x_;
    out.y = y_;
    out.x1 = box_x1_;
    out.y1 = box_y1_;
    out.x2 = box_x2_;
    out.y2 = box_y2_;
    if (project_angles(x_, y_, out.yaw_error, out.pitch_error)) {
        out.yaw_rate = -vx_ / fx_;
        out.pitch_rate = vy_ / fy_;
    }
    return out;
}

Status Tracker::status_at() const {
    if (!initialized_) {
        return Status::kLost;
    }
    if (static_cast<int>(miss_count_) >= config_.lost_frames) {
        return Status::kLost;
    }
    if (static_cast<int>(miss_count_) >= config_.hold_frames) {
        return Status::kHolding;
    }
    return Status::kTracking;
}

UpdateResult Tracker::result(bool observed) {
    const Status status = status_at();
    UpdateResult out;
    out.observed = observed;
    out.status_changed = status != last_status_;
    last_status_ = status;
    return out;
}

void Tracker::reset(const Measurement& measurement, double stamp) {
    initialized_ = true;
    has_interval_ = false;
    last_stamp_ = stamp;
    x_ = measurement.x;
    y_ = measurement.y;
    vx_ = 0.0;
    vy_ = 0.0;
    box_x1_ = measurement.x1;
    box_y1_ = measurement.y1;
    box_x2_ = measurement.x2;
    box_y2_ = measurement.y2;
    box_center_x_ = measurement.x;
    box_center_y_ = measurement.y;
    miss_count_ = 0;
}

bool Tracker::project_angles(double x, double y, double& yaw, double& pitch) const {
    if (!has_intrinsics_) {
        return false;
    }
    std::vector<cv::Point2d> source;
    source.emplace_back(x, y);
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(source, normalized, camera_matrix_, distortion_);
    // Image x grows right while yaw is positive to the left; image y grows
    // down and pitch is positive downwards.
    yaw = -std::atan(normalized[0].x);
    pitch = std::atan(normalized[0].y);
    return true;
}

} // namespace autoaim_tracker
