#ifndef AUTOAIM_TRACKER__TRACKER_HPP_
#define AUTOAIM_TRACKER__TRACKER_HPP_

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace autoaim_tracker {

enum class Status : std::uint8_t {
    kLost = 0,
    kTracking = 1,
    kHolding = 2,
};

struct TrackerConfig {
    double filter_alpha = 0.0;
    double filter_beta = 0.0;
    double max_jump_px = 0.0;
    double max_speed_px_s = 0.0;
    int hold_frames = 0;
    int lost_frames = 0;
};

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::vector<double> distortion;
    bool valid = false;
};

struct Measurement {
    double x = 0.0;
    double y = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

struct TargetState {
    Status status = Status::kLost;
    double stamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    double yaw_error = 0.0;
    double pitch_error = 0.0;
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
};

struct UpdateResult {
    // True when this call accepted a new observation.
    bool observed = false;
    // True when the status changed as a result of this call.
    bool status_changed = false;
};

// Single-target image-space tracker. Pure logic, no ROS dependency, so it can
// be exercised without a running graph.
class Tracker {
public:
    explicit Tracker(const TrackerConfig& config);

    void set_intrinsics(const CameraIntrinsics& intrinsics);
    bool has_intrinsics() const;

    // Feeds one inference frame that produced a target. A jump-gate rejection
    // counts as a missed frame.
    UpdateResult update(const Measurement& measurement, double stamp);

    // Feeds one inference frame that produced no target.
    UpdateResult miss();

    TargetState state() const;

private:
    Status status_at() const;
    UpdateResult result(bool observed);
    void reset(const Measurement& measurement, double stamp);
    bool project_angles(double x, double y, double& yaw, double& pitch) const;

    TrackerConfig config_;

    bool initialized_ = false;
    double last_stamp_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double box_x1_ = 0.0;
    double box_y1_ = 0.0;
    double box_x2_ = 0.0;
    double box_y2_ = 0.0;
    double box_center_x_ = 0.0;
    double box_center_y_ = 0.0;

    std::uint32_t miss_count_ = 0;
    bool has_interval_ = false;
    double average_dt_ = 0.0;

    Status last_status_ = Status::kLost;

    bool has_intrinsics_ = false;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    cv::Mat camera_matrix_;
    cv::Mat distortion_;
};

} // namespace autoaim_tracker

#endif // AUTOAIM_TRACKER__TRACKER_HPP_
