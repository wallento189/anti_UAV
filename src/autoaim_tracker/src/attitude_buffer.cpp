#include "autoaim_tracker/attitude_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace autoaim_tracker {

AttitudeBuffer::AttitudeBuffer(std::size_t capacity):
    capacity_(std::max<std::size_t>(capacity, 1)) {}

bool AttitudeBuffer::push(double stamp, double yaw, double pitch) {
    if (!std::isfinite(stamp) || !std::isfinite(yaw) || !std::isfinite(pitch)) {
        return false;
    }
    if (!samples_.empty() && stamp <= samples_.back().stamp) {
        return false;
    }

    samples_.push_back(AttitudeSample {stamp, yaw, pitch});
    while (samples_.size() > capacity_) {
        samples_.pop_front();
    }
    return true;
}

bool AttitudeBuffer::nearest(double stamp, double max_gap_s, AttitudeSample* out) const {
    if (out == nullptr || samples_.empty() || !std::isfinite(stamp) || !(max_gap_s > 0.0)) {
        return false;
    }

    const auto after = std::lower_bound(
        samples_.begin(),
        samples_.end(),
        stamp,
        [](const AttitudeSample& sample, double value) { return sample.stamp < value; }
    );

    const AttitudeSample* best = nullptr;
    double best_gap = 0.0;
    const auto consider = [&best, &best_gap, stamp](const AttitudeSample& sample) {
        const double gap = std::abs(sample.stamp - stamp);
        if (best == nullptr || gap < best_gap) {
            best = &sample;
            best_gap = gap;
        }
    };
    if (after != samples_.end()) {
        consider(*after);
    }
    if (after != samples_.begin()) {
        consider(*std::prev(after));
    }
    if (best == nullptr || best_gap > max_gap_s) {
        return false;
    }

    *out = *best;
    return true;
}

std::size_t AttitudeBuffer::size() const {
    return samples_.size();
}

bool AttitudeBuffer::empty() const {
    return samples_.empty();
}

void compose_target(
    const AttitudeSample& attitude,
    double yaw_error,
    double pitch_error,
    double* yaw_target,
    double* pitch_target
) {
    if (yaw_target == nullptr || pitch_target == nullptr) {
        return;
    }
    *yaw_target = attitude.yaw + yaw_error;
    *pitch_target = attitude.pitch + pitch_error;
}

} // namespace autoaim_tracker
