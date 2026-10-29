#ifndef AUTOAIM_TRACKER__ATTITUDE_BUFFER_HPP_
#define AUTOAIM_TRACKER__ATTITUDE_BUFFER_HPP_

#include <cstddef>
#include <deque>

namespace autoaim_tracker {

struct AttitudeSample {
    double stamp = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
};

// Bounded history of gimbal attitudes, keyed by the time the visual side
// received them.
//
// Pairing an image-space error with the attitude from the same instant is what
// turns the error into a latency-independent absolute target angle. Using the
// newest attitude instead would add the gimbal's own motion during the image
// latency to the setpoint, which reintroduces the oscillation that the error
// semantics suffers from.
class AttitudeBuffer {
public:
    explicit AttitudeBuffer(std::size_t capacity);

    // Returns false and ignores the sample when it is not finite or not
    // strictly newer than the last one, so duplicated or reordered feedback
    // frames cannot break the ordering the search relies on.
    bool push(double stamp, double yaw, double pitch);

    // Picks the sample closest in time to `stamp`, but only when the gap is
    // within `max_gap_s` on either side. Returns false otherwise.
    bool nearest(double stamp, double max_gap_s, AttitudeSample* out) const;

    std::size_t size() const;
    bool empty() const;

private:
    std::size_t capacity_;
    std::deque<AttitudeSample> samples_;
};

// Absolute direction of the target in the gimbal frame: the attitude observed
// at the same instant plus the angular offset from the camera optical axis.
// No wrapping and no two-axis coupling correction are applied.
void compose_target(
    const AttitudeSample& attitude,
    double yaw_error,
    double pitch_error,
    double* yaw_target,
    double* pitch_target
);

} // namespace autoaim_tracker

#endif // AUTOAIM_TRACKER__ATTITUDE_BUFFER_HPP_
