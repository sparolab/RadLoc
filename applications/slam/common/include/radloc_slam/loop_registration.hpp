#pragma once
// Turning a loop proposal into a relative-pose constraint.
//
// The place lookup only says two keyframes look alike; this works out how they
// are actually related. RadLoc registers by phase correlation rather than a
// point-cloud fit: rotation comes from the log-polar of the FFT magnitude and
// translation from a second correlation on the de-rotated images, which needs
// no correspondences and no initial guess - the descriptor is azimuth-averaged
// and has no heading to offer as one.

#include <optional>

#include <gtsam/geometry/Pose3.h>

#include "radloc/phase_corr.hpp"
#include "radloc_slam/keyframe_store.hpp"

namespace radloc_slam {

struct RegistrationParams {
  radloc::PhaseCorrParams phase_corr{};
  // Smallest correlation peak to accept. Below this the two scans did not
  // really register and the pair is dropped.
  double min_response = 0.3;
};

// The transform from the current keyframe to the previous one, or nothing when
// the correlation is too weak to trust.
std::optional<gtsam::Pose3> registerLoop(const KeyframeStore& store, const LoopProposal& proposal,
                                         const RegistrationParams& params);

}  // namespace radloc_slam
