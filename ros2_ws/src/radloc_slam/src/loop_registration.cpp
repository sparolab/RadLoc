#include "radloc_slam/loop_registration.hpp"

namespace radloc_slam {

std::optional<gtsam::Pose3> registerLoop(const KeyframeStore& store, const LoopProposal& proposal,
                                         const RegistrationParams& params) {
  const Cloud::Ptr target = store.cloud(proposal.previous_index);
  const Cloud::Ptr source = store.cloud(proposal.current_index);
  if (!target || !source || target->empty() || source->empty()) return std::nullopt;

  const radloc::Transform2D rel =
      radloc::registerScans(target->points, source->points, params.phase_corr);
  if (!rel.valid || rel.response < params.min_response) return std::nullopt;

  // Transform2D is the pose of the source in the target's frame, which is what
  // a between-factor from previous to current wants.
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, rel.theta),
                      gtsam::Point3(rel.x, rel.y, 0.0));
}

}  // namespace radloc_slam
