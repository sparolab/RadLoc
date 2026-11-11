#pragma once
// Everything the pose-graph node knows about the keyframes it has seen.
//
// The keyframes, their clouds and the descriptor database live behind one lock
// and are added together, so they cannot fall out of step.

#include <mutex>
#include <optional>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radloc/descriptor.hpp"
#include "radloc/retrieval.hpp"
#include "radloc_slam/pose_types.hpp"

namespace radloc_slam {

using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;

struct LoopProposal {
  int previous_index = -1;
  int current_index = -1;
  // RadLoc's descriptor is azimuth-averaged and carries no heading, so unlike a
  // sector-wise descriptor it offers no yaw to seed registration with. Phase
  // correlation recovers the rotation itself.
};

class KeyframeStore {
 public:
  explicit KeyframeStore(radloc::RetrievalParams params = {}) : place_db_(params) {}

  // Must be called before the first keyframe is added.
  void setRetrievalParams(const radloc::RetrievalParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    place_db_ = radloc::PlaceDatabase(params);
  }

  // How far the vehicle must have travelled since a keyframe before it may be
  // proposed as a loop. A distance, not a count of keyframes: excluding a fixed
  // number excludes whatever distance the spacing happens to give.
  void setMinLoopTravel(double metres) {
    std::lock_guard<std::mutex> lock(mutex_);
    min_loop_travel_ = metres;
  }

  int add(const Pose6D& pose, double stamp, Cloud::Ptr cloud, radloc::Descriptor descriptor) {
    std::lock_guard<std::mutex> lock(mutex_);
    travelled_.push_back(poses_.empty() ? 0.0
                                        : travelled_.back() + translationBetween(poses_.back(), pose));
    poses_.push_back(pose);
    optimised_poses_.push_back(pose);
    stamps_.push_back(stamp);
    clouds_.push_back(std::move(cloud));
    descriptors_.push_back(descriptor);
    place_db_.add(std::move(descriptor));
    return static_cast<int>(poses_.size()) - 1;
  }

  std::size_t size() const { std::lock_guard<std::mutex> l(mutex_); return poses_.size(); }
  Pose6D pose(int i) const { std::lock_guard<std::mutex> l(mutex_); return poses_.at(i); }
  Pose6D optimisedPose(int i) const { std::lock_guard<std::mutex> l(mutex_); return optimised_poses_.at(i); }
  Cloud::Ptr cloud(int i) const { std::lock_guard<std::mutex> l(mutex_); return clouds_.at(i); }
  double stamp(int i) const { std::lock_guard<std::mutex> l(mutex_); return stamps_.at(i); }
  radloc::Descriptor descriptor(int i) const { std::lock_guard<std::mutex> l(mutex_); return descriptors_.at(i); }
  std::vector<Pose6D> poses() const { std::lock_guard<std::mutex> l(mutex_); return poses_; }
  std::vector<Pose6D> optimisedPoses() const { std::lock_guard<std::mutex> l(mutex_); return optimised_poses_; }

  void setOptimisedPose(int i, const Pose6D& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    optimised_poses_.at(static_cast<std::size_t>(i)) = pose;
  }

  // The newest keyframe against the ones far enough back along the path.
  std::optional<LoopProposal> proposeLoop() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_.empty()) return std::nullopt;
    const int current = static_cast<int>(poses_.size()) - 1;

    int eligible = current;
    while (eligible > 0 && travelled_[current] - travelled_[eligible - 1] < min_loop_travel_)
      --eligible;
    if (eligible <= 0) return std::nullopt;

    const auto matches = place_db_.query(descriptors_.at(current), eligible);
    if (matches.empty()) return std::nullopt;
    return LoopProposal{matches.front().index, current};
  }

 private:
  mutable std::mutex mutex_;
  std::vector<Pose6D> poses_, optimised_poses_;
  std::vector<double> stamps_;
  std::vector<double> travelled_;   // cumulative path length, one per keyframe
  std::vector<Cloud::Ptr> clouds_;
  std::vector<radloc::Descriptor> descriptors_;
  radloc::PlaceDatabase place_db_;
  double min_loop_travel_ = 100.0;  // metres
};

}  // namespace radloc_slam
