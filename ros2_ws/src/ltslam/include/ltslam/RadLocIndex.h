#pragma once
// Inter-session loop proposal backed by the RadLoc descriptor.
//
// Drop-in replacement for the Scan Context path: both answer the same question,
// "which node of the target session does this source node look like?". Scan
// Context also returned a yaw estimate, which LT-SLAM never used - the relative
// pose comes from phase correlation in doICPVirtualRelative - so nothing is
// lost by returning an index alone.
//
// A RadLoc descriptor is one value per range band, averaged over azimuth, so it
// is rotation invariant by construction. That removes the ring-key plus
// column-shift search Scan Context needs, leaving a plain two-stage lookup.

#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "radloc/descriptor.hpp"
#include "radloc/retrieval.hpp"

class RadLocIndex {
 public:
  // Nodes must be added in node-index order: the position in the database is
  // the node index LT-SLAM will use.
  void add(const radloc::Descriptor& desc) {
    if (descriptors_.empty()) db_ = radloc::PlaceDatabase(params_);
    db_.add(desc);
    descriptors_.push_back(desc);
  }

  void build() {
    if (!descriptors_.empty()) db_.build();
  }

  std::size_t size() const { return descriptors_.size(); }

  void setRetrievalParams(const radloc::RetrievalParams& p) { params_ = p; }
  void setAcceptThreshold(double t) { accept_threshold_ = t; }

  // Target-session node index matching `query`, or -1 when the best candidate
  // is not close enough.
  int detectLoopClosureIDBetweenSession(const radloc::Descriptor& query) const {
    if (descriptors_.empty()) return -1;
    const auto matches = db_.query(query);
    if (matches.empty()) return -1;
    if (matches.front().distance > accept_threshold_) return -1;
    return matches.front().index;
  }

  const radloc::Descriptor& descriptor(int i) const {
    return descriptors_.at(static_cast<std::size_t>(i));
  }

 private:
  radloc::RetrievalParams params_{};
  radloc::PlaceDatabase db_{params_};
  std::vector<radloc::Descriptor> descriptors_;
  // Mean absolute difference between band values, in raw return units.
  //
  // On MulRan KAIST_03, with a 10 m loop radius, true loops score p50 0.34 and
  // p90 0.50 while false ones score p10 0.72 and p50 0.97, so the populations
  // separate around 0.6. That default is a starting point, not a constant of
  // nature: it is in return units, so it moves with the sensor and the scene,
  // and is worth re-measuring on a new dataset. Sized for
  // FineMetric::MeanAbsolute; recalibrate if FineMetric::Huber is selected.
  double accept_threshold_ = 0.6;
};
