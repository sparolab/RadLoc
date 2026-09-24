#pragma once
// Two-stage place retrieval.
//
//   coarse : Euclidean nearest neighbours over the leading `coarse_dims` of the
//            range-weighted descriptor, via a k-d tree. Weighting the bands
//            makes distant structure dominate the shortlist.
//   fine   : the shortlist is re-ranked on the full, unweighted descriptor, so
//            that near and far bands count equally once candidates are few.
//
// Weighting is therefore applied exactly once, when the coarse key is built.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

#include "radloc/descriptor.hpp"
#include "radloc/third_party/nanoflann.hpp"
#include "radloc/third_party/KDTreeVectorOfVectorsAdaptor.h"

namespace radloc {

// How the shortlist is scored.
enum class FineMetric {
  // Mean absolute difference over all bands. This is what the reference
  // implementation computes: its huber_loss() overwrites the delta it is passed
  // and tests `abs_diff <= 0`, so every band takes the L1 branch.
  MeanAbsolute,
  // Huber as it was presumably intended: quadratic below `huber_delta`, linear
  // above. Changes the ranking, so results are not comparable with archived runs.
  Huber,
};

struct RetrievalParams {
  // Bands used for the coarse k-d tree search. The reference implementation
  // used 20 between sessions and 8 within one session.
  int coarse_dims = 20;
  int top_k = 10;
  FineMetric fine_metric = FineMetric::MeanAbsolute;
  double huber_delta = 50.0;  // only read when fine_metric == Huber
};

struct Match {
  int index = -1;
  double distance = 0.0;  // fine-stage score, smaller is closer
};

inline double fineDistance(const std::vector<double>& a, const std::vector<double>& b,
                           const RetrievalParams& p) {
  if (a.size() != b.size()) throw std::invalid_argument("radloc: descriptor length mismatch");
  if (a.empty()) return 0.0;

  double acc = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = std::abs(a[i] - b[i]);
    if (p.fine_metric == FineMetric::Huber && d <= p.huber_delta) {
      acc += 0.5 * d * d;
    } else if (p.fine_metric == FineMetric::Huber) {
      acc += p.huber_delta * (d - 0.5 * p.huber_delta);
    } else {
      acc += d;
    }
  }
  return acc / static_cast<double>(a.size());
}

// A searchable set of places. Descriptors are stored unweighted; the coarse keys
// are derived once, when the index is built.
//
// The k-d tree adaptor holds a *reference* to the coarse key vector, so the tree
// cannot travel with a copy or a move of this object - the reference would point
// at the original's storage. Copy and move therefore drop the index, and query()
// rebuilds it on demand. Without that, putting a database inside a container
// that reallocates, or inside a struct that gets moved, is a use-after-move that
// only shows up as a crash at query time.
class PlaceDatabase {
 public:
  explicit PlaceDatabase(RetrievalParams params = {}) : params_(params) {}

  PlaceDatabase(const PlaceDatabase& other)
      : params_(other.params_), descriptors_(other.descriptors_), coarse_(other.coarse_) {}
  PlaceDatabase(PlaceDatabase&& other) noexcept
      : params_(other.params_),
        descriptors_(std::move(other.descriptors_)),
        coarse_(std::move(other.coarse_)) {
    other.index_.reset();
  }
  PlaceDatabase& operator=(const PlaceDatabase& other) {
    if (this != &other) {
      params_ = other.params_;
      descriptors_ = other.descriptors_;
      coarse_ = other.coarse_;
      index_.reset();
    }
    return *this;
  }
  PlaceDatabase& operator=(PlaceDatabase&& other) noexcept {
    if (this != &other) {
      params_ = other.params_;
      descriptors_ = std::move(other.descriptors_);
      coarse_ = std::move(other.coarse_);
      index_.reset();
      other.index_.reset();
    }
    return *this;
  }

  void reserve(std::size_t n) { descriptors_.reserve(n); coarse_.reserve(n); }

  // Returns the index assigned to this place. Invalidates the index; call
  // build() before querying.
  int add(const Descriptor& d) {
    if (params_.coarse_dims <= 0) throw std::invalid_argument("radloc: coarse_dims must be > 0");
    if (d.size() < static_cast<std::size_t>(params_.coarse_dims))
      throw std::invalid_argument("radloc: descriptor shorter than coarse_dims");
    descriptors_.push_back(d.bands);
    coarse_.push_back(d.rangeWeighted(static_cast<std::size_t>(params_.coarse_dims)));
    index_.reset();
    return static_cast<int>(descriptors_.size()) - 1;
  }

  std::size_t size() const { return descriptors_.size(); }
  const std::vector<double>& descriptor(int i) const { return descriptors_.at(static_cast<std::size_t>(i)); }

  // Optional: query() builds on demand anyway. Useful to pay the cost up front.
  void build() const {
    if (coarse_.empty()) { index_.reset(); return; }
    index_ = std::make_unique<KdTree>(params_.coarse_dims, coarse_, 10 /* leaf size */);
  }

  // Nearest places to `query`, best first. Only the first `max_index` entries
  // are eligible when `max_index` >= 0, which is how a single session excludes
  // its own recent neighbours.
  // `limit`, when non-negative, caps how many leading entries may be matched.
  // The caller usually knows better than the database how much of the recent
  // past to exclude - how far the vehicle has travelled, say.
  std::vector<Match> query(const Descriptor& q, int limit = -1) const {
    if (descriptors_.empty()) return {};
    if (limit == 0) return {};
    if (!index_) build();
    const std::vector<double> key =
        q.rangeWeighted(static_cast<std::size_t>(params_.coarse_dims));

    const std::size_t searchable =
        (limit > 0) ? std::min<std::size_t>(static_cast<std::size_t>(limit), descriptors_.size())
                    : descriptors_.size();
    const std::size_t k =
        std::min<std::size_t>(static_cast<std::size_t>(std::max(params_.top_k, 1)), searchable);
    std::vector<std::size_t> ids(k);
    std::vector<double> sq_dists(k);
    nanoflann::KNNResultSet<double> results(k);
    results.init(ids.data(), sq_dists.data());
    index_->index->findNeighbors(results, key.data(), nanoflann::SearchParams(10));

    std::vector<Match> matches;
    matches.reserve(results.size());
    for (std::size_t i = 0; i < results.size(); ++i) {
      if (limit > 0 && ids[i] >= static_cast<std::size_t>(limit)) continue;
      matches.push_back({static_cast<int>(ids[i]),
                         fineDistance(q.bands, descriptors_[ids[i]], params_)});
    }
    std::sort(matches.begin(), matches.end(),
              [](const Match& a, const Match& b) { return a.distance < b.distance; });
    return matches;
  }

  const RetrievalParams& params() const { return params_; }

 private:
  using KdTree = KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<double>>, double>;

  RetrievalParams params_;
  std::vector<std::vector<double>> descriptors_;  // unweighted, full length
  std::vector<std::vector<double>> coarse_;       // range-weighted, truncated
  mutable std::unique_ptr<KdTree> index_;
};

}  // namespace radloc
