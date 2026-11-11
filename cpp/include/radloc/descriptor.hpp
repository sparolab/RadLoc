#pragma once
// RadLoc place descriptor.
//
// A scan is reduced to one value per range band: the mean return over every
// azimuth in that band. Azimuths whose mean return stands out against their
// neighbours are rejected first by a 1-D cell-averaging CFAR, which removes
// receiver saturation streaks.
//
// Earlier versions stored 255 - intensity instead. Since the band value is a
// mean, that is only an affine flip of what is stored here, and both retrieval
// stages are invariant to it: the fine stage compares differences and the
// coarse stage is a Euclidean nearest-neighbour search, so the shared offset
// cancels in each. See fromInverted().
//
// The descriptor is *unweighted*. Range weighting - counting distant bands for
// more - is a property of how two places are compared, not of the place itself,
// so it lives in retrieval.hpp and is applied once when the coarse search key
// is built. See Descriptor::rangeWeighted().

#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "radloc/polar_image.hpp"

namespace radloc {

struct DescriptorParams {
  int patch_range = 84;     // range bins per band
  int patch_azimuth = 4;    // azimuths per cell
  int cfar_guard = 2;       // guard cells either side of the cell under test
  int cfar_reference = 84;  // reference cells either side
  double cfar_pfa = 0.2;    // probability of false alarm
};

// Mean return per range band, nearest band first.
struct Descriptor {
  std::vector<double> bands;

  std::size_t size() const { return bands.size(); }
  bool empty() const { return bands.empty(); }

  // Band b scaled by (b + 1), optionally truncated to the leading `dims`.
  // This is the form used as the coarse search key.
  std::vector<double> rangeWeighted(std::size_t dims = 0) const {
    const std::size_t n = (dims == 0 || dims > bands.size()) ? bands.size() : dims;
    std::vector<double> out(n);
    for (std::size_t b = 0; b < n; ++b) out[b] = bands[b] * static_cast<double>(b + 1);
    return out;
  }
};

// Cell-averaging CFAR over a 1-D signal. Returns one flag per sample, true where
// the sample exceeds the scaled average of its two reference windows.
inline std::vector<bool> caCfar1d(const std::vector<double>& signal, int guard,
                                  int reference, double pfa) {
  const int n = static_cast<int>(signal.size());
  if (n == 0) return {};
  if (reference <= 0) throw std::invalid_argument("radloc: reference cells must be > 0");

  const int window = guard + reference;
  const double threshold_factor = reference * (std::pow(pfa, -1.0 / reference) - 1.0);

  // Edge-padded access, mirroring numpy's pad(..., mode="edge").
  auto padded = [&](int k) {
    int i = k - window;
    if (i < 0) i = 0;
    if (i > n - 1) i = n - 1;
    return signal[static_cast<std::size_t>(i)];
  };

  // numpy convolves with a reversed kernel, so the left kernel (weights on its
  // first `reference` taps) reads the samples *after* the cell under test and
  // the right kernel those before it. Reproduced literally to stay bit-exact
  // with the reference implementation.
  const int ksize = 2 * window + 1;
  std::vector<bool> detections(static_cast<std::size_t>(n), false);
  for (int i = 0; i < n; ++i) {
    double left = 0.0, right = 0.0;
    for (int j = 0; j < ksize; ++j) {
      const int k = ksize - 1 - j;  // reversed kernel index
      if (k < reference) left += padded(i + j);
      if (k >= ksize - reference) right += padded(i + j);
    }
    const double noise = 0.5 * (left / reference + right / reference);
    detections[static_cast<std::size_t>(i)] =
        signal[static_cast<std::size_t>(i)] > noise * threshold_factor;
  }
  return detections;
}

// Mean return per azimuth, i.e. averaged over every range bin.
inline std::vector<double> azimuthMean(const PolarScan& scan) {
  const int R = scan.numRange(), A = scan.numAzimuth();
  std::vector<double> mean(static_cast<std::size_t>(A), 0.0);
  for (int a = 0; a < A; ++a) {
    double acc = 0.0;
    for (int r = 0; r < R; ++r) acc += scan.data.at<double>(r, a);
    mean[static_cast<std::size_t>(a)] = acc / R;
  }
  return mean;
}

// Length is numRange() / patch_range.
inline Descriptor computeDescriptor(const PolarScan& scan,
                                    const DescriptorParams& p = {}) {
  if (scan.empty()) throw std::runtime_error("radloc: empty scan");
  const int R = scan.numRange(), A = scan.numAzimuth();
  const int num_bands = R / p.patch_range;
  const int num_cells = A / p.patch_azimuth;
  if (num_bands <= 0 || num_cells <= 0)
    throw std::runtime_error("radloc: scan smaller than one patch");

  const std::vector<bool> masked =
      caCfar1d(azimuthMean(scan), p.cfar_guard, p.cfar_reference, p.cfar_pfa);

  Descriptor desc;
  desc.bands.assign(static_cast<std::size_t>(num_bands), 0.0);

  for (int b = 0; b < num_bands; ++b) {
    const int r0 = b * p.patch_range;

    double band_acc = 0.0;  // sum of per-cell mean return
    int band_cells = 0;     // cells that were not entirely masked
    for (int c = 0; c < num_cells; ++c) {
      const int a0 = c * p.patch_azimuth;

      double cell_acc = 0.0;
      int cell_n = 0;
      for (int a = a0; a < a0 + p.patch_azimuth; ++a) {
        if (masked[static_cast<std::size_t>(a)]) continue;  // saturated azimuth
        for (int r = r0; r < r0 + p.patch_range; ++r) {
          cell_acc += scan.data.at<double>(r, a);
          ++cell_n;
        }
      }
      if (cell_n == 0) continue;  // whole cell masked out
      band_acc += cell_acc / cell_n;
      ++band_cells;
    }

    desc.bands[static_cast<std::size_t>(b)] =
        (band_cells > 0) ? band_acc / band_cells : 0.0;
  }
  return desc;
}

// Converts a descriptor archived in the inverted form, (255 - mean) * (b + 1),
// into the range-weighted form produced by Descriptor::rangeWeighted(). The two
// rank candidates identically; this exists so archived descriptors can be mixed
// with freshly computed ones.
inline std::vector<double> fromInverted(const std::vector<double>& inverted) {
  std::vector<double> out(inverted.size());
  for (std::size_t b = 0; b < inverted.size(); ++b)
    out[b] = 255.0 * static_cast<double>(b + 1) - inverted[b];
  return out;
}

// Plain-text serialisation, whitespace separated on one line.
inline void writeRadLocDescriptorText(const std::string& path, const Descriptor& d) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("radloc: cannot write " + path);
  file.precision(9);
  for (std::size_t i = 0; i < d.bands.size(); ++i)
    file << d.bands[i] << (i + 1 == d.bands.size() ? '\n' : ' ');
}

inline Descriptor readRadLocDescriptorText(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("radloc: cannot open " + path);
  Descriptor d;
  double v;
  while (file >> v) d.bands.push_back(v);
  if (d.empty()) throw std::runtime_error("radloc: empty descriptor " + path);
  return d;
}

inline Descriptor computeDescriptor(const std::string& image_path,
                                    const DescriptorParams& p = {}) {
  return computeDescriptor(loadPolarImage(image_path), p);
}

}  // namespace radloc
