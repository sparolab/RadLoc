#pragma once
// Loading and layout normalisation for scanning-radar polar images.
//
// Two on-disk layouts are in circulation for the same data:
//   * Oxford form        : (num_azimuth) x (11 + num_range), the first eleven
//                          columns carrying timestamp/encoder metadata
//   * preprocessed form  : (num_range) x (num_azimuth), metadata already removed
//
// Both are normalised here to a single in-memory layout: rows = range bins,
// cols = azimuths.

#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace radloc {

// Number of metadata columns prepended to each azimuth in the Oxford format.
constexpr int kOxfordHeaderCols = 11;

// A polar scan with rows = range bins, cols = azimuths.
struct PolarScan {
  cv::Mat data;  // CV_64F

  int numRange() const { return data.rows; }
  int numAzimuth() const { return data.cols; }
  bool empty() const { return data.empty(); }
};

// Normalise an arbitrary polar image into (range, azimuth) layout.
//
// The reference Python pipeline (Referee/util/descriptor/feature_extractor.py,
// rotate_image with mode "hori") transposes so that the azimuth axis is the
// shorter one, then transposes again. The net effect is simply: the longer axis
// becomes rows. `strip_header` is false by default because the descriptors
// shipped in /storage/Datasets/ReFeree were generated with the eleven metadata
// columns left in place.
inline PolarScan toPolarScan(const cv::Mat& raw, bool strip_header = false) {
  if (raw.empty()) throw std::runtime_error("radloc: empty radar image");

  cv::Mat range_major;  // rows = range (the longer axis)
  if (raw.rows < raw.cols) {
    cv::transpose(raw, range_major);
  } else {
    range_major = raw;
  }

  if (strip_header) {
    if (range_major.rows <= kOxfordHeaderCols)
      throw std::runtime_error("radloc: image too small to strip header");
    range_major = range_major.rowRange(kOxfordHeaderCols, range_major.rows);
  }

  PolarScan scan;
  range_major.convertTo(scan.data, CV_64F);
  return scan;
}

// A single detection in the sensor frame, metres.
struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

// Points above `threshold`, one per (range, azimuth) cell that clears it.
// `range_resolution` is metres per range bin; `range_offset` skips the leading
// bins, which carry metadata in the Oxford layout.
inline std::vector<Point2D> toCartesianPoints(const PolarScan& scan,
                                              double range_resolution,
                                              double threshold,
                                              int range_offset = 0) {
  std::vector<Point2D> points;
  const int R = scan.numRange(), A = scan.numAzimuth();
  for (int a = 0; a < A; ++a) {
    const double angle = 2.0 * CV_PI * a / A;
    const double ca = std::cos(angle), sa = std::sin(angle);
    for (int r = range_offset; r < R; ++r) {
      if (scan.data.at<double>(r, a) <= threshold) continue;
      const double range = (r - range_offset) * range_resolution;
      points.push_back({range * ca, range * sa});
    }
  }
  return points;
}

inline PolarScan loadPolarImage(const std::string& path, bool strip_header = false) {
  cv::Mat raw = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (raw.empty()) throw std::runtime_error("radloc: cannot read " + path);
  return toPolarScan(raw, strip_header);
}

}  // namespace radloc
