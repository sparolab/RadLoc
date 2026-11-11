#pragma once
// 3-DoF registration of two radar scans by phase correlation (PhaRaO style).
//
//   1. rasterise each scan's points into a cartesian bird's-eye image
//   2. recover rotation from the log-polar of the FFT magnitude, which is
//      shift-invariant, so a rotation between the scans appears as a pure
//      translation along the angular axis
//   3. de-rotate the source and recover translation from a second phase
//      correlation on the cartesian images
//
// Ported from the standalone implementation in Distributed_Radar_SLAM
// (ltslam/PhaseCorrReg.h). Depends on OpenCV only - no FFTW, no PCL.

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

namespace radloc {

struct PhaseCorrParams {
  // Metres per bird's-eye pixel. The default keeps a 672 px image at about
  // +-200 m, matching the radar's usable range.
  double resolution = 0.59612233;
  int bev_size = 672;
  int blur_kernel = 5;
  double blur_sigma = 1.0;

  // The FFT magnitude is centrally symmetric, so the log-polar stage recovers
  // rotation only modulo 180 degrees. When set, both candidates are de-rotated
  // and the one with the stronger translation peak is kept. The original
  // implementation did not do this and could return a heading flipped by pi.
  bool resolve_180_ambiguity = true;
};

// Pose of the source scan expressed in the target scan's frame: rotate a target
// point by `theta` and add (x, y) to land on the corresponding source point.
// For a loop closure between keyframe i (target) and j (source) this is the
// relative measurement the factor wants.
struct Transform2D {
  double x = 0.0;      // metres
  double y = 0.0;      // metres
  double theta = 0.0;  // radians
  double response = 0.0;  // translation-stage phase-correlation peak
  bool valid = false;
};

inline void fftShift(cv::Mat& m) {
  const int cx = m.cols / 2, cy = m.rows / 2;
  cv::Mat q0(m, cv::Rect(0, 0, cx, cy)), q1(m, cv::Rect(cx, 0, cx, cy));
  cv::Mat q2(m, cv::Rect(0, cy, cx, cy)), q3(m, cv::Rect(cx, cy, cx, cy));
  cv::Mat tmp;
  q0.copyTo(tmp); q3.copyTo(q0); tmp.copyTo(q3);
  q1.copyTo(tmp); q2.copyTo(q1); tmp.copyTo(q2);
}

// Rasterise any range of points exposing .x and .y into a bird's-eye image.
template <typename PointRange>
inline cv::Mat rasterize(const PointRange& points, const PhaseCorrParams& p = {}) {
  cv::Mat img = cv::Mat::zeros(p.bev_size, p.bev_size, CV_32F);
  const double c = p.bev_size / 2.0;
  for (const auto& pt : points) {
    const int px = static_cast<int>(std::lround(pt.x / p.resolution + c));
    const int py = static_cast<int>(std::lround(pt.y / p.resolution + c));
    if (px >= 0 && px < p.bev_size && py >= 0 && py < p.bev_size)
      img.at<float>(py, px) += 1.0f;
  }
  if (p.blur_kernel > 1)
    cv::GaussianBlur(img, img, cv::Size(p.blur_kernel, p.blur_kernel), p.blur_sigma);
  return img;
}

// Log-polar of the shifted FFT magnitude: rotation becomes a translation.
inline cv::Mat logPolarSpectrum(const cv::Mat& cart) {
  cv::Mat padded;
  const int m = cv::getOptimalDFTSize(cart.rows), n = cv::getOptimalDFTSize(cart.cols);
  cv::copyMakeBorder(cart, padded, 0, m - cart.rows, 0, n - cart.cols,
                     cv::BORDER_CONSTANT, cv::Scalar::all(0));

  cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
  cv::Mat complex;
  cv::merge(planes, 2, complex);
  cv::dft(complex, complex);
  cv::split(complex, planes);
  cv::magnitude(planes[0], planes[1], planes[0]);

  cv::Mat mag = planes[0];
  mag += cv::Scalar::all(1.0);
  cv::log(mag, mag);
  fftShift(mag);

  cv::Mat log_polar;
  const cv::Point2f center(mag.cols / 2.0f, mag.rows / 2.0f);
  const double scale = mag.cols / std::log(static_cast<double>(mag.rows) / 2.0);
  cv::logPolar(mag, log_polar, center, scale,
               cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
  return log_polar;
}

// Registers two bird's-eye images.
inline Transform2D estimateTransform(const cv::Mat& target, const cv::Mat& source,
                                     const PhaseCorrParams& p = {}) {
  Transform2D out;
  if (target.empty() || source.empty()) return out;

  const cv::Mat pol_t = logPolarSpectrum(target);
  const cv::Mat pol_s = logPolarSpectrum(source);

  double rotation_response = 0.0;
  const cv::Point2d peak_r =
      cv::phaseCorrelate(pol_t, pol_s, cv::noArray(), &rotation_response);
  const double rotation_deg = peak_r.y * (360.0 / pol_t.rows);

  // Try the recovered rotation, and its 180 degree twin when asked, keeping
  // whichever de-rotation correlates better against the target.
  const cv::Point2f centre(source.cols / 2.0f, source.rows / 2.0f);
  auto translation_for = [&](double deg, double* response) {
    cv::Mat derotated;
    cv::warpAffine(source, derotated, cv::getRotationMatrix2D(centre, deg, 1.0),
                   source.size());
    return cv::phaseCorrelate(target, derotated, cv::noArray(), response);
  };

  double best_response = 0.0;
  cv::Point2d best_peak = translation_for(rotation_deg, &best_response);
  double best_deg = rotation_deg;

  if (p.resolve_180_ambiguity) {
    double flipped_response = 0.0;
    const cv::Point2d flipped_peak = translation_for(rotation_deg + 180.0, &flipped_response);
    if (flipped_response > best_response) {
      best_response = flipped_response;
      best_peak = flipped_peak;
      best_deg = rotation_deg + 180.0;
    }
  }

  // Wrap to (-pi, pi].
  const double theta = std::atan2(std::sin(best_deg * CV_PI / 180.0),
                                  std::cos(best_deg * CV_PI / 180.0));

  // phaseCorrelate returns the shift mapping the first image onto the second in
  // pixels, and the rasteriser scales metres to pixels positively, so the peak
  // converts straight to metres. That shift is measured against the *de-rotated*
  // source, though, so it lives in the rotated frame; rotating it back by theta
  // expresses it in the target frame.
  const double shift_x = best_peak.x * p.resolution;
  const double shift_y = best_peak.y * p.resolution;
  const double c = std::cos(theta), s = std::sin(theta);
  out.x = c * shift_x - s * shift_y;
  out.y = s * shift_x + c * shift_y;
  out.theta = theta;
  out.response = best_response;
  out.valid = true;
  return out;
}

// Convenience overload: rasterise then register.
template <typename PointRange>
inline Transform2D registerScans(const PointRange& target, const PointRange& source,
                                 const PhaseCorrParams& p = {}) {
  if (std::empty(target) || std::empty(source)) return {};
  return estimateTransform(rasterize(target, p), rasterize(source, p), p);
}

}  // namespace radloc
