// Self-test for phase-correlation registration: take a real scan, apply a known
// rigid transform to its points, and measure how well it is recovered.
//
//   test_phase_corr <polar_image.png>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "radloc/phase_corr.hpp"
#include "radloc/polar_image.hpp"

namespace {

std::vector<radloc::Point2D> transformPoints(const std::vector<radloc::Point2D>& in,
                                             double dx, double dy, double dtheta) {
  const double c = std::cos(dtheta), s = std::sin(dtheta);
  std::vector<radloc::Point2D> out;
  out.reserve(in.size());
  for (const auto& p : in) out.push_back({c * p.x - s * p.y + dx, s * p.x + c * p.y + dy});
  return out;
}

double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " <polar_image.png>\n";
    return 2;
  }

  const radloc::PolarScan scan = radloc::loadPolarImage(argv[1]);
  // MulRan / Oxford: 0.0596 m per bin, eleven metadata bins at the front.
  const auto points = radloc::toCartesianPoints(scan, 0.0596, 60.0, 11);
  std::cout << "scan " << scan.numRange() << "x" << scan.numAzimuth()
            << ", " << points.size() << " detections above threshold\n\n";

  const radloc::PhaseCorrParams params;
  struct Case { double dx, dy, dtheta_deg; };
  const std::vector<Case> cases = {
      {0, 0, 0},    {5, 0, 0},     {0, -8, 0},    {12, 7, 0},
      {0, 0, 10},   {0, 0, -25},   {6, -4, 15},   {-10, 9, 40},
      // Beyond +-90 degrees the log-polar stage is ambiguous by 180: these fail
      // unless PhaseCorrParams::resolve_180_ambiguity is set.
      {0, 0, 170},  {0, 0, 180},   {0, 0, -170},  {0, 0, 250},
      {8, -6, 200}, {-14, 3, 300},
  };

  std::printf("%8s %8s %8s | %8s %8s %8s | %7s %7s %7s | %8s\n",
              "dx", "dy", "dth", "est_dx", "est_dy", "est_dth",
              "e_x", "e_y", "e_th", "resp");
  std::printf("%s\n", std::string(88, '-').c_str());

  int failures = 0;
  for (const auto& c : cases) {
    const double dtheta = c.dtheta_deg * CV_PI / 180.0;
    const auto moved = transformPoints(points, c.dx, c.dy, dtheta);
    const auto est = radloc::registerScans(points, moved, params);

    const double ex = est.x - c.dx, ey = est.y - c.dy;
    const double eth = wrapPi(est.theta - dtheta) * 180.0 / CV_PI;
    std::printf("%8.2f %8.2f %8.1f | %8.2f %8.2f %8.1f | %7.2f %7.2f %7.2f | %8.4f\n",
                c.dx, c.dy, c.dtheta_deg, est.x, est.y, est.theta * 180.0 / CV_PI,
                ex, ey, eth, est.response);
    if (std::hypot(ex, ey) > 2.0 || std::abs(eth) > 3.0) ++failures;
  }
  std::printf("\n%d / %zu cases outside (2 m, 3 deg)\n", failures, cases.size());
  return failures == 0 ? 0 : 1;
}
