// Self-contained checks of the core (no dataset): rotation invariance of the descriptor, place retrieval, and
// phase-correlation registration of a synthetic scene.
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "radloc/descriptor.hpp"
#include "radloc/phase_corr.hpp"
#include "radloc/retrieval.hpp"

static int failures = 0;
#define CHECK(c)                                                    \
  do {                                                              \
    if (!(c)) {                                                     \
      std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);       \
      ++failures;                                                   \
    }                                                               \
  } while (0)

static radloc::PolarScan scene(unsigned seed, int range = 3360, int azimuth = 400) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> u(0, 1);
  cv::Mat m(range, azimuth, CV_64F);
  for (int r = 0; r < range; ++r)
    for (int a = 0; a < azimuth; ++a) m.at<double>(r, a) = 20 * u(g) + 80.0 * std::exp(-r / 800.0);
  for (int k = 0; k < 60; ++k) {
    const int r = g() % (range - 40), a = g() % (azimuth - 6);
    m(cv::Rect(a, r, 6, 40)) += 120 * u(g);
  }
  return radloc::PolarScan{m};
}

int main() {
  // 1. rotation invariance: shifting the azimuths (a heading change) leaves the bands unchanged
  const auto s = scene(1);
  cv::Mat rolled;
  cv::hconcat(s.data.colRange(100, s.numAzimuth()), s.data.colRange(0, 100), rolled);
  const auto d0 = radloc::computeDescriptor(s), d1 = radloc::computeDescriptor(radloc::PolarScan{rolled});
  CHECK(d0.size() == 40);
  double diff = 0;
  for (size_t b = 0; b < d0.size(); ++b) diff = std::max(diff, std::abs(d0.bands[b] - d1.bands[b]));
  CHECK(diff < 1e-9);

  // 2. retrieval: a database of distinct places finds the rotated copy of place 3
  radloc::PlaceDatabase db;
  for (unsigned i = 0; i < 8; ++i) db.add(radloc::computeDescriptor(scene(10 + i)));
  cv::Mat r3;
  const auto s3 = scene(13);
  cv::hconcat(s3.data.colRange(37, s3.numAzimuth()), s3.data.colRange(0, 37), r3);
  const auto m = db.query(radloc::computeDescriptor(radloc::PolarScan{r3}));
  CHECK(!m.empty() && m.front().index == 3);

  // 3. registration: a random 2D scene moved by a known rigid transform
  std::mt19937 g(7);
  std::uniform_real_distribution<double> u(-80, 80);
  std::vector<radloc::Point2D> target;
  for (int i = 0; i < 3000; ++i) target.push_back({u(g), u(g)});
  for (int i = 0; i < 2000; ++i) {  // a few walls
    const double t = (i % 500) / 500.0 * 120 - 60;
    target.push_back(i < 500 ? radloc::Point2D{t, 40} : i < 1000 ? radloc::Point2D{-50, t}
                     : i < 1500 ? radloc::Point2D{t * 0.5, -30 + t * 0.2} : radloc::Point2D{30, t * 0.7});
  }
  const double dx = 6, dy = -4, th = 15 * CV_PI / 180;
  // source = R target + t, the convention of tools/test_phase_corr.cpp: registerScans returns (t, R)
  std::vector<radloc::Point2D> source;
  for (const auto& p : target)
    source.push_back({std::cos(th) * p.x - std::sin(th) * p.y + dx, std::sin(th) * p.x + std::cos(th) * p.y + dy});
  const auto T = radloc::registerScans(target, source);
  std::printf("registration: x %.2f y %.2f theta %.2f deg (truth 6, -4, 15), response %.3f\n", T.x, T.y,
              T.theta * 180 / CV_PI, T.response);
  CHECK(T.valid);
  CHECK(std::abs(T.theta - th) < 2 * CV_PI / 180);
  CHECK(std::hypot(T.x - dx, T.y - dy) < 2.0);

  std::printf(failures ? "%d failure(s)\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
