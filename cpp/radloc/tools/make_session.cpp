// Builds an LT-SLAM session directory from a radar sequence.
//
//   make_session <polar_dir> <poses.csv> <out_dir> [keyframe_gap_m]
//
// Emits exactly what Session.cpp expects:
//
//   <out_dir>/singlesession_posegraph.g2o      VERTEX_SE3:QUAT + EDGE_SE3:QUAT
//   <out_dir>/RadLocDescriptors/<idx>,<ts>.rld one descriptor per keyframe
//   <out_dir>/Scans/<idx>,<ts>.pcd             sensor-frame points per keyframe
//
// Poses are written in the session's own frame, first keyframe at the origin,
// which is what a session produced by odometry looks like and what leaves
// LT-SLAM an actual inter-session transform to recover.
//
// This stands in for the odometry front end until it is ported; the geometry it
// writes comes from the dataset's ground-truth trajectory.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "radloc/descriptor.hpp"
#include "radloc/polar_image.hpp"

namespace fs = std::filesystem;

namespace {

struct Pose2D { double x = 0, y = 0, yaw = 0; };

std::vector<Pose2D> loadPoses(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::vector<Pose2D> poses;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> v;
    while (std::getline(ss, cell, ',')) v.push_back(cell.empty() ? 0.0 : std::stod(cell));
    if (v.size() < 3) continue;
    poses.push_back({v[1], v[2], 0.0});
  }
  // Heading from the direction of travel; the file carries no usable yaw.
  double last = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const std::size_t a = (i == 0) ? 0 : i - 1;
    const std::size_t b = std::min(poses.size() - 1, i + 1);
    const double dx = poses[b].x - poses[a].x, dy = poses[b].y - poses[a].y;
    if (std::hypot(dx, dy) > 0.05) last = std::atan2(dy, dx);
    poses[i].yaw = last;
  }
  return poses;
}

// Pose of `b` expressed in the frame of `a`.
Pose2D between(const Pose2D& a, const Pose2D& b) {
  const double c = std::cos(-a.yaw), s = std::sin(-a.yaw);
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double dyaw = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
  return {c * dx - s * dy, s * dx + c * dy, dyaw};
}

void writeVertex(std::ofstream& g2o, int idx, const Pose2D& p) {
  g2o << "VERTEX_SE3:QUAT " << idx << " " << p.x << " " << p.y << " 0 0 0 "
      << std::sin(p.yaw / 2.0) << " " << std::cos(p.yaw / 2.0) << "\n";
}

void writeEdge(std::ofstream& g2o, int from, int to, const Pose2D& rel) {
  g2o << "EDGE_SE3:QUAT " << from << " " << to << " " << rel.x << " " << rel.y
      << " 0 0 0 " << std::sin(rel.yaw / 2.0) << " " << std::cos(rel.yaw / 2.0)
      << " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n";
}

// Binary PCD of pcl::PointXYZI, which is what LT-SLAM's PointType is. Binary
// rather than ASCII because a session runs to hundreds of megabytes either way.
void writePcd(const std::string& path, const std::vector<radloc::Point2D>& pts) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write " + path);
  f << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
    << "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
    << "WIDTH " << pts.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
    << "POINTS " << pts.size() << "\nDATA binary\n";
  for (const auto& p : pts) {
    const float row[4] = {static_cast<float>(p.x), static_cast<float>(p.y), 0.0f, 1.0f};
    f.write(reinterpret_cast<const char*>(row), sizeof(row));
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0]
              << " <polar_dir> <poses.csv> <out_dir> [keyframe_gap_m]\n";
    return 2;
  }
  const std::string out_dir = argv[3];
  const double gap = (argc > 4) ? std::stod(argv[4]) : 5.0;

  std::vector<std::string> images;
  for (const auto& e : fs::directory_iterator(argv[1]))
    if (e.path().extension() == ".png") images.push_back(e.path().string());
  std::sort(images.begin(), images.end());

  const std::vector<Pose2D> poses = loadPoses(argv[2]);
  const int n = static_cast<int>(std::min(images.size(), poses.size()));

  fs::create_directories(out_dir + "/RadLocDescriptors");
  fs::create_directories(out_dir + "/Scans");

  // keyframes every `gap` metres of travel
  std::vector<int> keys{0};
  for (int i = 1; i < n; ++i)
    if (std::hypot(poses[i].x - poses[keys.back()].x,
                   poses[i].y - poses[keys.back()].y) >= gap)
      keys.push_back(i);

  std::ofstream g2o(out_dir + "/singlesession_posegraph.g2o");
  g2o << std::setprecision(9);
  const Pose2D& origin = poses[keys.front()];

  for (std::size_t k = 0; k < keys.size(); ++k) {
    const int frame = keys[k];
    const std::string stem = fs::path(images[frame]).stem().string();

    writeVertex(g2o, static_cast<int>(k), between(origin, poses[frame]));
    if (k > 0)
      writeEdge(g2o, static_cast<int>(k - 1), static_cast<int>(k),
                between(poses[keys[k - 1]], poses[frame]));

    const radloc::PolarScan scan = radloc::loadPolarImage(images[frame]);
    std::ostringstream name;
    name << k << "," << stem;
    writeRadLocDescriptorText(out_dir + "/RadLocDescriptors/" + name.str() + ".rld",
                              radloc::computeDescriptor(scan));
    writePcd(out_dir + "/Scans/" + name.str() + ".pcd",
                  radloc::toCartesianPoints(scan, 0.0596, 60.0, 11));

    if (k % 100 == 0)
      std::cout << "  " << k << " / " << keys.size() << "\r" << std::flush;
  }
  std::cout << "wrote " << keys.size() << " keyframes to " << out_dir
            << " (from " << n << " scans, " << gap << " m spacing)\n";
  return 0;
}
