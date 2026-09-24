// Checks the C++ descriptor against the reference .npy files produced by the
// Python implementation.
//
//   validate_descriptor <polar_dir> <npy_dir> [max_frames]

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "radloc/descriptor.hpp"

namespace fs = std::filesystem;

// Minimal reader for 1-D little-endian float64 .npy (format 1.0/2.0).
static std::vector<double> loadNpy(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);

  char magic[6];
  f.read(magic, 6);
  if (std::memcmp(magic, "\x93NUMPY", 6) != 0)
    throw std::runtime_error("not a .npy file: " + path);

  uint8_t major = 0, minor = 0;
  f.read(reinterpret_cast<char*>(&major), 1);
  f.read(reinterpret_cast<char*>(&minor), 1);

  uint32_t header_len = 0;
  if (major == 1) {
    uint16_t len16 = 0;
    f.read(reinterpret_cast<char*>(&len16), 2);
    header_len = len16;
  } else {
    f.read(reinterpret_cast<char*>(&header_len), 4);
  }

  std::string header(header_len, '\0');
  f.read(header.data(), header_len);
  if (header.find("'<f8'") == std::string::npos &&
      header.find("\"<f8\"") == std::string::npos)
    throw std::runtime_error("expected float64 in " + path + " -> " + header);
  if (header.find("True") != std::string::npos)
    throw std::runtime_error("fortran_order not supported: " + path);

  // Element count from the remaining bytes; shape may be (N,) or (N,1).
  const std::streampos data_begin = f.tellg();
  f.seekg(0, std::ios::end);
  const std::streamoff bytes = f.tellg() - data_begin;
  f.seekg(data_begin);

  std::vector<double> out(static_cast<size_t>(bytes / 8));
  f.read(reinterpret_cast<char*>(out.data()), bytes);
  return out;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0]
              << " <polar_dir> <npy_dir> [max_frames]\n";
    return 2;
  }
  const std::string polar_dir = argv[1];
  const std::string npy_dir = argv[2];
  const int max_frames = (argc > 3) ? std::stoi(argv[3]) : 50;

  std::vector<std::string> images;
  for (const auto& e : fs::directory_iterator(polar_dir))
    if (e.path().extension() == ".png") images.push_back(e.path().string());
  std::sort(images.begin(), images.end());
  if (images.empty()) {
    std::cerr << "no .png under " << polar_dir << "\n";
    return 1;
  }

  const int n = std::min<int>(max_frames, static_cast<int>(images.size()));
  double worst_abs = 0.0, worst_rel = 0.0;
  int worst_idx = -1, compared = 0, length = 0;

  for (int i = 0; i < n; ++i) {
    std::ostringstream name;
    name << npy_dir << "/" << std::setw(6) << std::setfill('0') << i << ".npy";
    if (!fs::exists(name.str())) continue;

    // Archived descriptors are stored inverted, (255 - mean) * (b + 1).
    const std::vector<double> expected =
        radloc::fromInverted(loadNpy(name.str()));
    const std::vector<double> actual =
        radloc::computeDescriptor(images[i]).rangeWeighted();
    if (expected.size() != actual.size()) {
      std::cerr << "frame " << i << ": length " << actual.size() << " vs "
                << expected.size() << " (reference)\n";
      return 1;
    }
    length = static_cast<int>(actual.size());

    double peak = 0.0;
    for (double v : expected) peak = std::max(peak, std::abs(v));
    for (size_t k = 0; k < actual.size(); ++k) {
      const double d = std::abs(actual[k] - expected[k]);
      if (d > worst_abs) {
        worst_abs = d;
        worst_rel = (peak > 0) ? d / peak : 0.0;
        worst_idx = i;
      }
    }
    ++compared;
  }

  if (compared == 0) {
    std::cerr << "no reference .npy matched under " << npy_dir << "\n";
    return 1;
  }

  std::cout << "frames compared : " << compared << "\n"
            << "descriptor length: " << length << "\n"
            << "max |diff|      : " << std::scientific << std::setprecision(3)
            << worst_abs << "  (frame " << worst_idx << ")\n"
            << "max relative    : " << worst_rel << "\n";

  const bool ok = worst_rel < 1e-9;
  std::cout << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
