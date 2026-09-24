// Checks the two-stage retrieval against the Python reference by running both
// over the same archived descriptors.
//
//   validate_retrieval <npy_dir> <num_desc> [coarse_dims] [top_k]
//
// Prints, for every query, the shortlist the C++ side produces; compare with
// tools/reference_retrieval.py run on the same inputs.

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "radloc/retrieval.hpp"

namespace fs = std::filesystem;

static std::vector<double> loadNpy(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  char magic[6]; f.read(magic, 6);
  if (std::memcmp(magic, "\x93NUMPY", 6) != 0) throw std::runtime_error("not .npy: " + path);
  uint8_t major = 0, minor = 0;
  f.read(reinterpret_cast<char*>(&major), 1);
  f.read(reinterpret_cast<char*>(&minor), 1);
  uint32_t header_len = 0;
  if (major == 1) { uint16_t l = 0; f.read(reinterpret_cast<char*>(&l), 2); header_len = l; }
  else { f.read(reinterpret_cast<char*>(&header_len), 4); }
  std::string header(header_len, '\0'); f.read(header.data(), header_len);
  const std::streampos begin = f.tellg();
  f.seekg(0, std::ios::end);
  const std::streamoff bytes = f.tellg() - begin;
  f.seekg(begin);
  std::vector<double> out(static_cast<size_t>(bytes / 8));
  f.read(reinterpret_cast<char*>(out.data()), bytes);
  return out;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " <npy_dir> <num_desc> [coarse_dims] [top_k]\n";
    return 2;
  }
  const std::string npy_dir = argv[1];
  const int n = std::stoi(argv[2]);

  radloc::RetrievalParams params;
  if (argc > 3) params.coarse_dims = std::stoi(argv[3]);
  if (argc > 4) params.top_k = std::stoi(argv[4]);

  // Split the archive in half: first half is the database, second half queries.
  const int split = n / 2;
  radloc::PlaceDatabase db(params);
  std::vector<radloc::Descriptor> queries;

  for (int i = 0; i < n; ++i) {
    std::ostringstream name;
    name << npy_dir << "/" << std::setw(6) << std::setfill('0') << i << ".npy";
    radloc::Descriptor d;
    const std::vector<double> weighted = radloc::fromInverted(loadNpy(name.str()));
    d.bands.resize(weighted.size());
    for (size_t b = 0; b < weighted.size(); ++b) d.bands[b] = weighted[b] / double(b + 1);
    if (i < split) db.add(d); else queries.push_back(d);
  }
  db.build();

  std::cout << std::fixed << std::setprecision(6);
  for (size_t q = 0; q < queries.size(); ++q) {
    const auto matches = db.query(queries[q]);
    std::cout << (split + q);
    for (const auto& m : matches) std::cout << " " << m.index << ":" << m.distance;
    std::cout << "\n";
  }
  return 0;
}
