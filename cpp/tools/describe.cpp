// Prints the RadLoc descriptor of each polar image, one line per image (for the C++/Python parity test).
//   describe <polar.png> [...]
#include <cstdio>
#include <iostream>

#include "radloc/descriptor.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: describe <polar.png> [...]\n";
    return 2;
  }
  for (int i = 1; i < argc; ++i) {
    const auto d = radloc::computeDescriptor(std::string(argv[i]));
    for (size_t b = 0; b < d.size(); ++b) std::printf(b ? " %.12g" : "%.12g", d.bands[b]);
    std::printf("\n");
  }
  return 0;
}
