#pragma once

#include <cstdint>
#include <string>

#include "base.h"

namespace rr {

struct GeneratorOptions {
  std::uint32_t nodes = 100000;
  std::uint64_t seed = 1;
  bool travel_time = true;
  double center_lat = 39.1;
  double center_lon = -84.5;
};

struct GeneratorSummary {
  std::uint64_t nodes = 0;
  std::uint64_t arcs = 0;
};

Status GenerateRoadNetwork(const GeneratorOptions& options, const std::string& graph_path,
                           const std::string& coord_path, GeneratorSummary* summary);

}
