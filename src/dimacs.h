#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "graph.h"

namespace rr {

struct RoadArcs {
  std::uint32_t node_count = 0;
  std::vector<Arc> arcs;
};

Result<RoadArcs> ReadRoadGraph(const std::string& path);
Result<std::vector<LatLon>> ReadRoadCoordinates(const std::string& path, std::uint32_t node_count);

}
