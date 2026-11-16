#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "base.h"

namespace rr {

bool ValidCoordinate(LatLon p);
double HaversineMeters(LatLon a, LatLon b);
std::uint64_t HilbertKey(LatLon p);
std::string EncodePolyline(std::span<const LatLon> points);
LatLon FromDegrees(double lat, double lon);

class KdTree {
 public:
  KdTree() = default;
  explicit KdTree(std::span<const LatLon> points);

  NodeId Nearest(LatLon query) const;
  std::size_t size() const { return items_.size(); }

 private:
  struct Item {
    std::int32_t lat;
    std::int32_t lon;
    NodeId id;
  };

  void Build(std::size_t lo, std::size_t hi, int depth);

  std::vector<Item> items_;
};

}
