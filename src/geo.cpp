#include "geo.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rr {

namespace {

constexpr double kEarthRadiusMeters = 6371008.8;
constexpr double kMicroToRadians = 3.14159265358979323846 / 180.0 * 1e-6;

void AppendPolylineValue(std::string& out, std::int64_t value) {
  std::uint64_t v = static_cast<std::uint64_t>(value) << 1;
  if (value < 0) v = ~v;
  while (v >= 0x20) {
    out.push_back(static_cast<char>((0x20 | (v & 0x1f)) + 63));
    v >>= 5;
  }
  out.push_back(static_cast<char>(v + 63));
}

}

bool ValidCoordinate(LatLon p) {
  return p.lat >= -90000000 && p.lat <= 90000000 && p.lon >= -180000000 && p.lon <= 180000000;
}

LatLon FromDegrees(double lat, double lon) {
  return {static_cast<std::int32_t>(std::lround(lat * 1e6)),
          static_cast<std::int32_t>(std::lround(lon * 1e6))};
}

double HaversineMeters(LatLon a, LatLon b) {
  double lat1 = a.lat * kMicroToRadians;
  double lat2 = b.lat * kMicroToRadians;
  double dlat = lat2 - lat1;
  double dlon = (static_cast<double>(b.lon) - static_cast<double>(a.lon)) * kMicroToRadians;
  double s1 = std::sin(dlat / 2);
  double s2 = std::sin(dlon / 2);
  double h = s1 * s1 + std::cos(lat1) * std::cos(lat2) * s2 * s2;
  return 2 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(h)));
}

std::uint64_t HilbertKey(LatLon p) {
  std::uint32_t x = static_cast<std::uint32_t>(static_cast<std::int64_t>(p.lon) + 180000000);
  std::uint32_t y = static_cast<std::uint32_t>(static_cast<std::int64_t>(p.lat) + 90000000);
  std::uint64_t d = 0;
  for (std::uint32_t s = std::uint32_t{1} << 31; s > 0; s >>= 1) {
    std::uint32_t rx = (x & s) != 0 ? 1 : 0;
    std::uint32_t ry = (y & s) != 0 ? 1 : 0;
    d += static_cast<std::uint64_t>(s) * s * ((3 * rx) ^ ry);
    if (ry == 0) {
      if (rx == 1) {
        x = ~x;
        y = ~y;
      }
      std::swap(x, y);
    }
  }
  return d;
}

std::string EncodePolyline(std::span<const LatLon> points) {
  std::string out;
  std::int64_t prev_lat = 0;
  std::int64_t prev_lon = 0;
  for (LatLon p : points) {
    std::int64_t lat = std::llround(p.lat / 10.0);
    std::int64_t lon = std::llround(p.lon / 10.0);
    AppendPolylineValue(out, lat - prev_lat);
    AppendPolylineValue(out, lon - prev_lon);
    prev_lat = lat;
    prev_lon = lon;
  }
  return out;
}

KdTree::KdTree(std::span<const LatLon> points) {
  items_.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    items_[i] = {points[i].lat, points[i].lon, static_cast<NodeId>(i)};
  }
  Build(0, items_.size(), 0);
}

void KdTree::Build(std::size_t lo, std::size_t hi, int depth) {
  while (hi - lo > 1) {
    std::size_t mid = lo + (hi - lo) / 2;
    auto begin = items_.begin();
    if (depth % 2 == 0) {
      std::nth_element(begin + lo, begin + mid, begin + hi, [](const Item& a, const Item& b) {
        return a.lat < b.lat || (a.lat == b.lat && a.id < b.id);
      });
    } else {
      std::nth_element(begin + lo, begin + mid, begin + hi, [](const Item& a, const Item& b) {
        return a.lon < b.lon || (a.lon == b.lon && a.id < b.id);
      });
    }
    Build(lo, mid, depth + 1);
    lo = mid + 1;
    ++depth;
  }
}

NodeId KdTree::Nearest(LatLon query) const {
  if (items_.empty()) return kNoNode;
  const double coslat = std::max(0.01, std::cos(query.lat * kMicroToRadians));
  struct Frame {
    std::size_t lo;
    std::size_t hi;
    int depth;
    double bound;
  };
  Frame stack[128];
  int top = 0;
  stack[top++] = {0, items_.size(), 0, 0.0};
  double best = std::numeric_limits<double>::infinity();
  NodeId best_id = kNoNode;
  while (top > 0) {
    Frame f = stack[--top];
    if (f.lo >= f.hi || f.bound > best) continue;
    std::size_t mid = f.lo + (f.hi - f.lo) / 2;
    const Item& item = items_[mid];
    double dy = static_cast<double>(item.lat) - query.lat;
    double dx = (static_cast<double>(item.lon) - query.lon) * coslat;
    double d = dx * dx + dy * dy;
    if (d < best || (d == best && item.id < best_id)) {
      best = d;
      best_id = item.id;
    }
    double diff = f.depth % 2 == 0 ? static_cast<double>(query.lat) - item.lat
                                   : (static_cast<double>(query.lon) - item.lon) * coslat;
    Frame left{f.lo, mid, f.depth + 1, f.bound};
    Frame right{mid + 1, f.hi, f.depth + 1, f.bound};
    Frame& far = diff < 0 ? right : left;
    far.bound = std::max(f.bound, diff * diff);
    if (diff < 0) {
      stack[top++] = right;
      stack[top++] = left;
    } else {
      stack[top++] = left;
      stack[top++] = right;
    }
  }
  return best_id;
}

}
