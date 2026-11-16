#include "geo.h"

#include <cmath>
#include <string>
#include <vector>

#include "base.h"
#include "testing.h"

namespace rr {
namespace {

RR_TEST(HaversineMatchesKnownDistances) {
  LatLon cincinnati = FromDegrees(39.1031, -84.5120);
  LatLon columbus = FromDegrees(39.9612, -82.9988);
  double meters = HaversineMeters(cincinnati, columbus);
  RR_CHECK(meters > 160000 && meters < 162000);
  RR_CHECK_EQ(HaversineMeters(cincinnati, cincinnati), 0.0);
  LatLon equator_a = FromDegrees(0, 0);
  LatLon equator_b = FromDegrees(0, 1);
  double degree = HaversineMeters(equator_a, equator_b);
  RR_CHECK(degree > 111100 && degree < 111400);
  LatLon pole_a = FromDegrees(89.9, 0);
  LatLon pole_b = FromDegrees(89.9, 180);
  double across = HaversineMeters(pole_a, pole_b);
  RR_CHECK(across > 22000 && across < 23000);
}

RR_TEST(PolylineEncodingMatchesTheGoogleExample) {
  std::vector<LatLon> points = {FromDegrees(38.5, -120.2), FromDegrees(40.7, -120.95),
                                FromDegrees(43.252, -126.453)};
  RR_CHECK_EQ(EncodePolyline(points), std::string("_p~iF~ps|U_ulLnnqC_mqNvxq`@"));
  RR_CHECK_EQ(EncodePolyline({}), std::string(""));
}

RR_TEST(HilbertKeysKeepNeighborsClose) {
  std::uint64_t a = HilbertKey(FromDegrees(39.1, -84.5));
  std::uint64_t b = HilbertKey(FromDegrees(39.1, -84.5));
  RR_CHECK_EQ(a, b);
  std::uint64_t near = HilbertKey(FromDegrees(39.100001, -84.500001));
  std::uint64_t far = HilbertKey(FromDegrees(-20.0, 150.0));
  std::uint64_t near_gap = a > near ? a - near : near - a;
  std::uint64_t far_gap = a > far ? a - far : far - a;
  RR_CHECK(near_gap < far_gap);
}

RR_TEST(CoordinateRangesAreChecked) {
  RR_CHECK(ValidCoordinate(FromDegrees(90, 180)));
  RR_CHECK(ValidCoordinate(FromDegrees(-90, -180)));
  RR_CHECK(!ValidCoordinate({90000001, 0}));
  RR_CHECK(!ValidCoordinate({0, -180000001}));
}

RR_TEST(KdTreeFindsTheSameNodeAsALinearScan) {
  Rng rng(99);
  for (int round = 0; round < 20; ++round) {
    std::uint32_t count = 1 + rng.Below(400);
    std::vector<LatLon> points;
    for (std::uint32_t i = 0; i < count; ++i) {
      points.push_back(FromDegrees(39 + rng.Uniform() * 2, -85 + rng.Uniform() * 2));
    }
    KdTree tree(points);
    RR_CHECK_EQ(tree.size(), std::size_t{count});
    for (int query = 0; query < 50; ++query) {
      LatLon ask = FromDegrees(38.5 + rng.Uniform() * 3, -85.5 + rng.Uniform() * 3);
      NodeId got = tree.Nearest(ask);
      double best = 1e300;
      NodeId expected = kNoNode;
      for (std::uint32_t i = 0; i < count; ++i) {
        double meters = HaversineMeters(ask, points[i]);
        if (meters < best) {
          best = meters;
          expected = i;
        }
      }
      double got_meters = HaversineMeters(ask, points[got]);
      RR_CHECK(std::fabs(got_meters - best) < 1e-6);
      (void)expected;
    }
  }
}

RR_TEST(KdTreeHandlesDuplicateAndEmptyInput) {
  KdTree empty;
  RR_CHECK_EQ(empty.Nearest(FromDegrees(0, 0)), kNoNode);
  std::vector<LatLon> same(10, FromDegrees(39.1, -84.5));
  KdTree tree(same);
  NodeId got = tree.Nearest(FromDegrees(39.2, -84.6));
  RR_CHECK(got < 10);
}

}
}

int main() { return rr::testing::RunAll(); }
