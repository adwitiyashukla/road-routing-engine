#include "search.h"

#include <string>
#include <vector>

#include "geo.h"
#include "graph.h"
#include "support.h"
#include "testing.h"
#include "thread_pool.h"

namespace rr {
namespace {

std::vector<LatLon> Line(std::uint32_t count) {
  std::vector<LatLon> coords;
  for (std::uint32_t i = 0; i < count; ++i) {
    coords.push_back(FromDegrees(39.1 + i * 0.01, -84.5));
  }
  return coords;
}

bool CheckPath(const Graph& graph, const std::vector<NodeId>& path, NodeId source, NodeId target,
               Weight weight, std::string* problem) {
  if (path.empty() || path.front() != source || path.back() != target) {
    *problem = "the path does not run from the source to the target";
    return false;
  }
  std::uint64_t sum = 0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    Weight arc = graph.ArcWeight(path[i], path[i + 1]);
    if (arc == kInf) {
      *problem = "the path uses an arc that is not in the graph";
      return false;
    }
    sum += arc;
  }
  if (sum != weight) {
    *problem = "the path weighs " + std::to_string(sum) + " but the query returned " +
               std::to_string(weight);
    return false;
  }
  return true;
}

RR_TEST(DijkstraOnAChainAndADeadEnd) {
  Result<Graph> graph = Graph::FromArcs(4, {{0, 1, 3}, {1, 2, 4}, {2, 3, 5}}, Line(4));
  RR_CHECK_OK(graph);
  Dijkstra search(graph->forward());
  RR_CHECK_EQ(search.Run(0, 3), 12u);
  RR_CHECK_EQ(search.Run(0, 0), 0u);
  RR_CHECK_EQ(search.Run(3, 0), kInf);
  RR_CHECK_EQ(search.Run(1, 2), 4u);
  std::vector<NodeId> path = search.Path();
  RR_CHECK_EQ(path.size(), std::size_t{2});
  RR_CHECK_EQ(path[0], 1u);
  RR_CHECK_EQ(path[1], 2u);
}

RR_TEST(TheCheapestRouteWinsOverTheShortestHop) {
  Result<Graph> graph =
      Graph::FromArcs(4, {{0, 1, 100}, {1, 3, 100}, {0, 2, 10}, {2, 3, 10}}, Line(4));
  RR_CHECK_OK(graph);
  Dijkstra search(graph->forward());
  RR_CHECK_EQ(search.Run(0, 3), 20u);
  std::vector<NodeId> path = search.Path();
  RR_CHECK_EQ(path.size(), std::size_t{3});
  RR_CHECK_EQ(path[1], 2u);
}

RR_TEST(EveryAlgorithmMatchesAllPairsOnASmallNetwork) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(300, 3, dir);
  RR_CHECK_OK(graph);
  const std::uint32_t n = graph->node_count();
  std::vector<Weight> expected = testing::AllPairs(*graph);
  ReverseGraph reverse(*graph);
  ThreadPool pool(2);
  double scale = GeoScale(*graph);
  RR_CHECK(scale > 0);
  Result<Landmarks> landmarks = Landmarks::Build(*graph, 6, 7, pool);
  RR_CHECK_OK(landmarks);
  RR_CHECK_EQ(landmarks->count(), 6u);
  Dijkstra plain(graph->forward());
  BidirectionalDijkstra bidirectional(graph->forward(), reverse.view());
  AStar<GeoPotential> astar(graph->forward(), GeoPotential(*graph, scale));
  BidirectionalAStar<GeoPotential> bidirectional_astar(graph->forward(), reverse.view(),
                                                       GeoPotential(*graph, scale));
  AStar<LandmarkPotential> alt(graph->forward(), LandmarkPotential(*landmarks, 4));
  BidirectionalAStar<LandmarkPotential> bidirectional_alt(graph->forward(), reverse.view(),
                                                          LandmarkPotential(*landmarks, 6));
  Rng rng(21);
  int checked = 0;
  for (int query = 0; query < 400; ++query) {
    NodeId source = rng.Below(n);
    NodeId target = rng.Below(n);
    Weight want = expected[std::size_t{source} * n + target];
    RR_CHECK_EQ(plain.Run(source, target), want);
    RR_CHECK_EQ(bidirectional.Run(source, target), want);
    RR_CHECK_EQ(astar.Run(source, target), want);
    RR_CHECK_EQ(bidirectional_astar.Run(source, target), want);
    RR_CHECK_EQ(alt.Run(source, target), want);
    RR_CHECK_EQ(bidirectional_alt.Run(source, target), want);
    if (want == kInf) continue;
    std::string problem;
    if (!CheckPath(*graph, plain.Path(), source, target, want, &problem) ||
        !CheckPath(*graph, bidirectional.Path(), source, target, want, &problem) ||
        !CheckPath(*graph, astar.Path(), source, target, want, &problem) ||
        !CheckPath(*graph, bidirectional_astar.Path(), source, target, want, &problem) ||
        !CheckPath(*graph, alt.Path(), source, target, want, &problem) ||
        !CheckPath(*graph, bidirectional_alt.Path(), source, target, want, &problem)) {
      testing::Report(__FILE__, __LINE__, problem);
      return;
    }
    ++checked;
  }
  RR_CHECK(checked > 300);
}

RR_TEST(GuidedSearchesSettleFewerNodes) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(20000, 4, dir);
  RR_CHECK_OK(graph);
  ReverseGraph reverse(*graph);
  ThreadPool pool(2);
  Result<Landmarks> landmarks = Landmarks::Build(*graph, 8, 5, pool);
  RR_CHECK_OK(landmarks);
  Dijkstra plain(graph->forward());
  AStar<GeoPotential> astar(graph->forward(), GeoPotential(*graph, GeoScale(*graph)));
  BidirectionalAStar<LandmarkPotential> alt(graph->forward(), reverse.view(),
                                            LandmarkPotential(*landmarks, 4));
  std::uint64_t plain_settled = 0;
  std::uint64_t astar_settled = 0;
  std::uint64_t alt_settled = 0;
  Rng rng(8);
  for (int query = 0; query < 30; ++query) {
    NodeId source = rng.Below(graph->node_count());
    NodeId target = rng.Below(graph->node_count());
    Weight want = plain.Run(source, target);
    plain_settled += plain.stats().settled;
    RR_CHECK_EQ(astar.Run(source, target), want);
    astar_settled += astar.stats().settled;
    RR_CHECK_EQ(alt.Run(source, target), want);
    alt_settled += alt.stats().settled;
  }
  RR_CHECK(astar_settled < plain_settled);
  RR_CHECK(alt_settled < astar_settled);
}

RR_TEST(LandmarkFilesRoundTripAndRefuseTheWrongGraph) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(1500, 6, dir);
  RR_CHECK_OK(graph);
  ThreadPool pool(1);
  Result<Landmarks> landmarks = Landmarks::Build(*graph, 4, 1, pool);
  RR_CHECK_OK(landmarks);
  std::string path = dir.File("graph.rrl");
  RR_CHECK_STATUS(landmarks->Save(path, *graph));
  Result<Landmarks> loaded = Landmarks::Load(path, *graph);
  RR_CHECK_OK(loaded);
  RR_CHECK_EQ(loaded->count(), 4u);
  for (NodeId v = 0; v < graph->node_count(); v += 17) {
    for (std::uint32_t i = 0; i < 8; ++i) {
      RR_CHECK_EQ(loaded->Row(v)[i], landmarks->Row(v)[i]);
    }
  }
  Result<Graph> other = testing::RandomRoadGraph(1500, 7, dir);
  RR_CHECK_OK(other);
  RR_CHECK(!Landmarks::Load(path, *other).ok());
  RR_CHECK(!Landmarks::Build(*graph, 0, 1, pool).ok());
  RR_CHECK(!Landmarks::Build(*graph, 100, 1, pool).ok());
}

RR_TEST(LandmarksNeedAStronglyConnectedGraph) {
  Result<Graph> graph = Graph::FromArcs(3, {{0, 1, 1}, {1, 2, 1}}, Line(3));
  RR_CHECK_OK(graph);
  ThreadPool pool(1);
  Result<Landmarks> landmarks = Landmarks::Build(*graph, 2, 1, pool);
  RR_CHECK(!landmarks.ok());
}

RR_TEST(TheGeometricBoundNeverExceedsTheTrueDistance) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(400, 9, dir);
  RR_CHECK_OK(graph);
  const std::uint32_t n = graph->node_count();
  std::vector<Weight> expected = testing::AllPairs(*graph);
  double scale = GeoScale(*graph);
  GeoPotential potential(*graph, scale);
  Rng rng(33);
  for (int query = 0; query < 2000; ++query) {
    NodeId source = rng.Below(n);
    NodeId target = rng.Below(n);
    potential.Prepare(source, target);
    Weight want = expected[std::size_t{source} * n + target];
    if (want == kInf) continue;
    RR_CHECK(potential.ToTarget(source) <= static_cast<std::int64_t>(want));
    RR_CHECK(potential.FromSource(target) <= static_cast<std::int64_t>(want));
  }
}

RR_TEST(TheLandmarkBoundNeverExceedsTheTrueDistance) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(400, 10, dir);
  RR_CHECK_OK(graph);
  const std::uint32_t n = graph->node_count();
  std::vector<Weight> expected = testing::AllPairs(*graph);
  ThreadPool pool(2);
  Result<Landmarks> landmarks = Landmarks::Build(*graph, 5, 2, pool);
  RR_CHECK_OK(landmarks);
  LandmarkPotential potential(*landmarks, 5);
  Rng rng(44);
  for (int query = 0; query < 2000; ++query) {
    NodeId source = rng.Below(n);
    NodeId target = rng.Below(n);
    potential.Prepare(source, target);
    Weight want = expected[std::size_t{source} * n + target];
    RR_CHECK(potential.ToTarget(source) <= static_cast<std::int64_t>(want));
    NodeId middle = rng.Below(n);
    Weight to_middle = expected[std::size_t{source} * n + middle];
    Weight from_middle = expected[std::size_t{middle} * n + target];
    if (to_middle == kInf || from_middle == kInf) continue;
    RR_CHECK(potential.ToTarget(middle) <= static_cast<std::int64_t>(from_middle));
    RR_CHECK(potential.FromSource(middle) <= static_cast<std::int64_t>(to_middle));
  }
}

}
}

int main() { return rr::testing::RunAll(); }
