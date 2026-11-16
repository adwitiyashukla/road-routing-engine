#include "ch.h"

#include <algorithm>
#include <string>
#include <vector>

#include "contract.h"
#include "geo.h"
#include "graph.h"
#include "search.h"
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

RR_TEST(TheHierarchyAnswersLikeAllPairs) {
  for (int threads : {1, 2, 4}) {
    testing::TempDir dir;
    Result<Graph> graph = testing::RandomRoadGraph(400, 12 + threads, dir);
    RR_CHECK_OK(graph);
    const std::uint32_t n = graph->node_count();
    std::vector<Weight> expected = testing::AllPairs(*graph);
    ContractOptions options;
    options.threads = threads;
    ContractReport report;
    Result<ChGraph> ch = Contract(*graph, options, &report);
    RR_CHECK_OK(ch);
    RR_CHECK_EQ(ch->node_count(), n);
    RR_CHECK(report.rounds > 1);
    RR_CHECK(ch->arc_count() >= graph->arc_count());
    ChQuery query(*ch);
    for (NodeId source = 0; source < n; source += 7) {
      for (NodeId target = 0; target < n; target += 11) {
        Weight want = expected[std::size_t{source} * n + target];
        Weight got = query.Run(source, target);
        if (got != want) {
          testing::Report(__FILE__, __LINE__,
                          "threads " + std::to_string(threads) + ": " + std::to_string(source) +
                              " to " + std::to_string(target) + " gave " + std::to_string(got) +
                              " instead of " + std::to_string(want));
          return;
        }
        if (want == kInf) continue;
        std::string problem;
        if (!CheckPath(*graph, query.Path(), source, target, want, &problem)) {
          testing::Report(__FILE__, __LINE__, problem);
          return;
        }
      }
    }
  }
}

RR_TEST(OneWayStreetsStayOneWay) {
  std::vector<Arc> arcs = {{0, 1, 10}, {1, 2, 10}, {2, 3, 10}, {3, 0, 10}};
  Result<Graph> graph = Graph::FromArcs(4, arcs, Line(4));
  RR_CHECK_OK(graph);
  ContractOptions options;
  Result<ChGraph> ch = Contract(*graph, options, nullptr);
  RR_CHECK_OK(ch);
  ChQuery query(*ch);
  RR_CHECK_EQ(query.Run(0, 3), 30u);
  RR_CHECK_EQ(query.Run(3, 0), 10u);
  RR_CHECK_EQ(query.Run(1, 0), 30u);
  RR_CHECK_EQ(query.Run(2, 2), 0u);
  std::vector<NodeId> path = query.Path();
  RR_CHECK_EQ(path.size(), std::size_t{1});
}

RR_TEST(UnreachableTargetsStayUnreachable) {
  std::vector<Arc> arcs = {{0, 1, 5}, {1, 0, 5}, {2, 3, 5}, {3, 2, 5}};
  Result<Graph> graph = Graph::FromArcs(4, arcs, Line(4));
  RR_CHECK_OK(graph);
  ContractOptions options;
  Result<ChGraph> ch = Contract(*graph, options, nullptr);
  RR_CHECK_OK(ch);
  ChQuery query(*ch);
  RR_CHECK_EQ(query.Run(0, 1), 5u);
  RR_CHECK_EQ(query.Run(0, 2), kInf);
  RR_CHECK(query.Path().empty());
  RR_CHECK_EQ(query.Run(3, 1), kInf);
}

RR_TEST(HierarchyFilesRoundTripAndRefuseTheWrongGraph) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(2000, 15, dir);
  RR_CHECK_OK(graph);
  ContractOptions options;
  options.threads = 2;
  Result<ChGraph> ch = Contract(*graph, options, nullptr);
  RR_CHECK_OK(ch);
  std::string path = dir.File("graph.rrch");
  RR_CHECK_STATUS(ch->Save(path, *graph));
  Result<ChGraph> loaded = ChGraph::Load(path, *graph, true);
  RR_CHECK_OK(loaded);
  RR_CHECK_EQ(loaded->arc_count(), ch->arc_count());
  RR_CHECK_EQ(loaded->shortcut_count(), ch->shortcut_count());
  ChQuery original(*ch);
  ChQuery copy(*loaded);
  Rng rng(5);
  for (int query = 0; query < 200; ++query) {
    NodeId source = rng.Below(graph->node_count());
    NodeId target = rng.Below(graph->node_count());
    RR_CHECK_EQ(copy.Run(source, target), original.Run(source, target));
  }
  Result<Graph> other = testing::RandomRoadGraph(2000, 16, dir);
  RR_CHECK_OK(other);
  RR_CHECK(!ChGraph::Load(path, *other, false).ok());
}

RR_TEST(BrokenHierarchiesAreRefused) {
  ChGraph::Parts parts;
  parts.rank = {0, 0};
  for (int d = 0; d < 2; ++d) parts.first[d] = {0, 0, 0};
  RR_CHECK(!ChGraph::FromParts(std::move(parts)).ok());

  ChGraph::Parts downward;
  downward.rank = {0, 1};
  downward.first[0] = {0, 1, 1};
  downward.head[0] = {0};
  downward.weight[0] = {5};
  downward.middle[0] = {kNoNode};
  downward.first[1] = {0, 0, 0};
  RR_CHECK(!ChGraph::FromParts(std::move(downward)).ok());

  ChGraph::Parts mismatched;
  mismatched.rank = {0, 1};
  mismatched.first[0] = {0, 1, 1};
  mismatched.head[0] = {1};
  mismatched.weight[0] = {5, 6};
  mismatched.middle[0] = {kNoNode};
  mismatched.first[1] = {0, 0, 0};
  RR_CHECK(!ChGraph::FromParts(std::move(mismatched)).ok());

  ChGraph::Parts good;
  good.rank = {0, 1};
  good.first[0] = {0, 1, 1};
  good.head[0] = {1};
  good.weight[0] = {5};
  good.middle[0] = {kNoNode};
  good.first[1] = {0, 0, 0};
  Result<ChGraph> ch = ChGraph::FromParts(std::move(good));
  RR_CHECK_OK(ch);
  ChQuery query(*ch);
  RR_CHECK_EQ(query.Run(0, 1), 5u);
  RR_CHECK_EQ(query.Run(1, 0), kInf);
}

RR_TEST(ManyToManyTablesMatchSingleQueries) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(3000, 17, dir);
  RR_CHECK_OK(graph);
  ContractOptions options;
  options.threads = 2;
  Result<ChGraph> ch = Contract(*graph, options, nullptr);
  RR_CHECK_OK(ch);
  ThreadPool pool(3);
  Rng rng(6);
  std::vector<NodeId> sources;
  std::vector<NodeId> targets;
  for (int i = 0; i < 40; ++i) sources.push_back(rng.Below(graph->node_count()));
  for (int i = 0; i < 25; ++i) targets.push_back(rng.Below(graph->node_count()));
  sources.push_back(targets[0]);
  std::vector<Weight> table = ChTable(*ch).Compute(sources, targets, pool);
  RR_CHECK_EQ(table.size(), sources.size() * targets.size());
  ChQuery query(*ch);
  for (std::size_t i = 0; i < sources.size(); ++i) {
    for (std::size_t j = 0; j < targets.size(); ++j) {
      Weight want = query.Run(sources[i], targets[j]);
      Weight got = table[i * targets.size() + j];
      if (want != got) {
        testing::Report(__FILE__, __LINE__,
                        std::to_string(sources[i]) + " to " + std::to_string(targets[j]) +
                            ": table says " + std::to_string(got) + " and the query says " +
                            std::to_string(want));
        return;
      }
    }
  }
  std::vector<Weight> empty = ChTable(*ch).Compute({}, targets, pool);
  RR_CHECK(empty.empty());
}

RR_TEST(ContractionHandlesTinyAndDisconnectedInput) {
  Result<Graph> single = Graph::FromArcs(1, {}, Line(1));
  RR_CHECK_OK(single);
  ContractOptions options;
  Result<ChGraph> ch = Contract(*single, options, nullptr);
  RR_CHECK_OK(ch);
  ChQuery query(*ch);
  RR_CHECK_EQ(query.Run(0, 0), 0u);
  RR_CHECK_EQ(ch->arc_count(), std::uint64_t{0});

  Result<Graph> pair = Graph::FromArcs(2, {{0, 1, 7}}, Line(2));
  RR_CHECK_OK(pair);
  Result<ChGraph> ch2 = Contract(*pair, options, nullptr);
  RR_CHECK_OK(ch2);
  ChQuery query2(*ch2);
  RR_CHECK_EQ(query2.Run(0, 1), 7u);
  RR_CHECK_EQ(query2.Run(1, 0), kInf);
}

RR_TEST(PathsUnpackThroughShortcuts) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(2500, 18, dir);
  RR_CHECK_OK(graph);
  ContractOptions options;
  options.threads = 2;
  Result<ChGraph> ch = Contract(*graph, options, nullptr);
  RR_CHECK_OK(ch);
  RR_CHECK(ch->shortcut_count() > 0);
  ChQuery query(*ch);
  Dijkstra reference(graph->forward());
  Rng rng(19);
  std::size_t longest = 0;
  for (int i = 0; i < 150; ++i) {
    NodeId source = rng.Below(graph->node_count());
    NodeId target = rng.Below(graph->node_count());
    Weight want = reference.Run(source, target);
    RR_CHECK_EQ(query.Run(source, target), want);
    if (want == kInf) continue;
    std::vector<NodeId> path = query.Path();
    std::string problem;
    if (!CheckPath(*graph, path, source, target, want, &problem)) {
      testing::Report(__FILE__, __LINE__, problem);
      return;
    }
    longest = std::max(longest, path.size());
  }
  RR_CHECK(longest > 10);
}

RR_TEST(PeelingDeadEndsNeedsNoShortcuts) {
  std::vector<Arc> arcs = {{0, 1, 10}, {1, 0, 10},  {1, 2, 10},  {2, 1, 10},  {2, 3, 10},
                           {3, 2, 10}, {0, 4, 100}, {4, 0, 100}, {3, 4, 100}, {4, 3, 100}};
  Result<Graph> graph = Graph::FromArcs(5, arcs, Line(5));
  RR_CHECK_OK(graph);
  ContractOptions options;
  Result<ChGraph> ch = Contract(*graph, options, nullptr);
  RR_CHECK_OK(ch);
  RR_CHECK_EQ(ch->shortcut_count(), std::uint64_t{0});
  ChQuery query(*ch);
  RR_CHECK_EQ(query.Run(0, 3), 30u);
  std::vector<NodeId> path = query.Path();
  RR_CHECK_EQ(path.size(), std::size_t{4});
  std::string problem;
  RR_CHECK(CheckPath(*graph, path, 0, 3, 30, &problem));
  RR_CHECK_EQ(query.Run(4, 2), 110u);
  RR_CHECK(CheckPath(*graph, query.Path(), 4, 2, 110, &problem));
}

}
}

int main() { return rr::testing::RunAll(); }
