#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "dimacs.h"
#include "generator.h"
#include "graph.h"

namespace rr::testing {

class TempDir {
 public:
  TempDir() {
    for (int i = 0; i < 100; ++i) {
      std::string name = "rrtest-" + std::to_string(Counter()++);
      path_ = std::filesystem::temp_directory_path() / name;
      std::error_code ec;
      if (std::filesystem::create_directory(path_, ec)) return;
    }
    path_ = std::filesystem::temp_directory_path();
  }

  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;

  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  std::string File(const std::string& name) const { return (path_ / name).string(); }

 private:
  static int& Counter() {
    static int counter = 1;
    return counter;
  }

  std::filesystem::path path_;
};

inline Result<Graph> RandomRoadGraph(std::uint32_t nodes, std::uint64_t seed, const TempDir& dir) {
  GeneratorOptions options;
  options.nodes = nodes;
  options.seed = seed;
  std::string graph_path = dir.File("random.gr");
  std::string coord_path = dir.File("random.co");
  Status written = GenerateRoadNetwork(options, graph_path, coord_path, nullptr);
  if (!written.ok()) return written;
  Result<RoadArcs> dimacs = ReadRoadGraph(graph_path);
  if (!dimacs.ok()) return dimacs.status();
  Result<std::vector<LatLon>> coords = ReadRoadCoordinates(coord_path, dimacs->node_count);
  if (!coords.ok()) return coords.status();
  Result<Graph> graph =
      Graph::FromArcs(dimacs->node_count, std::move(dimacs->arcs), std::move(*coords));
  if (!graph.ok()) return graph.status();
  Result<Graph> strong = LargestStrongComponent(*graph, nullptr);
  if (!strong.ok()) return strong.status();
  std::vector<NodeId> numbering = HilbertNumbering(*strong);
  return Renumber(*strong, numbering);
}

inline std::vector<Weight> AllPairs(const Graph& graph) {
  const std::uint32_t n = graph.node_count();
  std::vector<Weight> dist(std::size_t{n} * n, kInf);
  Adjacency g = graph.forward();
  for (NodeId v = 0; v < n; ++v) {
    dist[std::size_t{v} * n + v] = 0;
    for (std::uint32_t a = g.first[v]; a < g.first[v + 1]; ++a) {
      dist[std::size_t{v} * n + g.other[a]] = g.weight[a];
    }
  }
  for (NodeId k = 0; k < n; ++k) {
    for (NodeId i = 0; i < n; ++i) {
      Weight ik = dist[std::size_t{i} * n + k];
      if (ik == kInf) continue;
      for (NodeId j = 0; j < n; ++j) {
        Weight kj = dist[std::size_t{k} * n + j];
        if (kj == kInf) continue;
        Weight& ij = dist[std::size_t{i} * n + j];
        if (std::uint64_t{ik} + kj < ij) ij = ik + kj;
      }
    }
  }
  return dist;
}

}
