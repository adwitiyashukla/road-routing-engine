#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "base.h"
#include "io.h"

namespace rr {

struct Arc {
  NodeId tail;
  NodeId head;
  Weight weight;
};

struct Adjacency {
  const std::uint32_t* first = nullptr;
  const NodeId* other = nullptr;
  const Weight* weight = nullptr;
  std::uint32_t node_count = 0;
};

class Graph {
 public:
  Graph() = default;
  Graph(Graph&&) = default;
  Graph& operator=(Graph&&) = default;

  static Result<Graph> FromArcs(std::uint32_t node_count, std::vector<Arc> arcs,
                                std::vector<LatLon> coords);
  static Result<Graph> Load(const std::string& path, bool verify);
  Status Save(const std::string& path) const;

  std::uint32_t node_count() const { return node_count_; }
  std::uint32_t arc_count() const { return arc_count_; }
  Adjacency forward() const {
    return {first_out_.data(), head_.data(), weight_.data(), node_count_};
  }
  LatLon coord(NodeId v) const { return coords_[v]; }
  std::span<const LatLon> coords() const { return coords_.span(); }
  std::uint32_t identity() const { return identity_; }

  Weight ArcWeight(NodeId from, NodeId to) const;
  std::vector<Arc> Arcs() const;

 private:
  std::vector<std::uint32_t> Meta() const;
  Status Validate() const;
  void ComputeIdentity();

  std::uint32_t node_count_ = 0;
  std::uint32_t arc_count_ = 0;
  Array<std::uint32_t> first_out_;
  Array<NodeId> head_;
  Array<Weight> weight_;
  Array<LatLon> coords_;
  std::uint32_t identity_ = 0;
};

class ReverseGraph {
 public:
  explicit ReverseGraph(const Graph& graph);

  Adjacency view() const { return {first_in_.data(), tail_.data(), weight_.data(), node_count_}; }

 private:
  std::uint32_t node_count_ = 0;
  std::vector<std::uint32_t> first_in_;
  std::vector<NodeId> tail_;
  std::vector<Weight> weight_;
};

struct ComponentInfo {
  std::uint32_t components = 0;
  std::uint32_t kept_nodes = 0;
};

Result<Graph> LargestStrongComponent(const Graph& graph, ComponentInfo* info);
Result<Graph> Renumber(const Graph& graph, std::span<const NodeId> new_id);
std::vector<NodeId> HilbertNumbering(const Graph& graph);

}
