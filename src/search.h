#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "graph.h"
#include "heap.h"
#include "thread_pool.h"

namespace rr {

struct SearchStats {
  std::uint64_t settled = 0;
  std::uint64_t relaxed = 0;
};

class Labels {
 public:
  explicit Labels(std::uint32_t node_count) : entries_(node_count) {}

  void NewRound() {
    if (++round_ == 0) {
      for (Entry& e : entries_) e = Entry{};
      round_ = 1;
    }
  }
  bool Reached(NodeId v) const { return entries_[v].reached == round_; }
  bool Settled(NodeId v) const { return entries_[v].settled == round_; }
  Weight Dist(NodeId v) const { return Reached(v) ? entries_[v].dist : kInf; }
  NodeId Parent(NodeId v) const { return entries_[v].parent; }
  void Set(NodeId v, Weight dist, NodeId parent) {
    Entry& e = entries_[v];
    e.dist = dist;
    e.parent = parent;
    e.reached = round_;
  }
  void Settle(NodeId v) { entries_[v].settled = round_; }
  std::vector<NodeId> PathTo(NodeId v) const;

 private:
  struct Entry {
    Weight dist = kInf;
    NodeId parent = kNoNode;
    std::uint32_t reached = 0;
    std::uint32_t settled = 0;
  };
  std::vector<Entry> entries_;
  std::uint32_t round_ = 0;
};

class Dijkstra {
 public:
  explicit Dijkstra(Adjacency graph);

  Weight Run(NodeId source, NodeId target);
  std::vector<NodeId> Path() const { return labels_.PathTo(target_); }
  const SearchStats& stats() const { return stats_; }

  template <typename Visit>
  void RunFull(std::span<const NodeId> sources, Visit&& visit) {
    Start(sources);
    while (!heap_.Empty()) {
      Weight d = heap_.MinKey();
      NodeId v = SettleNext();
      if (!visit(v, d)) break;
    }
  }

  Weight Dist(NodeId v) const { return labels_.Dist(v); }
  NodeId Parent(NodeId v) const { return labels_.Parent(v); }
  std::vector<NodeId> PathTo(NodeId v) const { return labels_.PathTo(v); }

 private:
  void Start(std::span<const NodeId> sources);
  NodeId SettleNext();

  Adjacency graph_;
  Labels labels_;
  MinHeap<Weight> heap_;
  SearchStats stats_;
  NodeId target_ = kNoNode;
};

class BidirectionalDijkstra {
 public:
  BidirectionalDijkstra(Adjacency forward, Adjacency backward);

  Weight Run(NodeId source, NodeId target);
  std::vector<NodeId> Path() const;
  const SearchStats& stats() const { return stats_; }

 private:
  std::array<Adjacency, 2> graph_;
  std::array<Labels, 2> labels_;
  std::array<MinHeap<Weight>, 2> heap_;
  SearchStats stats_;
  NodeId meet_ = kNoNode;
  Weight best_ = kInf;
};

double GeoScale(const Graph& graph);

class GeoPotential {
 public:
  GeoPotential(const Graph& graph, double scale) : coords_(graph.coords()), scale_(scale) {}

  void Prepare(NodeId source, NodeId target) {
    source_ = coords_[source];
    target_ = coords_[target];
  }
  std::int64_t ToTarget(NodeId v) const { return Bound(coords_[v], target_); }
  std::int64_t FromSource(NodeId v) const { return Bound(source_, coords_[v]); }

 private:
  std::int64_t Bound(LatLon a, LatLon b) const;

  std::span<const LatLon> coords_;
  double scale_;
  LatLon source_;
  LatLon target_;
};

class Landmarks {
 public:
  static Result<Landmarks> Build(const Graph& graph, std::uint32_t count, std::uint64_t seed,
                                 ThreadPool& pool);
  static Result<Landmarks> Load(const std::string& path, const Graph& graph);
  Status Save(const std::string& path, const Graph& graph) const;

  std::uint32_t count() const { return count_; }
  std::span<const NodeId> nodes() const { return nodes_.span(); }
  const Weight* Row(NodeId v) const {
    return table_.data() + static_cast<std::size_t>(v) * 2 * count_;
  }

 private:
  std::uint32_t count_ = 0;
  std::uint32_t node_count_ = 0;
  Array<NodeId> nodes_;
  Array<Weight> table_;
};

class LandmarkPotential {
 public:
  LandmarkPotential(const Landmarks& landmarks, std::uint32_t active)
      : landmarks_(&landmarks), active_limit_(active) {}

  void Prepare(NodeId source, NodeId target);
  std::int64_t ToTarget(NodeId v) const;
  std::int64_t FromSource(NodeId v) const;

 private:
  const Landmarks* landmarks_;
  std::uint32_t active_limit_;
  std::uint32_t active_count_ = 0;
  std::array<std::uint32_t, 64> active_{};
  const Weight* source_row_ = nullptr;
  const Weight* target_row_ = nullptr;
};

template <typename Potential>
class AStar {
 public:
  AStar(Adjacency graph, Potential potential);

  Weight Run(NodeId source, NodeId target);
  std::vector<NodeId> Path() const { return labels_.PathTo(target_); }
  const SearchStats& stats() const { return stats_; }

 private:
  Adjacency graph_;
  Potential potential_;
  Labels labels_;
  std::vector<std::int64_t> pi_;
  MinHeap<std::uint64_t> heap_;
  SearchStats stats_;
  NodeId target_ = kNoNode;
};

template <typename Potential>
class BidirectionalAStar {
 public:
  BidirectionalAStar(Adjacency forward, Adjacency backward, Potential potential);

  Weight Run(NodeId source, NodeId target);
  std::vector<NodeId> Path() const;
  const SearchStats& stats() const { return stats_; }

 private:
  std::int64_t Offset(NodeId v);

  std::array<Adjacency, 2> graph_;
  Potential potential_;
  std::array<Labels, 2> labels_;
  std::array<MinHeap<std::int64_t>, 2> heap_;
  std::vector<std::int64_t> offset_;
  std::vector<std::uint32_t> offset_round_;
  std::uint32_t round_ = 0;
  SearchStats stats_;
  NodeId meet_ = kNoNode;
  Weight best_ = kInf;
};

extern template class AStar<GeoPotential>;
extern template class AStar<LandmarkPotential>;
extern template class BidirectionalAStar<GeoPotential>;
extern template class BidirectionalAStar<LandmarkPotential>;

}
