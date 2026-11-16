#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "graph.h"
#include "heap.h"
#include "search.h"
#include "thread_pool.h"

namespace rr {

struct UpArcs {
  const std::uint32_t* first = nullptr;
  const NodeId* head = nullptr;
  const Weight* weight = nullptr;
  const NodeId* middle = nullptr;
};

class ChGraph {
 public:
  struct Parts {
    std::vector<NodeId> rank;
    std::array<std::vector<std::uint32_t>, 2> first;
    std::array<std::vector<NodeId>, 2> head;
    std::array<std::vector<Weight>, 2> weight;
    std::array<std::vector<NodeId>, 2> middle;
  };

  ChGraph() = default;
  ChGraph(ChGraph&&) = default;
  ChGraph& operator=(ChGraph&&) = default;

  static Result<ChGraph> FromParts(Parts parts);
  static Result<ChGraph> Load(const std::string& path, const Graph& graph, bool verify);
  Status Save(const std::string& path, const Graph& graph) const;

  std::uint32_t node_count() const { return node_count_; }
  std::uint64_t arc_count() const { return head_[0].size() + head_[1].size(); }
  std::uint64_t shortcut_count() const;
  NodeId Rank(NodeId v) const { return rank_[v]; }
  NodeId NodeAt(NodeId r) const { return order_[r]; }
  UpArcs up(int direction) const {
    return {first_[direction].data(), head_[direction].data(), weight_[direction].data(),
            middle_[direction].data()};
  }
  bool FindArc(NodeId from_rank, NodeId to_rank, Weight* weight, NodeId* middle) const;

 private:
  Status Validate() const;

  std::uint32_t node_count_ = 0;
  Array<NodeId> rank_;
  Array<NodeId> order_;
  std::array<Array<std::uint32_t>, 2> first_;
  std::array<Array<NodeId>, 2> head_;
  std::array<Array<Weight>, 2> weight_;
  std::array<Array<NodeId>, 2> middle_;
};

class UpwardSearch {
 public:
  UpwardSearch();

  void Reset(NodeId start);
  bool Empty() const { return heap_.Empty(); }
  Weight MinKey() const { return heap_.MinKey(); }
  NodeId Pop(Weight* dist);
  void Relax(NodeId node, Weight dist, NodeId parent);
  Weight Dist(NodeId node) const {
    const Slot& s = slots_[Find(node)];
    return s.node == node ? s.dist : kInf;
  }
  NodeId Parent(NodeId node) const {
    const Slot& s = slots_[Find(node)];
    return s.node == node ? s.parent : kNoNode;
  }

 private:
  struct Slot {
    NodeId node = kNoNode;
    Weight dist = kInf;
    NodeId parent = kNoNode;
    std::uint32_t settled = 0;
  };

  std::uint32_t Find(NodeId node) const {
    std::uint32_t i = (node * 0x9e3779b1u) >> shift_;
    while (slots_[i].node != node && slots_[i].node != kNoNode) i = (i + 1) & mask_;
    return i;
  }
  void Grow();

  std::vector<Slot> slots_;
  std::vector<std::uint32_t> used_;
  MinHeap<Weight> heap_;
  std::uint32_t mask_ = 0;
  int shift_ = 0;
};

class ChQuery {
 public:
  explicit ChQuery(const ChGraph& ch) : ch_(&ch) {}

  Weight Run(NodeId source, NodeId target);
  std::vector<NodeId> Path() const;
  const SearchStats& stats() const { return stats_; }

 private:
  const ChGraph* ch_;
  std::array<UpwardSearch, 2> search_;
  SearchStats stats_;
  NodeId source_ = kNoNode;
  NodeId target_ = kNoNode;
  NodeId meet_ = kNoNode;
  Weight best_ = kInf;
};

class ChTable {
 public:
  explicit ChTable(const ChGraph& ch) : ch_(&ch) {}

  std::vector<Weight> Compute(std::span<const NodeId> sources, std::span<const NodeId> targets,
                              ThreadPool& pool) const;

 private:
  const ChGraph* ch_;
};

bool Stalled(const ChGraph& ch, int direction, const UpwardSearch& search, NodeId node,
             Weight dist);

}
