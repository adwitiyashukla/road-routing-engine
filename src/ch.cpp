#include "ch.h"

#include <algorithm>
#include <utility>

namespace rr {

namespace {

constexpr const char* kChMagic = "RRCH";
constexpr std::uint32_t kChFormat = 1;
constexpr int kInitialBits = 12;

std::uint32_t DirectionTag(const char* base, int direction) {
  char name[5] = {base[0], base[1], base[2], static_cast<char>('0' + direction), 0};
  return Tag(name);
}

template <typename Visit>
void SearchUpward(const ChGraph& ch, int direction, UpwardSearch& search, NodeId start,
                  SearchStats& stats, Visit&& visit) {
  search.Reset(start);
  UpArcs arcs = ch.up(direction);
  while (!search.Empty()) {
    Weight du = 0;
    NodeId u = search.Pop(&du);
    ++stats.settled;
    if (Stalled(ch, direction, search, u, du)) continue;
    visit(u, du);
    for (std::uint32_t a = arcs.first[u]; a < arcs.first[u + 1]; ++a) {
      std::uint64_t nd = std::uint64_t{du} + arcs.weight[a];
      ++stats.relaxed;
      if (nd < kInf) search.Relax(arcs.head[a], static_cast<Weight>(nd), u);
    }
  }
}

struct Bucket {
  NodeId node = kNoNode;
  std::uint32_t begin = 0;
  std::uint32_t end = 0;
};

}

Result<ChGraph> ChGraph::FromParts(Parts parts) {
  ChGraph ch;
  ch.node_count_ = static_cast<std::uint32_t>(parts.rank.size());
  std::vector<NodeId> order(parts.rank.size(), kNoNode);
  for (NodeId v = 0; v < ch.node_count_; ++v) {
    if (parts.rank[v] >= ch.node_count_ || order[parts.rank[v]] != kNoNode) {
      return Status::Error("contraction order is not a permutation");
    }
    order[parts.rank[v]] = v;
  }
  ch.rank_ = Array<NodeId>(std::move(parts.rank));
  ch.order_ = Array<NodeId>(std::move(order));
  for (int d = 0; d < 2; ++d) {
    ch.first_[d] = Array<std::uint32_t>(std::move(parts.first[d]));
    ch.head_[d] = Array<NodeId>(std::move(parts.head[d]));
    ch.weight_[d] = Array<Weight>(std::move(parts.weight[d]));
    ch.middle_[d] = Array<NodeId>(std::move(parts.middle[d]));
  }
  RR_RETURN_IF_ERROR(ch.Validate());
  return ch;
}

Status ChGraph::Save(const std::string& path, const Graph& graph) const {
  std::vector<std::uint32_t> meta = {node_count_,
                                     static_cast<std::uint32_t>(head_[0].size()),
                                     static_cast<std::uint32_t>(head_[1].size()),
                                     graph.identity(),
                                     kChFormat,
                                     0};
  ContainerWriter writer(kChMagic);
  writer.Add(Tag("META"), meta);
  writer.Add(Tag("RANK"), rank_.span());
  writer.Add(Tag("ORDR"), order_.span());
  for (int d = 0; d < 2; ++d) {
    writer.Add(DirectionTag("FST", d), first_[d].span());
    writer.Add(DirectionTag("HED", d), head_[d].span());
    writer.Add(DirectionTag("WGT", d), weight_[d].span());
    writer.Add(DirectionTag("MID", d), middle_[d].span());
  }
  return writer.Write(path);
}

Result<ChGraph> ChGraph::Load(const std::string& path, const Graph& graph, bool verify) {
  Result<ContainerReader> reader = ContainerReader::Open(path, kChMagic, verify);
  if (!reader.ok()) return reader.status();
  Result<Array<std::uint32_t>> meta = reader->Get<std::uint32_t>(Tag("META"), 6);
  if (!meta.ok()) return meta.status();
  const Array<std::uint32_t>& m = *meta;
  if (m[4] != kChFormat) return Status::Error(path + " has an unknown hierarchy format");
  if (m[0] != graph.node_count() || m[3] != graph.identity()) {
    return Status::Error(path + " was built for a different graph");
  }
  ChGraph ch;
  ch.node_count_ = m[0];
  Result<Array<NodeId>> rank = reader->Get<NodeId>(Tag("RANK"), ch.node_count_);
  if (!rank.ok()) return rank.status();
  Result<Array<NodeId>> order = reader->Get<NodeId>(Tag("ORDR"), ch.node_count_);
  if (!order.ok()) return order.status();
  ch.rank_ = std::move(*rank);
  ch.order_ = std::move(*order);
  for (int d = 0; d < 2; ++d) {
    Result<Array<std::uint32_t>> first =
        reader->Get<std::uint32_t>(DirectionTag("FST", d), ch.node_count_ + 1ULL);
    if (!first.ok()) return first.status();
    Result<Array<NodeId>> head = reader->Get<NodeId>(DirectionTag("HED", d), m[1 + d]);
    if (!head.ok()) return head.status();
    Result<Array<Weight>> weight = reader->Get<Weight>(DirectionTag("WGT", d), m[1 + d]);
    if (!weight.ok()) return weight.status();
    Result<Array<NodeId>> middle = reader->Get<NodeId>(DirectionTag("MID", d), m[1 + d]);
    if (!middle.ok()) return middle.status();
    ch.first_[d] = std::move(*first);
    ch.head_[d] = std::move(*head);
    ch.weight_[d] = std::move(*weight);
    ch.middle_[d] = std::move(*middle);
  }
  Status valid = ch.Validate();
  if (!valid.ok()) return Status::Error(path + ": " + valid.message());
  return ch;
}

Status ChGraph::Validate() const {
  const std::uint32_t n = node_count_;
  if (rank_.size() != n || order_.size() != n) {
    return Status::Error("hierarchy order is incomplete");
  }
  for (NodeId v = 0; v < n; ++v) {
    if (rank_[v] >= n || order_[rank_[v]] != v) {
      return Status::Error("hierarchy order is not a permutation");
    }
  }
  for (int d = 0; d < 2; ++d) {
    const std::uint32_t arcs = static_cast<std::uint32_t>(head_[d].size());
    if (first_[d].size() != std::size_t{n} + 1 || weight_[d].size() != arcs ||
        middle_[d].size() != arcs) {
      return Status::Error("hierarchy arrays have inconsistent sizes");
    }
    if (first_[d][0] != 0 || first_[d][n] != arcs) {
      return Status::Error("hierarchy offsets do not cover the arcs");
    }
    for (NodeId r = 0; r < n; ++r) {
      std::uint32_t begin = first_[d][r];
      std::uint32_t end = first_[d][r + 1];
      if (begin > end || end > arcs) return Status::Error("hierarchy offsets are not monotone");
      for (std::uint32_t a = begin; a < end; ++a) {
        if (head_[d][a] <= r || head_[d][a] >= n) {
          return Status::Error("hierarchy arc does not point upward");
        }
        if (a > begin && head_[d][a] <= head_[d][a - 1]) {
          return Status::Error("hierarchy arcs are not sorted");
        }
        if (middle_[d][a] != kNoNode && middle_[d][a] >= r) {
          return Status::Error("shortcut middle node is not below its arc");
        }
        if (weight_[d][a] == kInf) return Status::Error("hierarchy arc has no weight");
      }
    }
  }
  return Status();
}

std::uint64_t ChGraph::shortcut_count() const {
  std::uint64_t count = 0;
  for (int d = 0; d < 2; ++d) {
    for (NodeId m : middle_[d].span()) count += m != kNoNode ? 1 : 0;
  }
  return count;
}

bool ChGraph::FindArc(NodeId from_rank, NodeId to_rank, Weight* weight, NodeId* middle) const {
  int d = from_rank < to_rank ? 0 : 1;
  NodeId low = d == 0 ? from_rank : to_rank;
  NodeId high = d == 0 ? to_rank : from_rank;
  const NodeId* begin = head_[d].data() + first_[d][low];
  const NodeId* end = head_[d].data() + first_[d][low + 1];
  const NodeId* it = std::lower_bound(begin, end, high);
  if (it == end || *it != high) return false;
  std::size_t a = static_cast<std::size_t>(it - head_[d].data());
  *weight = weight_[d][a];
  *middle = middle_[d][a];
  return true;
}

UpwardSearch::UpwardSearch() {
  slots_.resize(std::size_t{1} << kInitialBits);
  mask_ = static_cast<std::uint32_t>(slots_.size() - 1);
  shift_ = 32 - kInitialBits;
  heap_.Resize(slots_.size());
}

void UpwardSearch::Reset(NodeId start) {
  for (std::uint32_t i : used_) slots_[i] = Slot{};
  used_.clear();
  heap_.Clear();
  Relax(start, 0, kNoNode);
}

NodeId UpwardSearch::Pop(Weight* dist) {
  std::uint32_t i = heap_.Pop();
  Slot& s = slots_[i];
  s.settled = 1;
  *dist = s.dist;
  return s.node;
}

void UpwardSearch::Relax(NodeId node, Weight dist, NodeId parent) {
  if (used_.size() * 2 >= slots_.size()) Grow();
  std::uint32_t i = Find(node);
  Slot& s = slots_[i];
  if (s.node == kNoNode) {
    s = {node, dist, parent, 0};
    used_.push_back(i);
    heap_.Push(i, dist);
  } else if (dist < s.dist && s.settled == 0) {
    s.dist = dist;
    s.parent = parent;
    heap_.Decrease(i, dist);
  }
}

void UpwardSearch::Grow() {
  std::vector<Slot> old = std::move(slots_);
  slots_.assign(old.size() * 2, Slot{});
  mask_ = static_cast<std::uint32_t>(slots_.size() - 1);
  --shift_;
  std::vector<std::uint32_t> moved(old.size(), 0);
  std::vector<std::uint32_t> used;
  used.reserve(used_.size());
  for (std::uint32_t i : used_) {
    std::uint32_t j = Find(old[i].node);
    slots_[j] = old[i];
    moved[i] = j;
    used.push_back(j);
  }
  used_ = std::move(used);
  heap_.Resize(slots_.size());
  for (auto& item : heap_.items()) item.id = moved[item.id];
  heap_.Reindex();
}

bool Stalled(const ChGraph& ch, int direction, const UpwardSearch& search, NodeId node,
             Weight dist) {
  UpArcs arcs = ch.up(1 - direction);
  for (std::uint32_t a = arcs.first[node]; a < arcs.first[node + 1]; ++a) {
    Weight above = search.Dist(arcs.head[a]);
    if (above != kInf && std::uint64_t{above} + arcs.weight[a] < dist) return true;
  }
  return false;
}

Weight ChQuery::Run(NodeId source, NodeId target) {
  const ChGraph& ch = *ch_;
  stats_ = {};
  source_ = source;
  target_ = target;
  best_ = kInf;
  meet_ = kNoNode;
  if (source == target) {
    best_ = 0;
    return 0;
  }
  search_[0].Reset(ch.Rank(source));
  search_[1].Reset(ch.Rank(target));
  bool active[2] = {true, true};
  int d = 0;
  while (active[0] || active[1]) {
    if (!active[d]) d = 1 - d;
    UpwardSearch& mine = search_[d];
    if (mine.Empty() || mine.MinKey() >= best_) {
      active[d] = false;
      d = 1 - d;
      continue;
    }
    Weight du = 0;
    NodeId u = mine.Pop(&du);
    ++stats_.settled;
    Weight other = search_[1 - d].Dist(u);
    if (other != kInf && std::uint64_t{du} + other < best_) {
      best_ = du + other;
      meet_ = u;
    }
    if (!Stalled(ch, d, mine, u, du)) {
      UpArcs arcs = ch.up(d);
      for (std::uint32_t a = arcs.first[u]; a < arcs.first[u + 1]; ++a) {
        std::uint64_t nd = std::uint64_t{du} + arcs.weight[a];
        ++stats_.relaxed;
        if (nd < best_) mine.Relax(arcs.head[a], static_cast<Weight>(nd), u);
      }
    }
    d = 1 - d;
  }
  return best_;
}

std::vector<NodeId> ChQuery::Path() const {
  const ChGraph& ch = *ch_;
  if (best_ == kInf) return {};
  if (source_ == target_) return {source_};
  std::vector<NodeId> ranks;
  for (NodeId r = meet_; r != kNoNode; r = search_[0].Parent(r)) ranks.push_back(r);
  std::reverse(ranks.begin(), ranks.end());
  for (NodeId r = search_[1].Parent(meet_); r != kNoNode; r = search_[1].Parent(r)) {
    ranks.push_back(r);
  }
  std::vector<NodeId> path;
  path.push_back(ch.NodeAt(ranks[0]));
  std::vector<std::pair<NodeId, NodeId>> stack;
  for (std::size_t i = 0; i + 1 < ranks.size(); ++i) {
    stack.push_back({ranks[i], ranks[i + 1]});
    while (!stack.empty()) {
      auto [from, to] = stack.back();
      stack.pop_back();
      Weight weight = 0;
      NodeId middle = kNoNode;
      if (!ch.FindArc(from, to, &weight, &middle)) return {};
      if (middle == kNoNode) {
        path.push_back(ch.NodeAt(to));
      } else {
        stack.push_back({middle, to});
        stack.push_back({from, middle});
      }
    }
  }
  return path;
}

std::vector<Weight> ChTable::Compute(std::span<const NodeId> sources,
                                     std::span<const NodeId> targets, ThreadPool& pool) const {
  const ChGraph& ch = *ch_;
  const std::size_t width = targets.size();
  std::vector<Weight> table(sources.size() * width, kInf);
  if (table.empty()) return table;
  struct Entry {
    NodeId node;
    std::uint32_t target;
    Weight dist;
  };
  const std::size_t grain = 8;
  std::vector<std::vector<Entry>> parts((targets.size() + grain - 1) / grain);
  std::vector<UpwardSearch> searches(static_cast<std::size_t>(pool.size()));
  pool.Run(targets.size(), grain, [&](std::size_t begin, std::size_t end, int worker) {
    std::vector<Entry>& out = parts[begin / grain];
    SearchStats stats;
    for (std::size_t j = begin; j < end; ++j) {
      SearchUpward(ch, 1, searches[worker], ch.Rank(targets[j]), stats, [&](NodeId u, Weight du) {
        out.push_back({u, static_cast<std::uint32_t>(j), du});
      });
    }
  });
  std::vector<Entry> entries;
  for (std::vector<Entry>& part : parts) {
    entries.insert(entries.end(), part.begin(), part.end());
    std::vector<Entry>().swap(part);
  }
  std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
    return a.node < b.node || (a.node == b.node && a.target < b.target);
  });
  std::size_t distinct = 0;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (i == 0 || entries[i].node != entries[i - 1].node) ++distinct;
  }
  std::size_t capacity = 16;
  while (capacity < distinct * 2) capacity *= 2;
  std::vector<Bucket> buckets(capacity);
  const std::uint32_t mask = static_cast<std::uint32_t>(capacity - 1);
  for (std::size_t i = 0; i < entries.size();) {
    std::size_t j = i;
    while (j < entries.size() && entries[j].node == entries[i].node) ++j;
    std::uint32_t slot = static_cast<std::uint32_t>(Mix64(entries[i].node)) & mask;
    while (buckets[slot].node != kNoNode) slot = (slot + 1) & mask;
    buckets[slot] = {entries[i].node, static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(j)};
    i = j;
  }
  pool.Run(sources.size(), grain, [&](std::size_t begin, std::size_t end, int worker) {
    SearchStats stats;
    for (std::size_t i = begin; i < end; ++i) {
      Weight* row = table.data() + i * width;
      SearchUpward(ch, 0, searches[worker], ch.Rank(sources[i]), stats, [&](NodeId u, Weight du) {
        std::uint32_t slot = static_cast<std::uint32_t>(Mix64(u)) & mask;
        while (buckets[slot].node != u) {
          if (buckets[slot].node == kNoNode) return;
          slot = (slot + 1) & mask;
        }
        for (std::uint32_t e = buckets[slot].begin; e < buckets[slot].end; ++e) {
          std::uint64_t total = std::uint64_t{du} + entries[e].dist;
          if (total < row[entries[e].target]) {
            row[entries[e].target] = static_cast<Weight>(total);
          }
        }
      });
    }
  });
  return table;
}

}
