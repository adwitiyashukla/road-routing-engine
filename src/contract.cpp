#include "contract.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <functional>
#include <numeric>
#include <utility>

#include "thread_pool.h"

namespace rr {

namespace {

constexpr std::uint32_t kOut = std::uint32_t{1} << 31;
constexpr std::uint32_t kIn = std::uint32_t{1} << 30;
constexpr std::uint32_t kBoth = kOut | kIn;
constexpr std::uint32_t kIdMask = kIn - 1;
constexpr std::uint32_t kHopLimit = 0xffff;

struct Range {
  std::uint32_t begin = 0;
  std::uint32_t size = 0;
  std::uint32_t cap = 0;
};

struct Pool {
  std::vector<Range> ranges;
  std::vector<std::uint32_t> head;
  std::vector<Weight> weight;
  std::vector<NodeId> middle;
  std::vector<std::uint16_t> hops;
  std::uint64_t used = 0;

  std::uint64_t capacity() const { return head.size(); }

  void Grow(std::uint64_t capacity) {
    head.resize(capacity);
    weight.resize(capacity);
    middle.resize(capacity);
    hops.resize(capacity);
  }

  void Move(std::uint64_t from, std::uint64_t to, std::uint32_t count) {
    std::copy_n(head.begin() + from, count, head.begin() + to);
    std::copy_n(weight.begin() + from, count, weight.begin() + to);
    std::copy_n(middle.begin() + from, count, middle.begin() + to);
    std::copy_n(hops.begin() + from, count, hops.begin() + to);
  }
};

struct Shortcut {
  NodeId from;
  NodeId to;
  Weight weight;
  NodeId middle;
  std::uint16_t hops;
};

struct Op {
  NodeId node;
  NodeId other;
  std::uint32_t dir;
  Weight weight;
  NodeId middle;
  std::uint16_t hops;
};

struct Neighbor {
  NodeId node;
  Weight weight;
  std::uint32_t hops;
};

class WitnessSearch {
 public:
  WitnessSearch() { Reset(std::size_t{1} << 12); }

  void Run(const Pool& pool, NodeId source, NodeId avoid, const std::uint8_t* blocked,
           const std::vector<NodeId>& targets, Weight limit, std::uint32_t settle_limit) {
    for (std::uint32_t i : used_) slots_[i] = Slot{};
    used_.clear();
    heap_.clear();
    std::uint32_t pending = 0;
    for (NodeId t : targets) {
      Slot& s = Insert(t);
      if (s.target == 0) {
        s.target = 1;
        ++pending;
      }
    }
    Insert(source).dist = 0;
    heap_.push_back(std::uint64_t{source});
    std::uint32_t settled = 0;
    while (!heap_.empty()) {
      std::pop_heap(heap_.begin(), heap_.end(), std::greater<>());
      std::uint64_t top = heap_.back();
      heap_.pop_back();
      Weight d = static_cast<Weight>(top >> 32);
      NodeId x = static_cast<NodeId>(top & 0xffffffff);
      Slot& s = slots_[Find(x)];
      if (s.settled != 0 || d > s.dist) continue;
      if (d > limit) break;
      s.settled = 1;
      if (s.target != 0 && --pending == 0) break;
      if (++settled > settle_limit) break;
      const Range& r = pool.ranges[x];
      for (std::uint32_t i = r.begin; i < r.begin + r.size; ++i) {
        std::uint32_t h = pool.head[i];
        if ((h & kOut) == 0) continue;
        NodeId y = h & kIdMask;
        if (y == avoid || (blocked != nullptr && blocked[y] != 0)) continue;
        std::uint64_t nd = std::uint64_t{d} + pool.weight[i];
        if (nd > limit) continue;
        Slot& t = Insert(y);
        if (nd < t.dist) {
          t.dist = static_cast<Weight>(nd);
          heap_.push_back(nd << 32 | y);
          std::push_heap(heap_.begin(), heap_.end(), std::greater<>());
        }
      }
    }
  }

  Weight Dist(NodeId v) const {
    const Slot& s = slots_[Find(v)];
    return s.node == v ? s.dist : kInf;
  }

 private:
  struct Slot {
    NodeId node = kNoNode;
    Weight dist = kInf;
    std::uint8_t target = 0;
    std::uint8_t settled = 0;
  };

  void Reset(std::size_t size) {
    slots_.assign(size, Slot{});
    used_.clear();
    mask_ = static_cast<std::uint32_t>(size - 1);
    shift_ = 32;
    while ((std::size_t{1} << (32 - shift_)) < size) --shift_;
  }

  std::uint32_t Find(NodeId v) const {
    std::uint32_t i = (v * 0x9e3779b1u) >> shift_;
    while (slots_[i].node != v && slots_[i].node != kNoNode) i = (i + 1) & mask_;
    return i;
  }

  Slot& Insert(NodeId v) {
    if (used_.size() * 2 >= slots_.size()) {
      std::vector<Slot> old = std::move(slots_);
      std::vector<std::uint32_t> old_used = std::move(used_);
      Reset(old.size() * 2);
      for (std::uint32_t i : old_used) {
        std::uint32_t j = Find(old[i].node);
        slots_[j] = old[i];
        used_.push_back(j);
      }
    }
    std::uint32_t i = Find(v);
    if (slots_[i].node == kNoNode) {
      slots_[i].node = v;
      used_.push_back(i);
    }
    return slots_[i];
  }

  std::vector<Slot> slots_;
  std::vector<std::uint32_t> used_;
  std::vector<std::uint64_t> heap_;
  std::uint32_t mask_ = 0;
  int shift_ = 32;
};

struct Worker {
  WitnessSearch search;
  std::vector<Neighbor> ins;
  std::vector<Neighbor> outs;
  std::vector<NodeId> targets;
  std::vector<Shortcut> scratch;
  std::uint64_t searches = 0;
};

class Contractor {
 public:
  Contractor(const Graph& graph, const ContractOptions& options)
      : graph_(graph),
        options_(options),
        pool_(options.threads),
        n_(graph.node_count()),
        priority_(n_, 0),
        level_(n_, 0),
        rank_(n_, kNoNode),
        in_batch_(n_, 0),
        workers_(static_cast<std::size_t>(pool_.size())) {}

  Result<ChGraph> Run(ContractReport* report);

 private:
  void BuildPool();
  void ComputeShortcuts(NodeId v, const std::uint8_t* blocked, Worker& worker,
                        std::vector<Shortcut>& out);
  float Priority(NodeId v, Worker& worker);
  bool IsLocalMin(NodeId v) const;
  bool EnsureCapacity(const std::vector<NodeId>& nodes, const std::vector<std::uint32_t>& required);
  void Compact();
  void ApplyOps(NodeId x, const Op* begin, const Op* end);
  void Insert(NodeId x, const Op& op);
  void Output(const std::vector<NodeId>& batch);
  Result<ChGraph> Assemble();

  static std::uint64_t GrowSize(std::uint32_t required) {
    return std::uint64_t{required} + required / 2 + 2;
  }

  const Graph& graph_;
  ContractOptions options_;
  ThreadPool pool_;
  std::uint32_t n_;
  Pool dyn_;
  std::vector<float> priority_;
  std::vector<std::uint32_t> level_;
  std::vector<NodeId> rank_;
  std::vector<std::uint8_t> in_batch_;
  std::vector<Worker> workers_;
  std::atomic<bool> overflow_{false};
  NodeId next_rank_ = 0;
  std::uint32_t max_degree_ = 0;
  std::uint64_t peak_entries_ = 0;
  std::array<std::vector<std::uint32_t>, 2> out_count_;
  std::array<std::vector<NodeId>, 2> out_head_;
  std::array<std::vector<Weight>, 2> out_weight_;
  std::array<std::vector<NodeId>, 2> out_middle_;
};

void Contractor::BuildPool() {
  ReverseGraph reverse(graph_);
  Adjacency fwd = graph_.forward();
  Adjacency bwd = reverse.view();
  std::vector<std::uint32_t> first(std::size_t{n_} + 1, 0);
  auto merge = [&](NodeId v, const std::function<void(NodeId, std::uint32_t, Weight)>& emit) {
    std::uint32_t i = fwd.first[v];
    std::uint32_t j = bwd.first[v];
    const std::uint32_t fe = fwd.first[v + 1];
    const std::uint32_t be = bwd.first[v + 1];
    while (i < fe || j < be) {
      if (j == be || (i < fe && fwd.other[i] < bwd.other[j])) {
        emit(fwd.other[i], kOut, fwd.weight[i]);
        ++i;
      } else if (i == fe || bwd.other[j] < fwd.other[i]) {
        emit(bwd.other[j], kIn, bwd.weight[j]);
        ++j;
      } else {
        if (fwd.weight[i] == bwd.weight[j]) {
          emit(fwd.other[i], kBoth, fwd.weight[i]);
        } else {
          emit(fwd.other[i], kOut, fwd.weight[i]);
          emit(bwd.other[j], kIn, bwd.weight[j]);
        }
        ++i;
        ++j;
      }
    }
  };
  pool_.Run(n_, 4096, [&](std::size_t begin, std::size_t end, int) {
    for (std::size_t v = begin; v < end; ++v) {
      std::uint32_t count = 0;
      merge(static_cast<NodeId>(v), [&](NodeId, std::uint32_t, Weight) { ++count; });
      first[v + 1] = count;
    }
  });
  std::partial_sum(first.begin(), first.end(), first.begin());
  const std::uint64_t total = first[n_];
  dyn_.Grow(total + total / 4 + (1 << 16));
  dyn_.ranges.resize(n_);
  pool_.Run(n_, 4096, [&](std::size_t begin, std::size_t end, int) {
    for (std::size_t v = begin; v < end; ++v) {
      std::uint32_t pos = first[v];
      merge(static_cast<NodeId>(v), [&](NodeId other, std::uint32_t dir, Weight w) {
        dyn_.head[pos] = other | dir;
        dyn_.weight[pos] = w;
        dyn_.middle[pos] = kNoNode;
        dyn_.hops[pos] = 1;
        ++pos;
      });
      dyn_.ranges[v] = {first[v], first[v + 1] - first[v], first[v + 1] - first[v]};
    }
  });
  dyn_.used = total;
  peak_entries_ = dyn_.capacity();
}

void Contractor::ComputeShortcuts(NodeId v, const std::uint8_t* blocked, Worker& worker,
                                  std::vector<Shortcut>& out) {
  const Range& r = dyn_.ranges[v];
  worker.ins.clear();
  worker.outs.clear();
  for (std::uint32_t i = r.begin; i < r.begin + r.size; ++i) {
    std::uint32_t h = dyn_.head[i];
    Neighbor nb{h & kIdMask, dyn_.weight[i], dyn_.hops[i]};
    if ((h & kIn) != 0) worker.ins.push_back(nb);
    if ((h & kOut) != 0) worker.outs.push_back(nb);
  }
  if (worker.ins.empty() || worker.outs.empty()) return;
  for (const Neighbor& in : worker.ins) {
    worker.targets.clear();
    Weight max_out = 0;
    for (const Neighbor& o : worker.outs) {
      if (o.node == in.node) continue;
      worker.targets.push_back(o.node);
      max_out = std::max(max_out, o.weight);
    }
    if (worker.targets.empty()) continue;
    std::uint64_t limit = std::uint64_t{in.weight} + max_out;
    if (limit >= kInf) {
      overflow_ = true;
      limit = kInf - 1;
    }
    worker.search.Run(dyn_, in.node, v, blocked, worker.targets, static_cast<Weight>(limit),
                      options_.settle_limit);
    ++worker.searches;
    for (const Neighbor& o : worker.outs) {
      if (o.node == in.node) continue;
      std::uint64_t cost = std::uint64_t{in.weight} + o.weight;
      if (worker.search.Dist(o.node) <= cost) continue;
      if (cost >= kInf) {
        overflow_ = true;
        continue;
      }
      std::uint32_t hops = std::min(in.hops + o.hops, kHopLimit);
      out.push_back(
          {in.node, o.node, static_cast<Weight>(cost), v, static_cast<std::uint16_t>(hops)});
    }
  }
}

float Contractor::Priority(NodeId v, Worker& worker) {
  const Range& r = dyn_.ranges[v];
  std::uint64_t removed = 0;
  std::uint64_t removed_hops = 0;
  for (std::uint32_t i = r.begin; i < r.begin + r.size; ++i) {
    std::uint32_t h = dyn_.head[i];
    int directions = ((h & kOut) != 0 ? 1 : 0) + ((h & kIn) != 0 ? 1 : 0);
    removed += directions;
    removed_hops += std::uint64_t{dyn_.hops[i]} * directions;
  }
  worker.scratch.clear();
  ComputeShortcuts(v, nullptr, worker, worker.scratch);
  std::uint64_t added_hops = 0;
  for (const Shortcut& s : worker.scratch) added_hops += s.hops;
  double p = level_[v];
  if (removed > 0) p += static_cast<double>(worker.scratch.size()) / static_cast<double>(removed);
  if (removed_hops > 0) p += static_cast<double>(added_hops) / static_cast<double>(removed_hops);
  return static_cast<float>(p);
}

bool Contractor::IsLocalMin(NodeId v) const {
  const Range& r = dyn_.ranges[v];
  const float pv = priority_[v];
  const std::uint64_t mv = Mix64(v);
  for (std::uint32_t i = r.begin; i < r.begin + r.size; ++i) {
    NodeId x = dyn_.head[i] & kIdMask;
    float px = priority_[x];
    if (px < pv || (px == pv && Mix64(x) < mv)) return false;
  }
  return true;
}

void Contractor::Compact() {
  std::vector<std::pair<std::uint32_t, NodeId>> order;
  for (NodeId v = 0; v < n_; ++v) {
    if (dyn_.ranges[v].cap > 0) order.push_back({dyn_.ranges[v].begin, v});
  }
  std::sort(order.begin(), order.end());
  std::uint64_t pos = 0;
  for (const auto& [begin, v] : order) {
    Range& r = dyn_.ranges[v];
    if (r.size > 0 && pos != begin) dyn_.Move(begin, pos, r.size);
    r.begin = static_cast<std::uint32_t>(pos);
    r.cap = r.size;
    pos += r.size;
  }
  for (NodeId v = 0; v < n_; ++v) {
    if (dyn_.ranges[v].cap == 0) dyn_.ranges[v].begin = static_cast<std::uint32_t>(pos);
  }
  dyn_.used = pos;
}

bool Contractor::EnsureCapacity(const std::vector<NodeId>& nodes,
                                const std::vector<std::uint32_t>& required) {
  auto needed = [&] {
    std::uint64_t total = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (required[i] > dyn_.ranges[nodes[i]].cap) total += GrowSize(required[i]);
    }
    return total;
  };
  std::uint64_t need = needed();
  if (dyn_.used + need > dyn_.capacity()) {
    Compact();
    need = needed();
    if (dyn_.used + need > dyn_.capacity()) {
      const std::uint64_t limit = 0xffffffffULL;
      if (dyn_.used + need > limit) return false;
      std::uint64_t capacity =
          std::max(dyn_.used + need + (1 << 16), dyn_.capacity() + dyn_.capacity() / 4);
      dyn_.Grow(std::min(capacity, limit));
      peak_entries_ = std::max(peak_entries_, dyn_.capacity());
    }
  }
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    Range& r = dyn_.ranges[nodes[i]];
    if (required[i] <= r.cap) continue;
    std::uint64_t cap = GrowSize(required[i]);
    if (r.size > 0) dyn_.Move(r.begin, dyn_.used, r.size);
    r.begin = static_cast<std::uint32_t>(dyn_.used);
    r.cap = static_cast<std::uint32_t>(cap);
    dyn_.used += cap;
  }
  return true;
}

void Contractor::ApplyOps(NodeId x, const Op* begin, const Op* end) {
  Range& r = dyn_.ranges[x];
  std::uint32_t kept = 0;
  for (std::uint32_t i = 0; i < r.size; ++i) {
    std::uint32_t p = r.begin + i;
    if (in_batch_[dyn_.head[p] & kIdMask] != 0) continue;
    if (kept != i) dyn_.Move(p, r.begin + kept, 1);
    ++kept;
  }
  r.size = kept;
  for (const Op* op = begin; op != end; ++op) Insert(x, *op);
}

void Contractor::Insert(NodeId x, const Op& op) {
  Range& r = dyn_.ranges[x];
  std::uint32_t* h = dyn_.head.data() + r.begin;
  Weight* w = dyn_.weight.data() + r.begin;
  NodeId* m = dyn_.middle.data() + r.begin;
  std::uint16_t* hp = dyn_.hops.data() + r.begin;
  std::int64_t same = -1;
  std::int64_t opposite = -1;
  for (std::uint32_t i = 0; i < r.size; ++i) {
    if ((h[i] & kIdMask) != op.other) continue;
    if ((h[i] & op.dir) != 0) {
      same = i;
    } else {
      opposite = i;
    }
  }
  auto matches = [&](std::int64_t i) {
    return i >= 0 && w[i] == op.weight && m[i] == op.middle && hp[i] == op.hops;
  };
  auto append = [&] {
    std::uint32_t i = r.size++;
    h[i] = op.other | op.dir;
    w[i] = op.weight;
    m[i] = op.middle;
    hp[i] = op.hops;
  };
  if (same >= 0) {
    if (w[same] <= op.weight) return;
    if ((h[same] & kBoth) == kBoth) {
      h[same] &= ~op.dir;
      append();
      return;
    }
    w[same] = op.weight;
    m[same] = op.middle;
    hp[same] = op.hops;
    if (matches(opposite)) {
      h[opposite] |= op.dir;
      std::uint32_t last = r.size - 1;
      if (static_cast<std::uint32_t>(same) != last) {
        dyn_.Move(r.begin + last, r.begin + static_cast<std::uint32_t>(same), 1);
      }
      --r.size;
    }
    return;
  }
  if (matches(opposite)) {
    h[opposite] |= op.dir;
    return;
  }
  append();
}

void Contractor::Output(const std::vector<NodeId>& batch) {
  std::array<std::vector<std::uint32_t>, 2> offset;
  std::array<std::size_t, 2> base = {out_head_[0].size(), out_head_[1].size()};
  for (int d = 0; d < 2; ++d) offset[d].assign(batch.size() + 1, 0);
  for (std::size_t i = 0; i < batch.size(); ++i) {
    NodeId v = batch[i];
    const Range& r = dyn_.ranges[v];
    rank_[v] = next_rank_ + static_cast<NodeId>(i);
    max_degree_ = std::max(max_degree_, r.size);
    std::uint32_t outs = 0;
    std::uint32_t ins = 0;
    for (std::uint32_t p = r.begin; p < r.begin + r.size; ++p) {
      outs += (dyn_.head[p] & kOut) != 0 ? 1 : 0;
      ins += (dyn_.head[p] & kIn) != 0 ? 1 : 0;
    }
    out_count_[0][rank_[v]] = outs;
    out_count_[1][rank_[v]] = ins;
    offset[0][i + 1] = offset[0][i] + outs;
    offset[1][i + 1] = offset[1][i] + ins;
  }
  for (int d = 0; d < 2; ++d) {
    std::size_t size = base[d] + offset[d][batch.size()];
    out_head_[d].resize(size);
    out_weight_[d].resize(size);
    out_middle_[d].resize(size);
  }
  pool_.Run(batch.size(), 256, [&](std::size_t begin, std::size_t end, int) {
    for (std::size_t i = begin; i < end; ++i) {
      const Range& r = dyn_.ranges[batch[i]];
      std::array<std::size_t, 2> pos = {base[0] + offset[0][i], base[1] + offset[1][i]};
      for (std::uint32_t p = r.begin; p < r.begin + r.size; ++p) {
        std::uint32_t h = dyn_.head[p];
        for (int d = 0; d < 2; ++d) {
          if ((h & (d == 0 ? kOut : kIn)) == 0) continue;
          out_head_[d][pos[d]] = h & kIdMask;
          out_weight_[d][pos[d]] = dyn_.weight[p];
          out_middle_[d][pos[d]] = dyn_.middle[p];
          ++pos[d];
        }
      }
    }
  });
  for (NodeId v : batch) dyn_.ranges[v] = Range{dyn_.ranges[v].begin, 0, 0};
  next_rank_ += static_cast<NodeId>(batch.size());
}

Result<ChGraph> Contractor::Assemble() {
  dyn_ = Pool{};
  ChGraph::Parts parts;
  for (int d = 0; d < 2; ++d) {
    std::vector<std::uint32_t>& first = parts.first[d];
    first.assign(std::size_t{n_} + 1, 0);
    for (NodeId r = 0; r < n_; ++r) first[r + 1] = first[r] + out_count_[d][r];
    std::vector<std::uint32_t>().swap(out_count_[d]);
    std::vector<NodeId>& head = out_head_[d];
    std::vector<Weight>& weight = out_weight_[d];
    std::vector<NodeId>& middle = out_middle_[d];
    pool_.Run(n_, 4096, [&](std::size_t begin, std::size_t end, int) {
      struct Item {
        NodeId head;
        Weight weight;
        NodeId middle;
      };
      std::vector<Item> items;
      for (std::size_t r = begin; r < end; ++r) {
        std::uint32_t a = first[r];
        std::uint32_t b = first[r + 1];
        items.clear();
        for (std::uint32_t i = a; i < b; ++i) {
          items.push_back(
              {rank_[head[i]], weight[i], middle[i] == kNoNode ? kNoNode : rank_[middle[i]]});
        }
        std::sort(items.begin(), items.end(),
                  [](const Item& x, const Item& y) { return x.head < y.head; });
        for (std::uint32_t i = a; i < b; ++i) {
          head[i] = items[i - a].head;
          weight[i] = items[i - a].weight;
          middle[i] = items[i - a].middle;
        }
      }
    });
    parts.head[d] = std::move(head);
    parts.weight[d] = std::move(weight);
    parts.middle[d] = std::move(middle);
  }
  parts.rank = std::move(rank_);
  return ChGraph::FromParts(std::move(parts));
}

Result<ChGraph> Contractor::Run(ContractReport* report) {
  Timer timer;
  for (int d = 0; d < 2; ++d) out_count_[d].assign(n_, 0);
  BuildPool();
  pool_.Run(n_, 512, [&](std::size_t begin, std::size_t end, int worker) {
    for (std::size_t v = begin; v < end; ++v) {
      priority_[v] = Priority(static_cast<NodeId>(v), workers_[worker]);
    }
  });
  std::vector<NodeId> remaining(n_);
  std::iota(remaining.begin(), remaining.end(), NodeId{0});
  std::vector<std::uint8_t> selected;
  std::vector<NodeId> batch;
  std::vector<NodeId> touched;
  std::vector<Op> ops;
  std::vector<std::uint32_t> op_begin;
  std::vector<std::uint32_t> required;
  std::uint32_t rounds = 0;
  double last_report = 0;
  while (!remaining.empty()) {
    ++rounds;
    selected.assign(remaining.size(), 0);
    pool_.Run(remaining.size(), 4096, [&](std::size_t begin, std::size_t end, int) {
      for (std::size_t i = begin; i < end; ++i) selected[i] = IsLocalMin(remaining[i]) ? 1 : 0;
    });
    batch.clear();
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      if (selected[i] != 0) batch.push_back(remaining[i]);
    }
    for (NodeId v : batch) in_batch_[v] = 1;

    const std::size_t grain = 32;
    std::vector<std::vector<Shortcut>> parts((batch.size() + grain - 1) / grain);
    pool_.Run(batch.size(), grain, [&](std::size_t begin, std::size_t end, int worker) {
      std::vector<Shortcut>& out = parts[begin / grain];
      for (std::size_t i = begin; i < end; ++i) {
        ComputeShortcuts(batch[i], in_batch_.data(), workers_[worker], out);
      }
    });
    ops.clear();
    for (std::vector<Shortcut>& part : parts) {
      for (const Shortcut& s : part) {
        ops.push_back({s.from, s.to, kOut, s.weight, s.middle, s.hops});
        ops.push_back({s.to, s.from, kIn, s.weight, s.middle, s.hops});
      }
      std::vector<Shortcut>().swap(part);
    }
    std::stable_sort(ops.begin(), ops.end(),
                     [](const Op& a, const Op& b) { return a.node < b.node; });

    touched.clear();
    for (NodeId v : batch) {
      const Range& r = dyn_.ranges[v];
      for (std::uint32_t p = r.begin; p < r.begin + r.size; ++p) {
        touched.push_back(dyn_.head[p] & kIdMask);
      }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    op_begin.assign(touched.size() + 1, 0);
    {
      std::size_t k = 0;
      for (std::size_t i = 0; i < touched.size(); ++i) {
        while (k < ops.size() && ops[k].node < touched[i]) ++k;
        op_begin[i] = static_cast<std::uint32_t>(k);
      }
      op_begin[touched.size()] = static_cast<std::uint32_t>(ops.size());
    }
    required.assign(touched.size(), 0);
    pool_.Run(touched.size(), 1024, [&](std::size_t begin, std::size_t end, int) {
      for (std::size_t i = begin; i < end; ++i) {
        NodeId x = touched[i];
        const Range& r = dyn_.ranges[x];
        std::uint32_t removed = 0;
        std::uint32_t level = level_[x];
        for (std::uint32_t p = r.begin; p < r.begin + r.size; ++p) {
          NodeId y = dyn_.head[p] & kIdMask;
          if (in_batch_[y] == 0) continue;
          ++removed;
          level = std::max(level, level_[y] + 1);
        }
        level_[x] = level;
        required[i] = r.size - removed + (op_begin[i + 1] - op_begin[i]);
      }
    });
    if (!EnsureCapacity(touched, required)) {
      return Status::Error("the hierarchy needs more than 4 billion arc slots");
    }
    pool_.Run(touched.size(), 256, [&](std::size_t begin, std::size_t end, int) {
      for (std::size_t i = begin; i < end; ++i) {
        ApplyOps(touched[i], ops.data() + op_begin[i], ops.data() + op_begin[i + 1]);
      }
    });
    Output(batch);
    for (NodeId v : batch) in_batch_[v] = 0;
    pool_.Run(touched.size(), 128, [&](std::size_t begin, std::size_t end, int worker) {
      for (std::size_t i = begin; i < end; ++i) {
        priority_[touched[i]] = Priority(touched[i], workers_[worker]);
      }
    });
    remaining.erase(std::remove_if(remaining.begin(), remaining.end(),
                                   [&](NodeId v) { return rank_[v] != kNoNode; }),
                    remaining.end());
    if (options_.verbose && (timer.Seconds() - last_report >= 10 || remaining.empty())) {
      last_report = timer.Seconds();
      std::fprintf(stderr, "  round %u: %zu nodes left, last batch %zu, %.1f s\n", rounds,
                   remaining.size(), batch.size(), last_report);
    }
    if (overflow_) return Status::Error("shortcut weights do not fit in 32 bits");
  }
  std::uint64_t searches = 0;
  for (const Worker& w : workers_) searches += w.searches;
  Result<ChGraph> ch = Assemble();
  if (report != nullptr) {
    report->rounds = rounds;
    report->witness_searches = searches;
    report->max_degree = max_degree_;
    report->peak_entries = peak_entries_;
    report->seconds = timer.Seconds();
  }
  return ch;
}

}

Result<ChGraph> Contract(const Graph& graph, const ContractOptions& options,
                         ContractReport* report) {
  Contractor contractor(graph, options);
  return contractor.Run(report);
}

}
