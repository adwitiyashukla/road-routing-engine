#include "search.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "geo.h"

namespace rr {

namespace {

constexpr const char* kLandmarkMagic = "RRLMARK";
constexpr std::uint32_t kLandmarkFormat = 1;
constexpr std::uint32_t kMaxLandmarks = 64;
constexpr std::int64_t kNoKey = std::numeric_limits<std::int64_t>::max() / 4;

}

std::vector<NodeId> Labels::PathTo(NodeId v) const {
  std::vector<NodeId> path;
  if (v >= entries_.size() || !Reached(v)) return path;
  for (NodeId x = v; x != kNoNode; x = entries_[x].parent) path.push_back(x);
  std::reverse(path.begin(), path.end());
  return path;
}

Dijkstra::Dijkstra(Adjacency graph) : graph_(graph), labels_(graph.node_count) {
  heap_.Resize(graph.node_count);
}

void Dijkstra::Start(std::span<const NodeId> sources) {
  labels_.NewRound();
  heap_.Clear();
  stats_ = {};
  for (NodeId s : sources) {
    if (labels_.Reached(s)) continue;
    labels_.Set(s, 0, kNoNode);
    heap_.Push(s, 0);
  }
}

NodeId Dijkstra::SettleNext() {
  NodeId v = heap_.Pop();
  labels_.Settle(v);
  ++stats_.settled;
  const Weight dv = labels_.Dist(v);
  for (std::uint32_t a = graph_.first[v]; a < graph_.first[v + 1]; ++a) {
    NodeId w = graph_.other[a];
    std::uint64_t nd = std::uint64_t{dv} + graph_.weight[a];
    if (nd >= kInf) continue;
    ++stats_.relaxed;
    if (!labels_.Reached(w)) {
      labels_.Set(w, static_cast<Weight>(nd), v);
      heap_.Push(w, static_cast<Weight>(nd));
    } else if (nd < labels_.Dist(w) && !labels_.Settled(w)) {
      labels_.Set(w, static_cast<Weight>(nd), v);
      heap_.Decrease(w, static_cast<Weight>(nd));
    }
  }
  return v;
}

Weight Dijkstra::Run(NodeId source, NodeId target) {
  target_ = target;
  const NodeId sources[1] = {source};
  Start(sources);
  while (!heap_.Empty()) {
    if (heap_.MinId() == target) {
      heap_.Pop();
      labels_.Settle(target);
      ++stats_.settled;
      return labels_.Dist(target);
    }
    SettleNext();
  }
  return kInf;
}

BidirectionalDijkstra::BidirectionalDijkstra(Adjacency forward, Adjacency backward)
    : graph_{forward, backward}, labels_{Labels(forward.node_count), Labels(backward.node_count)} {
  heap_[0].Resize(forward.node_count);
  heap_[1].Resize(backward.node_count);
}

Weight BidirectionalDijkstra::Run(NodeId source, NodeId target) {
  stats_ = {};
  for (int d = 0; d < 2; ++d) {
    labels_[d].NewRound();
    heap_[d].Clear();
  }
  labels_[0].Set(source, 0, kNoNode);
  heap_[0].Push(source, 0);
  labels_[1].Set(target, 0, kNoNode);
  heap_[1].Push(target, 0);
  best_ = kInf;
  meet_ = kNoNode;
  if (source == target) {
    best_ = 0;
    meet_ = source;
    return 0;
  }
  while (true) {
    std::uint64_t top0 = heap_[0].Empty() ? kInf : heap_[0].MinKey();
    std::uint64_t top1 = heap_[1].Empty() ? kInf : heap_[1].MinKey();
    if (top0 + top1 >= best_) break;
    int d = top0 <= top1 ? 0 : 1;
    Labels& mine = labels_[d];
    const Labels& other = labels_[1 - d];
    const Adjacency& g = graph_[d];
    NodeId u = heap_[d].Pop();
    mine.Settle(u);
    ++stats_.settled;
    const Weight du = mine.Dist(u);
    for (std::uint32_t a = g.first[u]; a < g.first[u + 1]; ++a) {
      NodeId x = g.other[a];
      std::uint64_t nd = std::uint64_t{du} + g.weight[a];
      if (nd >= kInf) continue;
      ++stats_.relaxed;
      if (!mine.Reached(x)) {
        mine.Set(x, static_cast<Weight>(nd), u);
        heap_[d].Push(x, static_cast<Weight>(nd));
      } else if (nd < mine.Dist(x) && !mine.Settled(x)) {
        mine.Set(x, static_cast<Weight>(nd), u);
        heap_[d].Decrease(x, static_cast<Weight>(nd));
      } else {
        continue;
      }
      if (other.Reached(x)) {
        std::uint64_t total = nd + other.Dist(x);
        if (total < best_) {
          best_ = static_cast<Weight>(total);
          meet_ = x;
        }
      }
    }
  }
  return best_;
}

std::vector<NodeId> BidirectionalDijkstra::Path() const {
  if (meet_ == kNoNode) return {};
  std::vector<NodeId> path = labels_[0].PathTo(meet_);
  for (NodeId x = labels_[1].Parent(meet_); x != kNoNode; x = labels_[1].Parent(x)) {
    path.push_back(x);
  }
  return path;
}

double GeoScale(const Graph& graph) {
  double scale = std::numeric_limits<double>::infinity();
  Adjacency g = graph.forward();
  for (NodeId v = 0; v < graph.node_count(); ++v) {
    for (std::uint32_t a = g.first[v]; a < g.first[v + 1]; ++a) {
      double meters = HaversineMeters(graph.coord(v), graph.coord(g.other[a]));
      if (meters <= 0) continue;
      scale = std::min(scale, g.weight[a] / meters);
    }
  }
  if (!std::isfinite(scale)) return 0;
  return scale * (1 - 1e-6);
}

std::int64_t GeoPotential::Bound(LatLon a, LatLon b) const {
  return static_cast<std::int64_t>(scale_ * HaversineMeters(a, b));
}

Result<Landmarks> Landmarks::Build(const Graph& graph, std::uint32_t count, std::uint64_t seed,
                                   ThreadPool& pool) {
  const std::uint32_t n = graph.node_count();
  if (count == 0 || count > kMaxLandmarks) {
    return Status::Error("landmark count must be between 1 and " + std::to_string(kMaxLandmarks));
  }
  count = std::min(count, n);
  const std::size_t width = std::size_t{2} * count;
  ReverseGraph reverse(graph);
  std::vector<Weight> table(std::size_t{n} * width, kInf);
  std::vector<NodeId> chosen;
  std::vector<char> is_landmark(n, 0);
  std::vector<char> has_landmark(n, 0);
  std::vector<std::uint64_t> size(n, 0);
  std::vector<NodeId> order;
  order.reserve(n);
  std::vector<std::uint32_t> child_first(std::size_t{n} + 1, 0);
  std::vector<NodeId> children(n, kNoNode);
  std::vector<Dijkstra> searches;
  searches.emplace_back(graph.forward());
  searches.emplace_back(reverse.view());
  Dijkstra& tree = searches[0];
  Rng rng(seed);
  const Status disconnected =
      Status::Error("landmarks need a strongly connected graph, import it without --keep-all");

  for (std::uint32_t i = 0; i < count; ++i) {
    NodeId root = rng.Below(n);
    order.clear();
    const NodeId roots[1] = {root};
    tree.RunFull(roots, [&](NodeId v, Weight) {
      order.push_back(v);
      return true;
    });
    if (order.size() != n) return disconnected;
    const Weight* root_row = table.data() + std::size_t{root} * width;
    for (NodeId v : order) {
      const Weight* row = table.data() + std::size_t{v} * width;
      std::int64_t lower = 0;
      for (std::uint32_t l = 0; l < i; ++l) {
        lower = std::max(lower, static_cast<std::int64_t>(root_row[2 * l + 1]) - row[2 * l + 1]);
        lower = std::max(lower, static_cast<std::int64_t>(row[2 * l]) - root_row[2 * l]);
      }
      size[v] = static_cast<std::uint64_t>(std::max<std::int64_t>(0, tree.Dist(v) - lower));
      has_landmark[v] = is_landmark[v];
    }
    std::fill(child_first.begin(), child_first.end(), 0);
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
      NodeId v = *it;
      NodeId parent = tree.Parent(v);
      if (parent == kNoNode) continue;
      ++child_first[parent + 1];
      if (has_landmark[v]) {
        size[v] = 0;
        has_landmark[parent] = 1;
      } else {
        size[parent] += size[v];
      }
    }
    if (has_landmark[root]) size[root] = 0;
    for (std::uint32_t v = 0; v < n; ++v) child_first[v + 1] += child_first[v];
    {
      std::vector<std::uint32_t> fill(child_first.begin(), child_first.end() - 1);
      for (NodeId v : order) {
        NodeId parent = tree.Parent(v);
        if (parent != kNoNode) children[fill[parent]++] = v;
      }
    }
    NodeId pick = kNoNode;
    std::uint64_t best_size = 0;
    for (NodeId v : order) {
      if (!has_landmark[v] && size[v] > best_size) {
        best_size = size[v];
        pick = v;
      }
    }
    if (pick == kNoNode) {
      do {
        pick = rng.Below(n);
      } while (is_landmark[pick]);
    } else {
      while (true) {
        NodeId best_child = kNoNode;
        std::uint64_t best_child_size = 0;
        for (std::uint32_t c = child_first[pick]; c < child_first[pick + 1]; ++c) {
          NodeId child = children[c];
          if (has_landmark[child]) continue;
          if (best_child == kNoNode || size[child] > best_child_size) {
            best_child = child;
            best_child_size = size[child];
          }
        }
        if (best_child == kNoNode) break;
        pick = best_child;
      }
    }
    chosen.push_back(pick);
    is_landmark[pick] = 1;
    std::uint64_t unreached[2] = {0, 0};
    pool.Run(2, 1, [&](std::size_t dir, std::size_t, int) {
      const NodeId sources[1] = {pick};
      std::uint64_t reached = 0;
      searches[dir].RunFull(sources, [&](NodeId v, Weight d) {
        table[std::size_t{v} * width + 2 * i + dir] = d;
        ++reached;
        return true;
      });
      unreached[dir] = n - reached;
    });
    if (unreached[0] != 0 || unreached[1] != 0) return disconnected;
  }
  Landmarks landmarks;
  landmarks.count_ = count;
  landmarks.node_count_ = n;
  landmarks.nodes_ = Array<NodeId>(std::move(chosen));
  landmarks.table_ = Array<Weight>(std::move(table));
  return landmarks;
}

Status Landmarks::Save(const std::string& path, const Graph& graph) const {
  std::vector<std::uint32_t> meta = {node_count_, count_, graph.identity(), kLandmarkFormat};
  ContainerWriter writer(kLandmarkMagic);
  writer.Add(Tag("META"), meta);
  writer.Add(Tag("NODE"), nodes_.span());
  writer.Add(Tag("DIST"), table_.span());
  return writer.Write(path);
}

Result<Landmarks> Landmarks::Load(const std::string& path, const Graph& graph) {
  Result<ContainerReader> reader = ContainerReader::Open(path, kLandmarkMagic, false);
  if (!reader.ok()) return reader.status();
  Result<Array<std::uint32_t>> meta = reader->Get<std::uint32_t>(Tag("META"), 4);
  if (!meta.ok()) return meta.status();
  const Array<std::uint32_t>& m = *meta;
  if (m[3] != kLandmarkFormat) return Status::Error(path + " has an unknown landmark format");
  if (m[0] != graph.node_count() || m[2] != graph.identity()) {
    return Status::Error(path + " was built for a different graph");
  }
  if (m[1] == 0 || m[1] > kMaxLandmarks || m[1] > m[0]) {
    return Status::Error(path + " has an invalid landmark count");
  }
  Landmarks landmarks;
  landmarks.node_count_ = m[0];
  landmarks.count_ = m[1];
  Result<Array<NodeId>> nodes = reader->Get<NodeId>(Tag("NODE"), landmarks.count_);
  if (!nodes.ok()) return nodes.status();
  for (NodeId v : nodes->span()) {
    if (v >= landmarks.node_count_) return Status::Error(path + " has an invalid landmark");
  }
  Result<Array<Weight>> table =
      reader->Get<Weight>(Tag("DIST"), std::uint64_t{landmarks.node_count_} * 2 * landmarks.count_);
  if (!table.ok()) return table.status();
  landmarks.nodes_ = std::move(*nodes);
  landmarks.table_ = std::move(*table);
  return landmarks;
}

void LandmarkPotential::Prepare(NodeId source, NodeId target) {
  const Landmarks& lm = *landmarks_;
  source_row_ = lm.Row(source);
  target_row_ = lm.Row(target);
  std::array<std::pair<std::int64_t, std::uint32_t>, 64> scored;
  const std::uint32_t k = lm.count();
  for (std::uint32_t l = 0; l < k; ++l) {
    std::int64_t a = static_cast<std::int64_t>(source_row_[2 * l + 1]) - target_row_[2 * l + 1];
    std::int64_t b = static_cast<std::int64_t>(target_row_[2 * l]) - source_row_[2 * l];
    scored[l] = {-std::max(a, b), l};
  }
  std::sort(scored.begin(), scored.begin() + k);
  active_count_ = std::min(active_limit_, k);
  for (std::uint32_t i = 0; i < active_count_; ++i) active_[i] = scored[i].second;
}

std::int64_t LandmarkPotential::ToTarget(NodeId v) const {
  const Weight* row = landmarks_->Row(v);
  std::int64_t best = 0;
  for (std::uint32_t i = 0; i < active_count_; ++i) {
    std::uint32_t l = active_[i];
    best = std::max(best, static_cast<std::int64_t>(row[2 * l + 1]) - target_row_[2 * l + 1]);
    best = std::max(best, static_cast<std::int64_t>(target_row_[2 * l]) - row[2 * l]);
  }
  return best;
}

std::int64_t LandmarkPotential::FromSource(NodeId v) const {
  const Weight* row = landmarks_->Row(v);
  std::int64_t best = 0;
  for (std::uint32_t i = 0; i < active_count_; ++i) {
    std::uint32_t l = active_[i];
    best = std::max(best, static_cast<std::int64_t>(source_row_[2 * l + 1]) - row[2 * l + 1]);
    best = std::max(best, static_cast<std::int64_t>(row[2 * l]) - source_row_[2 * l]);
  }
  return best;
}

template <typename Potential>
AStar<Potential>::AStar(Adjacency graph, Potential potential)
    : graph_(graph),
      potential_(std::move(potential)),
      labels_(graph.node_count),
      pi_(graph.node_count, 0) {
  heap_.Resize(graph.node_count);
}

template <typename Potential>
Weight AStar<Potential>::Run(NodeId source, NodeId target) {
  target_ = target;
  stats_ = {};
  labels_.NewRound();
  heap_.Clear();
  potential_.Prepare(source, target);
  labels_.Set(source, 0, kNoNode);
  pi_[source] = potential_.ToTarget(source);
  heap_.Push(source, static_cast<std::uint64_t>(pi_[source]));
  while (!heap_.Empty()) {
    NodeId u = heap_.Pop();
    labels_.Settle(u);
    ++stats_.settled;
    const Weight du = labels_.Dist(u);
    if (u == target) return du;
    for (std::uint32_t a = graph_.first[u]; a < graph_.first[u + 1]; ++a) {
      NodeId x = graph_.other[a];
      std::uint64_t nd = std::uint64_t{du} + graph_.weight[a];
      if (nd >= kInf) continue;
      ++stats_.relaxed;
      if (!labels_.Reached(x)) {
        pi_[x] = potential_.ToTarget(x);
        labels_.Set(x, static_cast<Weight>(nd), u);
        heap_.Push(x, nd + static_cast<std::uint64_t>(pi_[x]));
      } else if (nd < labels_.Dist(x) && !labels_.Settled(x)) {
        labels_.Set(x, static_cast<Weight>(nd), u);
        heap_.Decrease(x, nd + static_cast<std::uint64_t>(pi_[x]));
      }
    }
  }
  return kInf;
}

template <typename Potential>
BidirectionalAStar<Potential>::BidirectionalAStar(Adjacency forward, Adjacency backward,
                                                  Potential potential)
    : graph_{forward, backward},
      potential_(std::move(potential)),
      labels_{Labels(forward.node_count), Labels(backward.node_count)},
      offset_(forward.node_count, 0),
      offset_round_(forward.node_count, 0) {
  heap_[0].Resize(forward.node_count);
  heap_[1].Resize(backward.node_count);
}

template <typename Potential>
std::int64_t BidirectionalAStar<Potential>::Offset(NodeId v) {
  if (offset_round_[v] != round_) {
    offset_[v] = potential_.ToTarget(v) - potential_.FromSource(v);
    offset_round_[v] = round_;
  }
  return offset_[v];
}

template <typename Potential>
Weight BidirectionalAStar<Potential>::Run(NodeId source, NodeId target) {
  stats_ = {};
  if (++round_ == 0) {
    std::fill(offset_round_.begin(), offset_round_.end(), 0);
    round_ = 1;
  }
  for (int d = 0; d < 2; ++d) {
    labels_[d].NewRound();
    heap_[d].Clear();
  }
  potential_.Prepare(source, target);
  labels_[0].Set(source, 0, kNoNode);
  heap_[0].Push(source, Offset(source));
  labels_[1].Set(target, 0, kNoNode);
  heap_[1].Push(target, -Offset(target));
  best_ = kInf;
  meet_ = kNoNode;
  if (source == target) {
    best_ = 0;
    meet_ = source;
    return 0;
  }
  while (true) {
    std::int64_t top0 = heap_[0].Empty() ? kNoKey : heap_[0].MinKey();
    std::int64_t top1 = heap_[1].Empty() ? kNoKey : heap_[1].MinKey();
    if (top0 + top1 >= 2 * static_cast<std::int64_t>(best_)) break;
    int d = top0 <= top1 ? 0 : 1;
    const std::int64_t sign = d == 0 ? 1 : -1;
    Labels& mine = labels_[d];
    const Labels& other = labels_[1 - d];
    const Adjacency& g = graph_[d];
    NodeId u = heap_[d].Pop();
    mine.Settle(u);
    ++stats_.settled;
    const Weight du = mine.Dist(u);
    for (std::uint32_t a = g.first[u]; a < g.first[u + 1]; ++a) {
      NodeId x = g.other[a];
      std::uint64_t nd = std::uint64_t{du} + g.weight[a];
      if (nd >= kInf) continue;
      ++stats_.relaxed;
      if (!mine.Reached(x)) {
        mine.Set(x, static_cast<Weight>(nd), u);
        heap_[d].Push(x, 2 * static_cast<std::int64_t>(nd) + sign * Offset(x));
      } else if (nd < mine.Dist(x) && !mine.Settled(x)) {
        mine.Set(x, static_cast<Weight>(nd), u);
        heap_[d].Decrease(x, 2 * static_cast<std::int64_t>(nd) + sign * Offset(x));
      } else {
        continue;
      }
      if (other.Reached(x)) {
        std::uint64_t total = nd + other.Dist(x);
        if (total < best_) {
          best_ = static_cast<Weight>(total);
          meet_ = x;
        }
      }
    }
  }
  return best_;
}

template <typename Potential>
std::vector<NodeId> BidirectionalAStar<Potential>::Path() const {
  if (meet_ == kNoNode) return {};
  std::vector<NodeId> path = labels_[0].PathTo(meet_);
  for (NodeId x = labels_[1].Parent(meet_); x != kNoNode; x = labels_[1].Parent(x)) {
    path.push_back(x);
  }
  return path;
}

template class AStar<GeoPotential>;
template class AStar<LandmarkPotential>;
template class BidirectionalAStar<GeoPotential>;
template class BidirectionalAStar<LandmarkPotential>;

}
