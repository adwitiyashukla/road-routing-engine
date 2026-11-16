#include "graph.h"

#include <algorithm>
#include <numeric>
#include <utility>

#include "geo.h"

namespace rr {

namespace {

constexpr const char* kGraphMagic = "RRGRAPH";
constexpr std::uint32_t kGraphFormat = 1;

}

Result<Graph> Graph::FromArcs(std::uint32_t node_count, std::vector<Arc> arcs,
                              std::vector<LatLon> coords) {
  if (node_count == 0 || node_count > kMaxNodes) {
    return Status::Error("node count must be between 1 and " + std::to_string(kMaxNodes));
  }
  if (coords.size() != node_count) return Status::Error("coordinate count does not match nodes");
  if (arcs.size() >= std::size_t{0xffffffff}) return Status::Error("too many arcs");
  for (LatLon p : coords) {
    if (!ValidCoordinate(p)) return Status::Error("coordinate out of range");
  }
  std::vector<std::uint32_t> first(std::size_t{node_count} + 1, 0);
  for (const Arc& arc : arcs) {
    if (arc.tail >= node_count || arc.head >= node_count) {
      return Status::Error("arc endpoint out of range");
    }
    if (arc.weight > kMaxArcWeight) return Status::Error("arc weight too large");
    ++first[arc.tail + 1];
  }
  std::partial_sum(first.begin(), first.end(), first.begin());
  std::vector<std::uint64_t> packed(arcs.size());
  {
    std::vector<std::uint32_t> fill(first.begin(), first.end() - 1);
    for (const Arc& arc : arcs) {
      packed[fill[arc.tail]++] = static_cast<std::uint64_t>(arc.head) << 32 | arc.weight;
    }
  }
  std::vector<Arc>().swap(arcs);
  std::vector<NodeId> head;
  std::vector<Weight> weight;
  head.reserve(packed.size());
  weight.reserve(packed.size());
  std::vector<std::uint32_t> first_out(std::size_t{node_count} + 1, 0);
  for (NodeId v = 0; v < node_count; ++v) {
    auto begin = packed.begin() + first[v];
    auto end = packed.begin() + first[v + 1];
    std::sort(begin, end);
    NodeId last = kNoNode;
    for (auto it = begin; it != end; ++it) {
      NodeId h = static_cast<NodeId>(*it >> 32);
      if (h == v || h == last) continue;
      last = h;
      head.push_back(h);
      weight.push_back(static_cast<Weight>(*it & 0xffffffff));
    }
    first_out[v + 1] = static_cast<std::uint32_t>(head.size());
  }
  Graph graph;
  graph.node_count_ = node_count;
  graph.arc_count_ = static_cast<std::uint32_t>(head.size());
  graph.first_out_ = Array<std::uint32_t>(std::move(first_out));
  graph.head_ = Array<NodeId>(std::move(head));
  graph.weight_ = Array<Weight>(std::move(weight));
  graph.coords_ = Array<LatLon>(std::move(coords));
  graph.ComputeIdentity();
  return graph;
}

std::vector<std::uint32_t> Graph::Meta() const {
  return {node_count_, arc_count_, kGraphFormat, 0};
}

void Graph::ComputeIdentity() {
  std::vector<std::uint32_t> meta = Meta();
  std::uint32_t crcs[5] = {
      Crc32c(0, meta.data(), meta.size() * sizeof(std::uint32_t)),
      Crc32c(0, first_out_.data(), first_out_.size() * sizeof(std::uint32_t)),
      Crc32c(0, head_.data(), head_.size() * sizeof(NodeId)),
      Crc32c(0, weight_.data(), weight_.size() * sizeof(Weight)),
      Crc32c(0, coords_.data(), coords_.size() * sizeof(LatLon)),
  };
  identity_ = SectionIdentity(crcs);
}

Status Graph::Save(const std::string& path) const {
  std::vector<std::uint32_t> meta = Meta();
  ContainerWriter writer(kGraphMagic);
  writer.Add(Tag("META"), meta);
  writer.Add(Tag("FRST"), first_out_.span());
  writer.Add(Tag("HEAD"), head_.span());
  writer.Add(Tag("WGHT"), weight_.span());
  writer.Add(Tag("COOR"), coords_.span());
  return writer.Write(path);
}

Result<Graph> Graph::Load(const std::string& path, bool verify) {
  Result<ContainerReader> reader = ContainerReader::Open(path, kGraphMagic, verify);
  if (!reader.ok()) return reader.status();
  Result<Array<std::uint32_t>> meta = reader->Get<std::uint32_t>(Tag("META"), 4);
  if (!meta.ok()) return meta.status();
  if ((*meta)[2] != kGraphFormat) return Status::Error(path + " has an unknown graph format");
  Graph graph;
  graph.node_count_ = (*meta)[0];
  graph.arc_count_ = (*meta)[1];
  if (graph.node_count_ == 0 || graph.node_count_ > kMaxNodes) {
    return Status::Error(path + " has an invalid node count");
  }
  Result<Array<std::uint32_t>> first =
      reader->Get<std::uint32_t>(Tag("FRST"), graph.node_count_ + 1ULL);
  if (!first.ok()) return first.status();
  Result<Array<NodeId>> head = reader->Get<NodeId>(Tag("HEAD"), graph.arc_count_);
  if (!head.ok()) return head.status();
  Result<Array<Weight>> weight = reader->Get<Weight>(Tag("WGHT"), graph.arc_count_);
  if (!weight.ok()) return weight.status();
  Result<Array<LatLon>> coords = reader->Get<LatLon>(Tag("COOR"), graph.node_count_);
  if (!coords.ok()) return coords.status();
  graph.first_out_ = std::move(*first);
  graph.head_ = std::move(*head);
  graph.weight_ = std::move(*weight);
  graph.coords_ = std::move(*coords);
  graph.identity_ = reader->identity();
  Status valid = graph.Validate();
  if (!valid.ok()) return Status::Error(path + ": " + valid.message());
  return graph;
}

Status Graph::Validate() const {
  if (first_out_[0] != 0 || first_out_[node_count_] != arc_count_) {
    return Status::Error("arc offsets do not cover the arc array");
  }
  for (NodeId v = 0; v < node_count_; ++v) {
    std::uint32_t begin = first_out_[v];
    std::uint32_t end = first_out_[v + 1];
    if (begin > end || end > arc_count_) return Status::Error("arc offsets are not monotone");
    for (std::uint32_t a = begin; a < end; ++a) {
      if (head_[a] >= node_count_ || head_[a] == v) return Status::Error("invalid arc head");
      if (a > begin && head_[a] <= head_[a - 1]) return Status::Error("arcs are not sorted");
      if (weight_[a] > kMaxArcWeight) return Status::Error("arc weight too large");
    }
    if (!ValidCoordinate(coords_[v])) return Status::Error("coordinate out of range");
  }
  return Status();
}

Weight Graph::ArcWeight(NodeId from, NodeId to) const {
  const NodeId* begin = head_.data() + first_out_[from];
  const NodeId* end = head_.data() + first_out_[from + 1];
  const NodeId* it = std::lower_bound(begin, end, to);
  if (it == end || *it != to) return kInf;
  return weight_[static_cast<std::size_t>(it - head_.data())];
}

std::vector<Arc> Graph::Arcs() const {
  std::vector<Arc> arcs;
  arcs.reserve(arc_count_);
  for (NodeId v = 0; v < node_count_; ++v) {
    for (std::uint32_t a = first_out_[v]; a < first_out_[v + 1]; ++a) {
      arcs.push_back({v, head_[a], weight_[a]});
    }
  }
  return arcs;
}

ReverseGraph::ReverseGraph(const Graph& graph) : node_count_(graph.node_count()) {
  Adjacency fwd = graph.forward();
  first_in_.assign(std::size_t{node_count_} + 1, 0);
  for (std::uint32_t a = 0; a < graph.arc_count(); ++a) ++first_in_[fwd.other[a] + 1];
  std::partial_sum(first_in_.begin(), first_in_.end(), first_in_.begin());
  tail_.resize(graph.arc_count());
  weight_.resize(graph.arc_count());
  std::vector<std::uint32_t> fill(first_in_.begin(), first_in_.end() - 1);
  for (NodeId v = 0; v < node_count_; ++v) {
    for (std::uint32_t a = fwd.first[v]; a < fwd.first[v + 1]; ++a) {
      std::uint32_t slot = fill[fwd.other[a]]++;
      tail_[slot] = v;
      weight_[slot] = fwd.weight[a];
    }
  }
}

Result<Graph> Renumber(const Graph& graph, std::span<const NodeId> new_id) {
  const std::uint32_t n = graph.node_count();
  if (new_id.size() != n) return Status::Error("renumbering has the wrong size");
  std::uint32_t kept = 0;
  for (NodeId id : new_id) {
    if (id != kNoNode) ++kept;
  }
  if (kept == 0) return Status::Error("renumbering removes every node");
  std::vector<LatLon> coords(kept);
  std::vector<char> used(kept, 0);
  for (NodeId v = 0; v < n; ++v) {
    NodeId id = new_id[v];
    if (id == kNoNode) continue;
    if (id >= kept || used[id]) return Status::Error("renumbering is not a bijection");
    used[id] = 1;
    coords[id] = graph.coord(v);
  }
  std::vector<Arc> arcs;
  Adjacency fwd = graph.forward();
  for (NodeId v = 0; v < n; ++v) {
    if (new_id[v] == kNoNode) continue;
    for (std::uint32_t a = fwd.first[v]; a < fwd.first[v + 1]; ++a) {
      NodeId h = new_id[fwd.other[a]];
      if (h != kNoNode) arcs.push_back({new_id[v], h, fwd.weight[a]});
    }
  }
  return Graph::FromArcs(kept, std::move(arcs), std::move(coords));
}

Result<Graph> LargestStrongComponent(const Graph& graph, ComponentInfo* info) {
  const std::uint32_t n = graph.node_count();
  Adjacency fwd = graph.forward();
  std::vector<std::uint32_t> index(n, kNoNode);
  std::vector<std::uint32_t> low(n, 0);
  std::vector<std::uint32_t> component(n, kNoNode);
  std::vector<NodeId> stack;
  std::vector<bool> on_stack(n, false);
  struct Frame {
    NodeId node;
    std::uint32_t next;
  };
  std::vector<Frame> calls;
  std::uint32_t counter = 0;
  std::uint32_t components = 0;
  std::vector<std::uint32_t> sizes;
  for (NodeId root = 0; root < n; ++root) {
    if (index[root] != kNoNode) continue;
    index[root] = low[root] = counter++;
    stack.push_back(root);
    on_stack[root] = true;
    calls.push_back({root, fwd.first[root]});
    while (!calls.empty()) {
      NodeId v = calls.back().node;
      if (calls.back().next < fwd.first[v + 1]) {
        NodeId w = fwd.other[calls.back().next++];
        if (index[w] == kNoNode) {
          index[w] = low[w] = counter++;
          stack.push_back(w);
          on_stack[w] = true;
          calls.push_back({w, fwd.first[w]});
        } else if (on_stack[w]) {
          low[v] = std::min(low[v], index[w]);
        }
        continue;
      }
      calls.pop_back();
      if (!calls.empty()) {
        NodeId parent = calls.back().node;
        low[parent] = std::min(low[parent], low[v]);
      }
      if (low[v] == index[v]) {
        std::uint32_t size = 0;
        NodeId w = kNoNode;
        do {
          w = stack.back();
          stack.pop_back();
          on_stack[w] = false;
          component[w] = components;
          ++size;
        } while (w != v);
        sizes.push_back(size);
        ++components;
      }
    }
  }
  std::uint32_t best =
      static_cast<std::uint32_t>(std::max_element(sizes.begin(), sizes.end()) - sizes.begin());
  std::vector<NodeId> new_id(n, kNoNode);
  NodeId next = 0;
  for (NodeId v = 0; v < n; ++v) {
    if (component[v] == best) new_id[v] = next++;
  }
  if (info != nullptr) {
    info->components = components;
    info->kept_nodes = next;
  }
  return Renumber(graph, new_id);
}

std::vector<NodeId> HilbertNumbering(const Graph& graph) {
  const std::uint32_t n = graph.node_count();
  std::vector<std::pair<std::uint64_t, NodeId>> keys(n);
  for (NodeId v = 0; v < n; ++v) keys[v] = {HilbertKey(graph.coord(v)), v};
  std::sort(keys.begin(), keys.end());
  std::vector<NodeId> new_id(n);
  for (NodeId i = 0; i < n; ++i) new_id[keys[i].second] = i;
  return new_id;
}

}
