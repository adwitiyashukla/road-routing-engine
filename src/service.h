#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ch.h"
#include "geo.h"
#include "graph.h"
#include "http.h"
#include "search.h"
#include "thread_pool.h"

namespace rr {

enum class Algorithm {
  kAuto,
  kDijkstra,
  kBidirectional,
  kAStar,
  kBidirectionalAStar,
  kAlt,
  kBidirectionalAlt,
  kHierarchy
};

bool ParseAlgorithm(std::string_view name, Algorithm* algorithm);
const char* AlgorithmName(Algorithm algorithm);

struct EngineFiles {
  std::string graph;
  std::string hierarchy;
  std::string landmarks;
  bool verify = false;
  bool snap = true;
};

struct RouteResult {
  Weight weight = kInf;
  Algorithm algorithm = Algorithm::kAuto;
  SearchStats stats;
  double milliseconds = 0;
  std::vector<NodeId> path;
};

class Engine {
 public:
  static Result<std::unique_ptr<Engine>> Open(const EngineFiles& files);

  const Graph& graph() const { return graph_; }
  Adjacency forward() const { return graph_.forward(); }
  Adjacency backward() const { return reverse_->view(); }
  const ChGraph* hierarchy() const { return hierarchy_ ? &*hierarchy_ : nullptr; }
  const Landmarks* landmarks() const { return landmarks_ ? &*landmarks_ : nullptr; }
  double geo_scale() const { return geo_scale_; }
  NodeId Nearest(LatLon point) const;
  bool can_snap() const { return kd_.size() > 0; }
  double PathMeters(std::span<const NodeId> path) const;
  Algorithm Resolve(Algorithm algorithm) const;

 private:
  Graph graph_;
  std::unique_ptr<ReverseGraph> reverse_;
  std::optional<ChGraph> hierarchy_;
  std::optional<Landmarks> landmarks_;
  KdTree kd_;
  double geo_scale_ = 0;
};

class Querier {
 public:
  explicit Querier(const Engine& engine) : engine_(&engine) {}

  Result<RouteResult> Route(NodeId source, NodeId target, Algorithm algorithm, bool want_path);
  Result<std::vector<Weight>> Table(std::span<const NodeId> sources,
                                    std::span<const NodeId> targets, ThreadPool& pool);

 private:
  const Engine* engine_;
  std::unique_ptr<Dijkstra> dijkstra_;
  std::unique_ptr<BidirectionalDijkstra> bidirectional_;
  std::unique_ptr<AStar<GeoPotential>> astar_;
  std::unique_ptr<BidirectionalAStar<GeoPotential>> bidirectional_astar_;
  std::unique_ptr<AStar<LandmarkPotential>> alt_;
  std::unique_ptr<BidirectionalAStar<LandmarkPotential>> bidirectional_alt_;
  std::unique_ptr<ChQuery> hierarchy_;
};

class Service {
 public:
  Service(const Engine& engine, int threads)
      : engine_(&engine), pool_(threads), max_table_cells_(200000) {}

  HttpResponse Handle(const HttpRequest& request);
  std::uint64_t served() const { return served_; }

 private:
  class Lease {
   public:
    Lease(Service& service);
    ~Lease();
    Querier& operator*() { return *querier_; }

   private:
    Service* service_;
    std::unique_ptr<Querier> querier_;
  };

  HttpResponse Health();
  HttpResponse Route(const HttpRequest& request);
  HttpResponse Nearest(const HttpRequest& request);
  HttpResponse Table(const HttpRequest& request);
  Result<std::vector<NodeId>> ResolvePoints(const HttpRequest& request, std::string_view points,
                                            std::string_view nodes, std::vector<LatLon>* resolved);

  const Engine* engine_;
  ThreadPool pool_;
  std::size_t max_table_cells_;
  std::mutex mutex_;
  std::vector<std::unique_ptr<Querier>> idle_;
  std::atomic<std::uint64_t> served_{0};
};

}
