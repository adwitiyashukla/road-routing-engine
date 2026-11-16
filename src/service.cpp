#include "service.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace rr {

namespace {

struct Json {
  std::string out;

  void Open(char c) { out.push_back(c); }
  void Close(char c) { out.push_back(c); }
  void Comma() {
    if (!out.empty() && out.back() != '{' && out.back() != '[') out.push_back(',');
  }
  void Key(std::string_view name) {
    Comma();
    out.push_back('"');
    out.append(name);
    out.append("\":");
  }
  void Number(std::int64_t value) {
    char digits[24];
    auto result = std::to_chars(digits, digits + sizeof(digits), value);
    out.append(digits, result.ptr);
  }
  void Fixed(double value, int decimals) {
    char text[40];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    out.append(text);
  }
  void Text(std::string_view value) {
    out.push_back('"');
    out.append(JsonEscape(value));
    out.push_back('"');
  }
  void Bool(bool value) { out.append(value ? "true" : "false"); }
  void Null() { out.append("null"); }
  void Field(std::string_view name, std::int64_t value) {
    Key(name);
    Number(value);
  }
  void Field(std::string_view name, std::string_view value) {
    Key(name);
    Text(value);
  }
  void Flag(std::string_view name, bool value) {
    Key(name);
    Bool(value);
  }
  void Point(std::string_view name, NodeId node, LatLon p) {
    Key(name);
    Open('{');
    Field("node", static_cast<std::int64_t>(node));
    Key("lat");
    Fixed(p.lat / 1e6, 6);
    Key("lon");
    Fixed(p.lon / 1e6, 6);
    Close('}');
  }
};

HttpResponse Reply(int status, std::string body) {
  HttpResponse response;
  response.status = status;
  response.body = std::move(body);
  return response;
}

HttpResponse Fail(int status, const std::string& message) {
  return Reply(status, "{\"error\":\"" + JsonEscape(message) + "\"}");
}

bool ParseDouble(std::string_view text, double* out) {
  if (text.empty() || text.size() > 32) return false;
  std::string copy(text);
  char* end = nullptr;
  double value = std::strtod(copy.c_str(), &end);
  if (end != copy.c_str() + copy.size() || !std::isfinite(value)) return false;
  *out = value;
  return true;
}

bool ParseNodeId(std::string_view text, std::uint32_t bound, NodeId* out) {
  if (text.empty() || text.size() > 10) return false;
  std::uint64_t value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') return false;
    value = value * 10 + static_cast<std::uint64_t>(c - '0');
  }
  if (value >= bound) return false;
  *out = static_cast<NodeId>(value);
  return true;
}

bool ParsePoint(std::string_view text, LatLon* out) {
  std::size_t comma = text.find(',');
  if (comma == std::string_view::npos) return false;
  double lat = 0;
  double lon = 0;
  if (!ParseDouble(text.substr(0, comma), &lat)) return false;
  if (!ParseDouble(text.substr(comma + 1), &lon)) return false;
  if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;
  *out = FromDegrees(lat, lon);
  return true;
}

std::vector<std::string_view> Split(std::string_view text, char separator) {
  std::vector<std::string_view> parts;
  while (!text.empty()) {
    std::size_t at = text.find(separator);
    std::string_view part = text.substr(0, at);
    if (!part.empty()) parts.push_back(part);
    if (at == std::string_view::npos) break;
    text.remove_prefix(at + 1);
  }
  return parts;
}

}

bool ParseAlgorithm(std::string_view name, Algorithm* algorithm) {
  const std::pair<std::string_view, Algorithm> names[] = {
      {"auto", Algorithm::kAuto},
      {"dijkstra", Algorithm::kDijkstra},
      {"bidirectional", Algorithm::kBidirectional},
      {"astar", Algorithm::kAStar},
      {"bidirectional-astar", Algorithm::kBidirectionalAStar},
      {"alt", Algorithm::kAlt},
      {"bidirectional-alt", Algorithm::kBidirectionalAlt},
      {"ch", Algorithm::kHierarchy},
  };
  for (const auto& [text, value] : names) {
    if (text == name) {
      *algorithm = value;
      return true;
    }
  }
  return false;
}

const char* AlgorithmName(Algorithm algorithm) {
  switch (algorithm) {
    case Algorithm::kDijkstra:
      return "dijkstra";
    case Algorithm::kBidirectional:
      return "bidirectional";
    case Algorithm::kAStar:
      return "astar";
    case Algorithm::kBidirectionalAStar:
      return "bidirectional-astar";
    case Algorithm::kAlt:
      return "alt";
    case Algorithm::kBidirectionalAlt:
      return "bidirectional-alt";
    case Algorithm::kHierarchy:
      return "ch";
    case Algorithm::kAuto:
      return "auto";
  }
  return "auto";
}

Result<std::unique_ptr<Engine>> Engine::Open(const EngineFiles& files) {
  Result<Graph> graph = Graph::Load(files.graph, files.verify);
  if (!graph.ok()) return graph.status();
  std::unique_ptr<Engine> engine(new Engine());
  engine->graph_ = std::move(*graph);
  engine->reverse_ = std::make_unique<ReverseGraph>(engine->graph_);
  if (!files.hierarchy.empty()) {
    Result<ChGraph> ch = ChGraph::Load(files.hierarchy, engine->graph_, files.verify);
    if (!ch.ok()) return ch.status();
    engine->hierarchy_ = std::move(*ch);
  }
  if (!files.landmarks.empty()) {
    Result<Landmarks> landmarks = Landmarks::Load(files.landmarks, engine->graph_);
    if (!landmarks.ok()) return landmarks.status();
    engine->landmarks_ = std::move(*landmarks);
  }
  engine->geo_scale_ = GeoScale(engine->graph_);
  if (files.snap) engine->kd_ = KdTree(engine->graph_.coords());
  return engine;
}

NodeId Engine::Nearest(LatLon point) const { return kd_.Nearest(point); }

double Engine::PathMeters(std::span<const NodeId> path) const {
  double meters = 0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    meters += HaversineMeters(graph_.coord(path[i]), graph_.coord(path[i + 1]));
  }
  return meters;
}

Algorithm Engine::Resolve(Algorithm algorithm) const {
  if (algorithm != Algorithm::kAuto) return algorithm;
  if (hierarchy_) return Algorithm::kHierarchy;
  if (landmarks_) return Algorithm::kBidirectionalAlt;
  return Algorithm::kBidirectional;
}

Result<RouteResult> Querier::Route(NodeId source, NodeId target, Algorithm algorithm,
                                   bool want_path) {
  const Engine& engine = *engine_;
  RouteResult result;
  result.algorithm = engine.Resolve(algorithm);
  const Landmarks* landmarks = engine.landmarks();
  if ((result.algorithm == Algorithm::kAlt || result.algorithm == Algorithm::kBidirectionalAlt) &&
      landmarks == nullptr) {
    return Status::Error("no landmark file is loaded");
  }
  if (result.algorithm == Algorithm::kHierarchy && engine.hierarchy() == nullptr) {
    return Status::Error("no hierarchy file is loaded");
  }
  Timer timer;
  switch (result.algorithm) {
    case Algorithm::kDijkstra: {
      if (!dijkstra_) dijkstra_ = std::make_unique<Dijkstra>(engine.forward());
      result.weight = dijkstra_->Run(source, target);
      result.stats = dijkstra_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = dijkstra_->Path();
      break;
    }
    case Algorithm::kBidirectional: {
      if (!bidirectional_) {
        bidirectional_ =
            std::make_unique<BidirectionalDijkstra>(engine.forward(), engine.backward());
      }
      result.weight = bidirectional_->Run(source, target);
      result.stats = bidirectional_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = bidirectional_->Path();
      break;
    }
    case Algorithm::kAStar: {
      if (!astar_) {
        astar_ = std::make_unique<AStar<GeoPotential>>(
            engine.forward(), GeoPotential(engine.graph(), engine.geo_scale()));
      }
      result.weight = astar_->Run(source, target);
      result.stats = astar_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = astar_->Path();
      break;
    }
    case Algorithm::kBidirectionalAStar: {
      if (!bidirectional_astar_) {
        bidirectional_astar_ = std::make_unique<BidirectionalAStar<GeoPotential>>(
            engine.forward(), engine.backward(), GeoPotential(engine.graph(), engine.geo_scale()));
      }
      result.weight = bidirectional_astar_->Run(source, target);
      result.stats = bidirectional_astar_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = bidirectional_astar_->Path();
      break;
    }
    case Algorithm::kAlt: {
      if (!alt_) {
        alt_ = std::make_unique<AStar<LandmarkPotential>>(engine.forward(),
                                                          LandmarkPotential(*landmarks, 4));
      }
      result.weight = alt_->Run(source, target);
      result.stats = alt_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = alt_->Path();
      break;
    }
    case Algorithm::kBidirectionalAlt: {
      if (!bidirectional_alt_) {
        bidirectional_alt_ = std::make_unique<BidirectionalAStar<LandmarkPotential>>(
            engine.forward(), engine.backward(), LandmarkPotential(*landmarks, 4));
      }
      result.weight = bidirectional_alt_->Run(source, target);
      result.stats = bidirectional_alt_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = bidirectional_alt_->Path();
      break;
    }
    case Algorithm::kHierarchy:
    case Algorithm::kAuto: {
      if (!hierarchy_) hierarchy_ = std::make_unique<ChQuery>(*engine.hierarchy());
      result.weight = hierarchy_->Run(source, target);
      result.stats = hierarchy_->stats();
      result.milliseconds = timer.Seconds() * 1000;
      if (want_path) result.path = hierarchy_->Path();
      break;
    }
  }
  return result;
}

Result<std::vector<Weight>> Querier::Table(std::span<const NodeId> sources,
                                           std::span<const NodeId> targets, ThreadPool& pool) {
  const Engine& engine = *engine_;
  if (engine.hierarchy() != nullptr) {
    return ChTable(*engine.hierarchy()).Compute(sources, targets, pool);
  }
  if (!dijkstra_) dijkstra_ = std::make_unique<Dijkstra>(engine.forward());
  std::vector<Weight> table(sources.size() * targets.size(), kInf);
  for (std::size_t i = 0; i < sources.size(); ++i) {
    const NodeId one[1] = {sources[i]};
    dijkstra_->RunFull(one, [](NodeId, Weight) { return true; });
    for (std::size_t j = 0; j < targets.size(); ++j) {
      table[i * targets.size() + j] = dijkstra_->Dist(targets[j]);
    }
  }
  return table;
}

Service::Lease::Lease(Service& service) : service_(&service) {
  std::lock_guard<std::mutex> lock(service.mutex_);
  if (!service.idle_.empty()) {
    querier_ = std::move(service.idle_.back());
    service.idle_.pop_back();
  }
  if (!querier_) querier_ = std::make_unique<Querier>(*service.engine_);
}

Service::Lease::~Lease() {
  std::lock_guard<std::mutex> lock(service_->mutex_);
  service_->idle_.push_back(std::move(querier_));
}

HttpResponse Service::Handle(const HttpRequest& request) {
  ++served_;
  if (request.path == "/health") return Health();
  if (request.path == "/route") return Route(request);
  if (request.path == "/nearest") return Nearest(request);
  if (request.path == "/table") return Table(request);
  return Fail(404, "unknown path " + request.path);
}

HttpResponse Service::Health() {
  const Engine& engine = *engine_;
  Json json;
  json.Open('{');
  json.Field("status", std::string_view("ok"));
  json.Field("nodes", static_cast<std::int64_t>(engine.graph().node_count()));
  json.Field("arcs", static_cast<std::int64_t>(engine.graph().arc_count()));
  json.Flag("hierarchy", engine.hierarchy() != nullptr);
  json.Key("shortcuts");
  if (engine.hierarchy() != nullptr) {
    json.Number(static_cast<std::int64_t>(engine.hierarchy()->shortcut_count()));
  } else {
    json.Null();
  }
  json.Key("landmarks");
  if (engine.landmarks() != nullptr) {
    json.Number(static_cast<std::int64_t>(engine.landmarks()->count()));
  } else {
    json.Null();
  }
  json.Flag("snapping", engine.can_snap());
  json.Field("algorithm", std::string_view(AlgorithmName(engine.Resolve(Algorithm::kAuto))));
  json.Field("requests", static_cast<std::int64_t>(served_.load()));
  json.Close('}');
  json.out.push_back('\n');
  return Reply(200, std::move(json.out));
}

Result<std::vector<NodeId>> Service::ResolvePoints(const HttpRequest& request,
                                                   std::string_view points, std::string_view nodes,
                                                   std::vector<LatLon>* resolved) {
  const Engine& engine = *engine_;
  std::vector<NodeId> out;
  if (const std::string* value = request.Param(nodes)) {
    for (std::string_view part : Split(*value, ';')) {
      NodeId node = kNoNode;
      if (!ParseNodeId(part, engine.graph().node_count(), &node)) {
        return Status::Error("bad node id in " + std::string(nodes));
      }
      out.push_back(node);
    }
  } else if (const std::string* value2 = request.Param(points)) {
    if (!engine.can_snap()) return Status::Error("this server was started without snapping");
    for (std::string_view part : Split(*value2, ';')) {
      LatLon point{};
      if (!ParsePoint(part, &point)) {
        return Status::Error("bad coordinate in " + std::string(points));
      }
      out.push_back(engine.Nearest(point));
    }
  } else {
    return Status::Error("missing " + std::string(points) + " or " + std::string(nodes));
  }
  if (out.empty()) return Status::Error("no points in " + std::string(points));
  if (resolved != nullptr) {
    for (NodeId node : out) resolved->push_back(engine.graph().coord(node));
  }
  return out;
}

HttpResponse Service::Route(const HttpRequest& request) {
  const Engine& engine = *engine_;
  Result<std::vector<NodeId>> from = ResolvePoints(request, "from", "from_node", nullptr);
  if (!from.ok()) return Fail(400, from.status().message());
  Result<std::vector<NodeId>> to = ResolvePoints(request, "to", "to_node", nullptr);
  if (!to.ok()) return Fail(400, to.status().message());
  if (from->size() != 1 || to->size() != 1) return Fail(400, "route takes one point per side");
  Algorithm algorithm = Algorithm::kAuto;
  if (const std::string* name = request.Param("algorithm")) {
    if (!ParseAlgorithm(*name, &algorithm)) return Fail(400, "unknown algorithm " + *name);
  }
  bool want_path = true;
  bool want_polyline = false;
  if (const std::string* value = request.Param("path")) {
    if (*value == "none") {
      want_path = false;
    } else if (*value == "polyline") {
      want_polyline = true;
    } else if (*value != "nodes") {
      return Fail(400, "path must be nodes, polyline or none");
    }
  }
  Lease lease(*this);
  Result<RouteResult> route = (*lease).Route((*from)[0], (*to)[0], algorithm, want_path);
  if (!route.ok()) return Fail(400, route.status().message());
  Json json;
  json.Open('{');
  json.Field("algorithm", std::string_view(AlgorithmName(route->algorithm)));
  json.Point("from", (*from)[0], engine.graph().coord((*from)[0]));
  json.Point("to", (*to)[0], engine.graph().coord((*to)[0]));
  json.Key("weight");
  if (route->weight == kInf) {
    json.Null();
  } else {
    json.Number(static_cast<std::int64_t>(route->weight));
  }
  json.Flag("reachable", route->weight != kInf);
  json.Field("settled", static_cast<std::int64_t>(route->stats.settled));
  json.Key("query_ms");
  json.Fixed(route->milliseconds, 3);
  if (want_path && route->weight != kInf) {
    json.Field("hops", static_cast<std::int64_t>(route->path.size()));
    json.Key("distance_meters");
    json.Fixed(engine.PathMeters(route->path), 1);
    if (want_polyline) {
      std::vector<LatLon> points;
      points.reserve(route->path.size());
      for (NodeId node : route->path) points.push_back(engine.graph().coord(node));
      json.Field("polyline", EncodePolyline(points));
    } else {
      json.Key("path");
      json.Open('[');
      for (NodeId node : route->path) {
        json.Comma();
        json.Number(static_cast<std::int64_t>(node));
      }
      json.Close(']');
    }
  }
  json.Close('}');
  json.out.push_back('\n');
  return Reply(200, std::move(json.out));
}

HttpResponse Service::Nearest(const HttpRequest& request) {
  const Engine& engine = *engine_;
  if (!engine.can_snap()) return Fail(400, "this server was started without snapping");
  const std::string* value = request.Param("point");
  LatLon point{};
  if (value == nullptr || !ParsePoint(*value, &point)) return Fail(400, "point must be lat,lon");
  NodeId node = engine.Nearest(point);
  Json json;
  json.Open('{');
  json.Point("nearest", node, engine.graph().coord(node));
  json.Key("meters");
  json.Fixed(HaversineMeters(point, engine.graph().coord(node)), 1);
  json.Close('}');
  json.out.push_back('\n');
  return Reply(200, std::move(json.out));
}

HttpResponse Service::Table(const HttpRequest& request) {
  Result<std::vector<NodeId>> sources = ResolvePoints(request, "sources", "source_nodes", nullptr);
  if (!sources.ok()) return Fail(400, sources.status().message());
  Result<std::vector<NodeId>> targets = ResolvePoints(request, "targets", "target_nodes", nullptr);
  if (!targets.ok()) return Fail(400, targets.status().message());
  if (sources->size() * targets->size() > max_table_cells_) {
    return Fail(400, "a table may have at most " + std::to_string(max_table_cells_) + " cells");
  }
  Timer timer;
  Lease lease(*this);
  Result<std::vector<Weight>> table = (*lease).Table(*sources, *targets, pool_);
  if (!table.ok()) return Fail(400, table.status().message());
  Json json;
  json.Open('{');
  json.Field("sources", static_cast<std::int64_t>(sources->size()));
  json.Field("targets", static_cast<std::int64_t>(targets->size()));
  json.Key("query_ms");
  json.Fixed(timer.Seconds() * 1000, 3);
  json.Key("weights");
  json.Open('[');
  for (std::size_t i = 0; i < sources->size(); ++i) {
    json.Comma();
    json.Open('[');
    for (std::size_t j = 0; j < targets->size(); ++j) {
      json.Comma();
      Weight w = (*table)[i * targets->size() + j];
      if (w == kInf) {
        json.Null();
      } else {
        json.Number(static_cast<std::int64_t>(w));
      }
    }
    json.Close(']');
  }
  json.Close(']');
  json.Close('}');
  json.out.push_back('\n');
  return Reply(200, std::move(json.out));
}

}
