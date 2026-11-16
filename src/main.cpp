#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "base.h"
#include "ch.h"
#include "contract.h"
#include "dimacs.h"
#include "generator.h"
#include "geo.h"
#include "graph.h"
#include "http.h"
#include "io.h"
#include "search.h"
#include "service.h"
#include "thread_pool.h"

namespace rr {
namespace {

const char* kUsage =
    "usage: roadroute <command> [options]\n"
    "\n"
    "commands:\n"
    "  generate   write a synthetic road network in DIMACS format\n"
    "  import     turn a road network text file and its coordinates into one binary graph\n"
    "  prepare    build the contraction hierarchy and the landmark tables\n"
    "  route      answer one query and print it as JSON\n"
    "  bench      time every available algorithm on random queries\n"
    "  difftest   check every algorithm against plain Dijkstra\n"
    "  serve      run the HTTP routing server\n"
    "\n"
    "generate --out PREFIX [--nodes N] [--seed S] [--metric time|distance]\n"
    "         [--center LAT,LON]\n"
    "import --graph FILE.gr --coords FILE.co --out FILE.rrg [--keep-all]\n"
    "       [--no-reorder]\n"
    "prepare --graph FILE.rrg [--ch FILE] [--landmarks FILE] [--landmark-count K]\n"
    "        [--settle-limit N] [--threads N] [--seed S] [--quiet]\n"
    "route --graph FILE.rrg (--from LAT,LON --to LAT,LON | --from-node I --to-node J)\n"
    "      [--algorithm NAME] [--path nodes|polyline|none] [--ch FILE] [--no-ch]\n"
    "      [--landmarks FILE] [--no-landmarks] [--verify]\n"
    "bench --graph FILE.rrg [--queries N] [--seed S] [--algorithms LIST]\n"
    "      [--table N] [--threads N] [--ch FILE] [--landmarks FILE] [--verify]\n"
    "difftest --graph FILE.rrg [--queries N] [--seed S] [--ch FILE] [--landmarks FILE]\n"
    "serve --graph FILE.rrg [--host ADDRESS] [--port PORT] [--threads N]\n"
    "      [--ch FILE] [--landmarks FILE] [--no-snap] [--verify]\n";

class Args {
 public:
  Args(int argc, char** argv) {
    for (int i = 0; i < argc; ++i) {
      std::string_view item(argv[i]);
      if (item.size() > 2 && item.substr(0, 2) == "--") {
        std::string_view name = item.substr(2);
        std::size_t equals = name.find('=');
        if (equals != std::string_view::npos) {
          options_.push_back(
              {std::string(name.substr(0, equals)), std::string(name.substr(equals + 1)), true});
        } else {
          options_.push_back({std::string(name), std::string(), false});
        }
        continue;
      }
      if (!options_.empty() && !options_.back().has_value) {
        options_.back().value = std::string(item);
        options_.back().has_value = true;
        continue;
      }
      positional_.push_back(std::string(item));
    }
  }

  bool Flag(std::string_view name) {
    for (Option& option : options_) {
      if (option.name == name && !option.has_value) {
        option.used = true;
        return true;
      }
    }
    return false;
  }

  const std::string* Get(std::string_view name) {
    for (Option& option : options_) {
      if (option.name == name && option.has_value) {
        option.used = true;
        return &option.value;
      }
    }
    return nullptr;
  }

  std::string GetOr(std::string_view name, std::string fallback) {
    const std::string* value = Get(name);
    return value != nullptr ? *value : fallback;
  }

  Result<std::uint64_t> Number(std::string_view name, std::uint64_t fallback, std::uint64_t low,
                               std::uint64_t high) {
    const std::string* text = Get(name);
    if (text == nullptr) return fallback;
    std::uint64_t value = 0;
    if (text->empty() || text->size() > 20) {
      return Status::Error("--" + std::string(name) + " needs a number");
    }
    for (char c : *text) {
      if (c < '0' || c > '9') return Status::Error("--" + std::string(name) + " needs a number");
      value = value * 10 + static_cast<std::uint64_t>(c - '0');
    }
    if (value < low || value > high) {
      return Status::Error("--" + std::string(name) + " must be between " + std::to_string(low) +
                           " and " + std::to_string(high));
    }
    return value;
  }

  Status Unused() const {
    for (const Option& option : options_) {
      if (!option.used) return Status::Error("unknown option --" + option.name);
    }
    if (!positional_.empty()) return Status::Error("unexpected argument " + positional_[0]);
    return Status();
  }

 private:
  struct Option {
    std::string name;
    std::string value;
    bool has_value = false;
    bool used = false;
  };

  std::vector<Option> options_;
  std::vector<std::string> positional_;
};

std::string Derived(const std::string& graph, const char* extension) {
  std::filesystem::path path(graph);
  path.replace_extension(extension);
  return path.string();
}

Result<EngineFiles> ReadEngineFiles(Args& args, bool snap_default) {
  EngineFiles files;
  const std::string* graph = args.Get("graph");
  if (graph == nullptr) return Status::Error("--graph is required");
  files.graph = *graph;
  files.verify = args.Flag("verify");
  files.snap = snap_default && !args.Flag("no-snap");
  const std::string* ch = args.Get("ch");
  if (ch != nullptr) {
    files.hierarchy = *ch;
  } else if (!args.Flag("no-ch")) {
    std::string guess = Derived(files.graph, ".rrch");
    if (std::filesystem::exists(guess)) files.hierarchy = guess;
  }
  const std::string* landmarks = args.Get("landmarks");
  if (landmarks != nullptr) {
    files.landmarks = *landmarks;
  } else if (!args.Flag("no-landmarks")) {
    std::string guess = Derived(files.graph, ".rrl");
    if (std::filesystem::exists(guess)) files.landmarks = guess;
  }
  return files;
}

double FileMegabytes(const std::string& path) {
  std::error_code ec;
  std::uintmax_t size = std::filesystem::file_size(path, ec);
  return ec ? 0 : static_cast<double>(size) / (1024 * 1024);
}

Status Generate(Args& args) {
  GeneratorOptions options;
  Result<std::uint64_t> nodes = args.Number("nodes", 100000, 64, 200000000);
  if (!nodes.ok()) return nodes.status();
  Result<std::uint64_t> seed = args.Number("seed", 1, 0, 0xffffffffffffffffULL);
  if (!seed.ok()) return seed.status();
  options.nodes = static_cast<std::uint32_t>(*nodes);
  options.seed = *seed;
  std::string metric = args.GetOr("metric", "time");
  if (metric != "time" && metric != "distance") {
    return Status::Error("--metric must be time or distance");
  }
  options.travel_time = metric == "time";
  std::string center = args.GetOr("center", "39.1,-84.5");
  if (std::sscanf(center.c_str(), "%lf,%lf", &options.center_lat, &options.center_lon) != 2) {
    return Status::Error("--center must be LAT,LON");
  }
  if (options.center_lat < -85 || options.center_lat > 85 || options.center_lon < -180 ||
      options.center_lon > 180) {
    return Status::Error("--center is out of range");
  }
  const std::string* prefix = args.Get("out");
  if (prefix == nullptr) return Status::Error("--out is required");
  RR_RETURN_IF_ERROR(args.Unused());
  std::string graph_path = *prefix + ".gr";
  std::string coord_path = *prefix + ".co";
  Timer timer;
  GeneratorSummary summary;
  RR_RETURN_IF_ERROR(GenerateRoadNetwork(options, graph_path, coord_path, &summary));
  std::printf("wrote %s and %s: %llu nodes, %llu arcs, %.1f MB in %.2f s\n", graph_path.c_str(),
              coord_path.c_str(), static_cast<unsigned long long>(summary.nodes),
              static_cast<unsigned long long>(summary.arcs),
              FileMegabytes(graph_path) + FileMegabytes(coord_path), timer.Seconds());
  return Status();
}

Status Import(Args& args) {
  const std::string* graph_path = args.Get("graph");
  const std::string* coord_path = args.Get("coords");
  const std::string* out = args.Get("out");
  if (graph_path == nullptr || coord_path == nullptr || out == nullptr) {
    return Status::Error("--graph, --coords and --out are required");
  }
  bool keep_all = args.Flag("keep-all");
  bool reorder = !args.Flag("no-reorder");
  RR_RETURN_IF_ERROR(args.Unused());
  Timer timer;
  Result<RoadArcs> dimacs = ReadRoadGraph(*graph_path);
  if (!dimacs.ok()) return dimacs.status();
  double read_graph = timer.Seconds();
  Result<std::vector<LatLon>> coords = ReadRoadCoordinates(*coord_path, dimacs->node_count);
  if (!coords.ok()) return coords.status();
  std::printf("read %u nodes and %zu arcs in %.2f s\n", dimacs->node_count, dimacs->arcs.size(),
              timer.Seconds());
  (void)read_graph;
  Result<Graph> graph =
      Graph::FromArcs(dimacs->node_count, std::move(dimacs->arcs), std::move(*coords));
  if (!graph.ok()) return graph.status();
  std::printf("built the graph: %u nodes, %u arcs after dropping loops and parallel arcs\n",
              graph->node_count(), graph->arc_count());
  Graph current = std::move(*graph);
  if (!keep_all) {
    ComponentInfo info;
    Result<Graph> largest = LargestStrongComponent(current, &info);
    if (!largest.ok()) return largest.status();
    std::printf("kept the largest strongly connected component: %u of %u nodes in %u components\n",
                info.kept_nodes, current.node_count(), info.components);
    current = std::move(*largest);
  }
  if (reorder) {
    std::vector<NodeId> numbering = HilbertNumbering(current);
    Result<Graph> ordered = Renumber(current, numbering);
    if (!ordered.ok()) return ordered.status();
    current = std::move(*ordered);
    std::printf("renumbered the nodes along a Hilbert curve\n");
  }
  RR_RETURN_IF_ERROR(current.Save(*out));
  std::printf("wrote %s: %u nodes, %u arcs, %.1f MB, total %.2f s\n", out->c_str(),
              current.node_count(), current.arc_count(), FileMegabytes(*out), timer.Seconds());
  return Status();
}

Status Prepare(Args& args) {
  const std::string* graph_path = args.Get("graph");
  if (graph_path == nullptr) return Status::Error("--graph is required");
  bool verify = args.Flag("verify");
  bool quiet = args.Flag("quiet");
  Result<std::uint64_t> threads =
      args.Number("threads", static_cast<std::uint64_t>(ThreadPool::DefaultThreads()), 1, 1024);
  if (!threads.ok()) return threads.status();
  Result<std::uint64_t> settle = args.Number("settle-limit", 500, 1, 1000000);
  if (!settle.ok()) return settle.status();
  Result<std::uint64_t> count = args.Number("landmark-count", 16, 1, 64);
  if (!count.ok()) return count.status();
  Result<std::uint64_t> seed = args.Number("seed", 1, 0, 0xffffffffffffffffULL);
  if (!seed.ok()) return seed.status();
  bool want_ch = !args.Flag("no-ch");
  bool want_landmarks = !args.Flag("no-landmarks");
  std::string ch_path = args.GetOr("ch", Derived(*graph_path, ".rrch"));
  std::string landmark_path = args.GetOr("landmarks", Derived(*graph_path, ".rrl"));
  RR_RETURN_IF_ERROR(args.Unused());
  Result<Graph> graph = Graph::Load(*graph_path, verify);
  if (!graph.ok()) return graph.status();
  std::printf("loaded %s: %u nodes, %u arcs\n", graph_path->c_str(), graph->node_count(),
              graph->arc_count());
  if (want_ch) {
    ContractOptions options;
    options.threads = static_cast<int>(*threads);
    options.settle_limit = static_cast<std::uint32_t>(*settle);
    options.verbose = !quiet;
    ContractReport report;
    std::printf("contracting with %d threads\n", options.threads);
    Result<ChGraph> ch = Contract(*graph, options, &report);
    if (!ch.ok()) return ch.status();
    RR_RETURN_IF_ERROR(ch->Save(ch_path, *graph));
    std::printf(
        "wrote %s: %llu arcs of which %llu are shortcuts, %u rounds, %llu witness searches, "
        "largest degree %u, %.1f MB, %.2f s\n",
        ch_path.c_str(), static_cast<unsigned long long>(ch->arc_count()),
        static_cast<unsigned long long>(ch->shortcut_count()), report.rounds,
        static_cast<unsigned long long>(report.witness_searches), report.max_degree,
        FileMegabytes(ch_path), report.seconds);
  }
  if (want_landmarks) {
    ThreadPool pool(static_cast<int>(*threads));
    Timer timer;
    Result<Landmarks> landmarks =
        Landmarks::Build(*graph, static_cast<std::uint32_t>(*count), *seed, pool);
    if (!landmarks.ok()) return landmarks.status();
    RR_RETURN_IF_ERROR(landmarks->Save(landmark_path, *graph));
    std::printf("wrote %s: %u landmarks, %.1f MB, %.2f s\n", landmark_path.c_str(),
                landmarks->count(), FileMegabytes(landmark_path), timer.Seconds());
  }
  return Status();
}

Result<NodeId> ReadEndpoint(Args& args, const Engine& engine, const char* point_name,
                            const char* node_name) {
  if (const std::string* text = args.Get(node_name)) {
    std::uint64_t value = 0;
    for (char c : *text) {
      if (c < '0' || c > '9')
        return Status::Error(std::string("--") + node_name + " needs a node id");
      value = value * 10 + static_cast<std::uint64_t>(c - '0');
      if (value >= engine.graph().node_count()) {
        return Status::Error(std::string("--") + node_name + " is out of range");
      }
    }
    if (text->empty()) return Status::Error(std::string("--") + node_name + " needs a node id");
    return static_cast<NodeId>(value);
  }
  if (const std::string* text = args.Get(point_name)) {
    double lat = 0;
    double lon = 0;
    if (std::sscanf(text->c_str(), "%lf,%lf", &lat, &lon) != 2) {
      return Status::Error(std::string("--") + point_name + " must be LAT,LON");
    }
    if (!engine.can_snap()) return Status::Error("snapping is turned off");
    return engine.Nearest(FromDegrees(lat, lon));
  }
  return Status::Error(std::string("--") + point_name + " or --" + node_name + " is required");
}

Status Route(Args& args) {
  Result<EngineFiles> files = ReadEngineFiles(args, true);
  if (!files.ok()) return files.status();
  Algorithm algorithm = Algorithm::kAuto;
  if (const std::string* name = args.Get("algorithm")) {
    if (!ParseAlgorithm(*name, &algorithm)) return Status::Error("unknown algorithm " + *name);
  }
  std::string path_mode = args.GetOr("path", "nodes");
  if (path_mode != "nodes" && path_mode != "polyline" && path_mode != "none") {
    return Status::Error("--path must be nodes, polyline or none");
  }
  Result<std::unique_ptr<Engine>> engine = Engine::Open(*files);
  if (!engine.ok()) return engine.status();
  Result<NodeId> source = ReadEndpoint(args, **engine, "from", "from-node");
  if (!source.ok()) return source.status();
  Result<NodeId> target = ReadEndpoint(args, **engine, "to", "to-node");
  if (!target.ok()) return target.status();
  RR_RETURN_IF_ERROR(args.Unused());
  Service service(**engine, 1);
  HttpRequest request;
  request.method = "GET";
  request.path = "/route";
  request.query.emplace_back("from_node", std::to_string(*source));
  request.query.emplace_back("to_node", std::to_string(*target));
  request.query.emplace_back("algorithm", AlgorithmName(algorithm));
  request.query.emplace_back("path", path_mode);
  HttpResponse response = service.Handle(request);
  std::fputs(response.body.c_str(), stdout);
  return response.status == 200 ? Status() : Status::Error("the query failed");
}

struct Candidate {
  const char* name;
  Algorithm algorithm;
  bool needs_hierarchy;
  bool needs_landmarks;
};

const Candidate kCandidates[] = {
    {"dijkstra", Algorithm::kDijkstra, false, false},
    {"bidirectional", Algorithm::kBidirectional, false, false},
    {"astar", Algorithm::kAStar, false, false},
    {"bidirectional-astar", Algorithm::kBidirectionalAStar, false, false},
    {"alt", Algorithm::kAlt, false, true},
    {"bidirectional-alt", Algorithm::kBidirectionalAlt, false, true},
    {"ch", Algorithm::kHierarchy, true, false},
};

std::vector<Candidate> SelectAlgorithms(const Engine& engine, const std::string* list,
                                        Status* status) {
  std::vector<Candidate> chosen;
  if (list == nullptr) {
    for (const Candidate& candidate : kCandidates) {
      if (candidate.needs_hierarchy && engine.hierarchy() == nullptr) continue;
      if (candidate.needs_landmarks && engine.landmarks() == nullptr) continue;
      chosen.push_back(candidate);
    }
    return chosen;
  }
  std::string_view text(*list);
  while (!text.empty()) {
    std::size_t comma = text.find(',');
    std::string_view name = text.substr(0, comma);
    text = comma == std::string_view::npos ? std::string_view() : text.substr(comma + 1);
    bool found = false;
    for (const Candidate& candidate : kCandidates) {
      if (name != candidate.name) continue;
      if (candidate.needs_hierarchy && engine.hierarchy() == nullptr) {
        *status = Status::Error("ch needs a hierarchy file");
        return {};
      }
      if (candidate.needs_landmarks && engine.landmarks() == nullptr) {
        *status = Status::Error(std::string(name) + " needs a landmark file");
        return {};
      }
      chosen.push_back(candidate);
      found = true;
      break;
    }
    if (!found) {
      *status = Status::Error("unknown algorithm " + std::string(name));
      return {};
    }
  }
  if (chosen.empty()) *status = Status::Error("--algorithms is empty");
  return chosen;
}

double Quantile(std::vector<double>& values, double q) {
  if (values.empty()) return 0;
  std::size_t index = static_cast<std::size_t>(q * (values.size() - 1) + 0.5);
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

Status Bench(Args& args) {
  Result<EngineFiles> files = ReadEngineFiles(args, false);
  if (!files.ok()) return files.status();
  Result<std::uint64_t> queries = args.Number("queries", 1000, 1, 10000000);
  if (!queries.ok()) return queries.status();
  Result<std::uint64_t> seed = args.Number("seed", 1, 0, 0xffffffffffffffffULL);
  if (!seed.ok()) return seed.status();
  Result<std::uint64_t> table = args.Number("table", 0, 0, 4096);
  if (!table.ok()) return table.status();
  Result<std::uint64_t> threads =
      args.Number("threads", static_cast<std::uint64_t>(ThreadPool::DefaultThreads()), 1, 1024);
  if (!threads.ok()) return threads.status();
  const std::string* list = args.Get("algorithms");
  std::string list_copy;
  if (list != nullptr) list_copy = *list;
  Result<std::unique_ptr<Engine>> engine = Engine::Open(*files);
  if (!engine.ok()) return engine.status();
  RR_RETURN_IF_ERROR(args.Unused());
  Status select_status;
  std::vector<Candidate> chosen =
      SelectAlgorithms(**engine, list != nullptr ? &list_copy : nullptr, &select_status);
  if (!select_status.ok()) return select_status;
  const std::uint32_t n = (*engine)->graph().node_count();
  std::vector<std::pair<NodeId, NodeId>> pairs;
  pairs.reserve(*queries);
  Rng rng(*seed);
  for (std::uint64_t i = 0; i < *queries; ++i) {
    pairs.push_back({rng.Below(n), rng.Below(n)});
  }
  Querier querier(**engine);
  std::printf("graph %s: %u nodes, %u arcs\n", files->graph.c_str(), n,
              (*engine)->graph().arc_count());
  std::printf("%llu random queries, seed %llu\n", static_cast<unsigned long long>(*queries),
              static_cast<unsigned long long>(*seed));
  const std::size_t warmup = std::min<std::size_t>(pairs.size(), 50);
  for (const Candidate& candidate : chosen) {
    for (std::size_t i = 0; i < warmup; ++i) {
      Result<RouteResult> ignored =
          querier.Route(pairs[i].first, pairs[i].second, candidate.algorithm, false);
      if (!ignored.ok()) return ignored.status();
    }
  }
  std::vector<std::vector<double>> times(chosen.size());
  std::vector<std::uint64_t> settled(chosen.size(), 0);
  std::vector<std::uint64_t> relaxed(chosen.size(), 0);
  std::vector<std::uint64_t> reached(chosen.size(), 0);
  for (std::vector<double>& t : times) t.reserve(pairs.size());
  std::uint64_t mismatches = 0;
  for (const auto& [source, target] : pairs) {
    Weight first = kInf;
    for (std::size_t a = 0; a < chosen.size(); ++a) {
      Result<RouteResult> result = querier.Route(source, target, chosen[a].algorithm, false);
      if (!result.ok()) return result.status();
      times[a].push_back(result->milliseconds);
      settled[a] += result->stats.settled;
      relaxed[a] += result->stats.relaxed;
      reached[a] += result->weight != kInf ? 1 : 0;
      if (a == 0) {
        first = result->weight;
      } else if (result->weight != first) {
        ++mismatches;
      }
    }
  }
  std::printf("\n%-20s %10s %10s %10s %12s %12s\n", "algorithm", "mean ms", "median", "p99",
              "settled", "relaxed");
  double baseline = 0;
  for (std::size_t a = 0; a < chosen.size(); ++a) {
    double sum = 0;
    for (double t : times[a]) sum += t;
    double mean = sum / static_cast<double>(times[a].size());
    if (a == 0) baseline = mean;
    std::printf("%-20s %10.4f %10.4f %10.4f %12llu %12llu\n", chosen[a].name, mean,
                Quantile(times[a], 0.5), Quantile(times[a], 0.99),
                static_cast<unsigned long long>(settled[a] / times[a].size()),
                static_cast<unsigned long long>(relaxed[a] / times[a].size()));
  }
  if (chosen.size() > 1 && baseline > 0) {
    std::printf("\nspeedup against %s:\n", chosen[0].name);
    for (std::size_t a = 1; a < chosen.size(); ++a) {
      double sum = 0;
      for (double t : times[a]) sum += t;
      double mean = sum / static_cast<double>(times[a].size());
      std::printf("  %-20s %8.1fx\n", chosen[a].name, baseline / mean);
    }
  }
  std::printf("\nreachable pairs: %llu of %llu, distance mismatches: %llu\n",
              static_cast<unsigned long long>(reached[0]),
              static_cast<unsigned long long>(pairs.size()),
              static_cast<unsigned long long>(mismatches));
  if (*table > 0) {
    std::vector<NodeId> sources;
    std::vector<NodeId> targets;
    for (std::uint64_t i = 0; i < *table; ++i) {
      sources.push_back(rng.Below(n));
      targets.push_back(rng.Below(n));
    }
    ThreadPool pool(static_cast<int>(*threads));
    Result<std::vector<Weight>> ignored = querier.Table(sources, targets, pool);
    if (!ignored.ok()) return ignored.status();
    Timer timer;
    Result<std::vector<Weight>> matrix = querier.Table(sources, targets, pool);
    if (!matrix.ok()) return matrix.status();
    double seconds = timer.Seconds();
    std::uint64_t cells = *table * *table;
    std::printf("\n%llu by %llu table on %llu threads: %.3f s, %.2f us per cell\n",
                static_cast<unsigned long long>(*table), static_cast<unsigned long long>(*table),
                static_cast<unsigned long long>(*threads), seconds,
                seconds * 1e6 / static_cast<double>(cells));
  }
  return mismatches == 0 ? Status() : Status::Error("the algorithms disagree");
}

Status Difftest(Args& args) {
  Result<EngineFiles> files = ReadEngineFiles(args, false);
  if (!files.ok()) return files.status();
  Result<std::uint64_t> queries = args.Number("queries", 1000, 1, 10000000);
  if (!queries.ok()) return queries.status();
  Result<std::uint64_t> seed = args.Number("seed", 1, 0, 0xffffffffffffffffULL);
  if (!seed.ok()) return seed.status();
  Result<std::unique_ptr<Engine>> engine = Engine::Open(*files);
  if (!engine.ok()) return engine.status();
  RR_RETURN_IF_ERROR(args.Unused());
  Status select_status;
  std::vector<Candidate> chosen = SelectAlgorithms(**engine, nullptr, &select_status);
  if (!select_status.ok()) return select_status;
  const Graph& graph = (*engine)->graph();
  Querier querier(**engine);
  Rng rng(*seed);
  std::uint64_t failures = 0;
  std::uint64_t unreachable = 0;
  for (std::uint64_t q = 0; q < *queries; ++q) {
    NodeId source = rng.Below(graph.node_count());
    NodeId target = rng.Below(graph.node_count());
    Result<RouteResult> reference = querier.Route(source, target, Algorithm::kDijkstra, true);
    if (!reference.ok()) return reference.status();
    if (reference->weight == kInf) ++unreachable;
    for (const Candidate& candidate : chosen) {
      Result<RouteResult> result = querier.Route(source, target, candidate.algorithm, true);
      if (!result.ok()) return result.status();
      if (result->weight != reference->weight) {
        ++failures;
        if (failures < 11) {
          std::printf("%s disagrees on %u to %u: %u instead of %u\n", candidate.name, source,
                      target, result->weight, reference->weight);
        }
        continue;
      }
      if (result->weight == kInf) continue;
      const std::vector<NodeId>& path = result->path;
      std::string problem;
      if (path.empty() || path.front() != source || path.back() != target) {
        problem = "the path does not run from the source to the target";
      } else {
        std::uint64_t sum = 0;
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
          Weight arc = graph.ArcWeight(path[i], path[i + 1]);
          if (arc == kInf) {
            problem = "the path uses an arc that is not in the graph";
            break;
          }
          sum += arc;
        }
        if (problem.empty() && sum != result->weight) {
          problem = "the path weight is " + std::to_string(sum) + " but the query returned " +
                    std::to_string(result->weight);
        }
      }
      if (!problem.empty()) {
        ++failures;
        if (failures < 11) {
          std::printf("%s on %u to %u: %s\n", candidate.name, source, target, problem.c_str());
        }
      }
    }
  }
  std::printf(
      "%llu queries against plain Dijkstra over %zu algorithms, %llu unreachable pairs, "
      "%llu failures\n",
      static_cast<unsigned long long>(*queries), chosen.size(),
      static_cast<unsigned long long>(unreachable), static_cast<unsigned long long>(failures));
  return failures == 0 ? Status() : Status::Error("the difftest failed");
}

Status Serve(Args& args) {
  Result<EngineFiles> files = ReadEngineFiles(args, true);
  if (!files.ok()) return files.status();
  std::string host = args.GetOr("host", "127.0.0.1");
  Result<std::uint64_t> port = args.Number("port", 8080, 0, 65535);
  if (!port.ok()) return port.status();
  Result<std::uint64_t> threads =
      args.Number("threads", static_cast<std::uint64_t>(ThreadPool::DefaultThreads()), 1, 1024);
  if (!threads.ok()) return threads.status();
  Result<std::unique_ptr<Engine>> engine = Engine::Open(*files);
  if (!engine.ok()) return engine.status();
  RR_RETURN_IF_ERROR(args.Unused());
  Service service(**engine, static_cast<int>(*threads));
  HttpServer server;
  RR_RETURN_IF_ERROR(
      server.Start(host, static_cast<std::uint16_t>(*port), static_cast<int>(*threads),
                   [&service](const HttpRequest& request) { return service.Handle(request); }));
  std::printf("serving %u nodes on http://%s:%u with %llu threads\n",
              (*engine)->graph().node_count(), host.c_str(), server.port(),
              static_cast<unsigned long long>(*threads));
  std::printf("try /health, /route, /nearest and /table\n");
  std::fflush(stdout);
  server.Wait();
  return Status();
}

int Main(int argc, char** argv) {
  if (argc < 2) {
    std::fputs(kUsage, stderr);
    return 2;
  }
  std::string_view command(argv[1]);
  if (command == "--help" || command == "-h" || command == "help") {
    std::fputs(kUsage, stdout);
    return 0;
  }
  Args args(argc - 2, argv + 2);
  Status status;
  if (command == "generate") {
    status = Generate(args);
  } else if (command == "import") {
    status = Import(args);
  } else if (command == "prepare") {
    status = Prepare(args);
  } else if (command == "route") {
    status = Route(args);
  } else if (command == "bench") {
    status = Bench(args);
  } else if (command == "difftest") {
    status = Difftest(args);
  } else if (command == "serve") {
    status = Serve(args);
  } else {
    std::fprintf(stderr, "roadroute: unknown command %s\n\n", argv[1]);
    std::fputs(kUsage, stderr);
    return 2;
  }
  if (!status.ok()) {
    std::fprintf(stderr, "roadroute: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}

}
}

int main(int argc, char** argv) { return rr::Main(argc, argv); }
