#include "generator.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <memory>
#include <string_view>

#include "geo.h"
#include "io.h"

namespace rr {

namespace {

constexpr double kSpacingDegrees = 0.0025;
constexpr std::uint32_t kHighwayEvery = 64;
constexpr std::uint32_t kArterialEvery = 8;
constexpr std::uint32_t kRampEvery = 16;

enum class RoadClass { kLocal, kArterial };

class Sink {
 public:
  virtual ~Sink() = default;
  virtual Status Node(std::uint64_t id, LatLon p) = 0;
  virtual Status Arc(std::uint64_t from, std::uint64_t to, Weight weight) = 0;
};

class Counter : public Sink {
 public:
  Status Node(std::uint64_t, LatLon) override {
    ++nodes;
    return Status();
  }
  Status Arc(std::uint64_t, std::uint64_t, Weight) override {
    ++arcs;
    return Status();
  }

  std::uint64_t nodes = 0;
  std::uint64_t arcs = 0;
};

class TextOut {
 public:
  static Result<TextOut> Create(const std::string& path) {
    Result<std::unique_ptr<FileWriter>> file = FileWriter::Create(path);
    if (!file.ok()) return file.status();
    TextOut out;
    out.file_ = std::move(*file);
    out.buffer_.reserve(kFlushAt + 128);
    return out;
  }

  void Char(char c) { buffer_.push_back(c); }
  void Text(std::string_view s) { buffer_.append(s); }
  void Number(std::int64_t v) {
    char digits[24];
    auto result = std::to_chars(digits, digits + sizeof(digits), v);
    buffer_.append(digits, result.ptr);
  }
  Status Flush(bool force) {
    if (!force && buffer_.size() < kFlushAt) return Status();
    Status status = file_->Write(buffer_);
    buffer_.clear();
    return status;
  }
  Status Finish() {
    RR_RETURN_IF_ERROR(Flush(true));
    return file_->Finish();
  }

 private:
  static constexpr std::size_t kFlushAt = 1 << 20;
  std::unique_ptr<FileWriter> file_;
  std::string buffer_;
};

class Writer : public Sink {
 public:
  Writer(TextOut graph, TextOut coords) : graph_(std::move(graph)), coords_(std::move(coords)) {}

  Status Header(std::uint64_t nodes, std::uint64_t arcs) {
    graph_.Text("c synthetic road network written by roadroute generate\np sp ");
    graph_.Number(static_cast<std::int64_t>(nodes));
    graph_.Char(' ');
    graph_.Number(static_cast<std::int64_t>(arcs));
    graph_.Char('\n');
    coords_.Text("c synthetic road network written by roadroute generate\np aux sp co ");
    coords_.Number(static_cast<std::int64_t>(nodes));
    coords_.Char('\n');
    return Status();
  }

  Status Node(std::uint64_t id, LatLon p) override {
    coords_.Text("v ");
    coords_.Number(static_cast<std::int64_t>(id + 1));
    coords_.Char(' ');
    coords_.Number(p.lon);
    coords_.Char(' ');
    coords_.Number(p.lat);
    coords_.Char('\n');
    return coords_.Flush(false);
  }

  Status Arc(std::uint64_t from, std::uint64_t to, Weight weight) override {
    graph_.Text("a ");
    graph_.Number(static_cast<std::int64_t>(from + 1));
    graph_.Char(' ');
    graph_.Number(static_cast<std::int64_t>(to + 1));
    graph_.Char(' ');
    graph_.Number(weight);
    graph_.Char('\n');
    return graph_.Flush(false);
  }

  Status Finish() {
    RR_RETURN_IF_ERROR(graph_.Finish());
    return coords_.Finish();
  }

 private:
  TextOut graph_;
  TextOut coords_;
};

double Unit(std::uint64_t h, int shift) {
  return static_cast<double>((h >> shift) & 0xffff) / 65536.0;
}

class Network {
 public:
  explicit Network(const GeneratorOptions& options) : options_(options) {
    side_ = std::max<std::uint32_t>(
        8, static_cast<std::uint32_t>(std::ceil(std::sqrt(options.nodes / 3.3))));
    highways_ = (side_ - 1) / kHighwayEvery + 1;
    grid_nodes_ = std::uint64_t{side_} * side_;
    highway_nodes_ = 2 * std::uint64_t{highways_} * side_;
    dlat_ = kSpacingDegrees;
    dlon_ = kSpacingDegrees / std::cos(options.center_lat * 3.14159265358979323846 / 180.0);
  }

  bool Fits() const {
    double half_lat = side_ * dlat_ / 2 + 1;
    double half_lon = side_ * dlon_ / 2 + 1;
    return std::fabs(options_.center_lat) + half_lat < 85 &&
           std::fabs(options_.center_lon) + half_lon < 180;
  }

  Status Emit(Sink& sink) {
    next_shape_ = grid_nodes_ + highway_nodes_;
    for (std::uint32_t r = 0; r < side_; ++r) {
      for (std::uint32_t c = 0; c < side_; ++c) {
        RR_RETURN_IF_ERROR(sink.Node(GridId(r, c), GridPoint(r, c)));
      }
    }
    for (std::uint32_t k = 0; k < highways_; ++k) {
      for (std::uint32_t i = 0; i < side_; ++i) {
        RR_RETURN_IF_ERROR(sink.Node(RowHighwayId(k, i), RowHighwayPoint(k, i)));
      }
    }
    for (std::uint32_t k = 0; k < highways_; ++k) {
      for (std::uint32_t i = 0; i < side_; ++i) {
        RR_RETURN_IF_ERROR(sink.Node(ColumnHighwayId(k, i), ColumnHighwayPoint(k, i)));
      }
    }
    for (std::uint32_t r = 0; r < side_; ++r) {
      for (std::uint32_t c = 0; c < side_; ++c) {
        if (c + 1 < side_) {
          RoadClass cls = r % kArterialEvery == 0 ? RoadClass::kArterial : RoadClass::kLocal;
          RR_RETURN_IF_ERROR(Street(sink, GridId(r, c), GridId(r, c + 1), cls, 0));
        }
        if (r + 1 < side_) {
          RoadClass cls = c % kArterialEvery == 0 ? RoadClass::kArterial : RoadClass::kLocal;
          RR_RETURN_IF_ERROR(Street(sink, GridId(r, c), GridId(r + 1, c), cls, 1));
        }
      }
    }
    for (std::uint32_t k = 0; k < highways_; ++k) {
      const std::uint32_t line = k * kHighwayEvery;
      for (std::uint32_t i = 0; i < side_; ++i) {
        if (i + 1 < side_) {
          RR_RETURN_IF_ERROR(Link(sink, RowHighwayId(k, i), RowHighwayPoint(k, i),
                                  RowHighwayId(k, i + 1), RowHighwayPoint(k, i + 1), 1.02,
                                  SpeedFor(RowHighwayId(k, i), 100, 20)));
          RR_RETURN_IF_ERROR(Link(sink, ColumnHighwayId(k, i), ColumnHighwayPoint(k, i),
                                  ColumnHighwayId(k, i + 1), ColumnHighwayPoint(k, i + 1), 1.02,
                                  SpeedFor(ColumnHighwayId(k, i), 100, 20)));
        }
        if (i % kRampEvery == 0) {
          RR_RETURN_IF_ERROR(Link(sink, RowHighwayId(k, i), RowHighwayPoint(k, i), GridId(line, i),
                                  GridPoint(line, i), 1.3, 40));
          RR_RETURN_IF_ERROR(Link(sink, ColumnHighwayId(k, i), ColumnHighwayPoint(k, i),
                                  GridId(i, line), GridPoint(i, line), 1.3, 40));
        }
        if (i % kHighwayEvery == 0) {
          std::uint32_t other = i / kHighwayEvery;
          RR_RETURN_IF_ERROR(Link(sink, RowHighwayId(k, i), RowHighwayPoint(k, i),
                                  ColumnHighwayId(other, line), ColumnHighwayPoint(other, line),
                                  1.5, 60));
        }
      }
    }
    return Status();
  }

 private:
  std::uint64_t Hash(std::uint64_t key) const { return Mix64(Mix64(options_.seed) ^ key); }

  std::uint64_t GridId(std::uint32_t r, std::uint32_t c) const {
    return std::uint64_t{r} * side_ + c;
  }
  std::uint64_t RowHighwayId(std::uint32_t k, std::uint32_t c) const {
    return grid_nodes_ + std::uint64_t{k} * side_ + c;
  }
  std::uint64_t ColumnHighwayId(std::uint32_t k, std::uint32_t r) const {
    return grid_nodes_ + std::uint64_t{highways_} * side_ + std::uint64_t{k} * side_ + r;
  }

  LatLon Place(double row, double column) const {
    return FromDegrees(options_.center_lat + (row - side_ / 2.0) * dlat_,
                       options_.center_lon + (column - side_ / 2.0) * dlon_);
  }
  LatLon GridPoint(std::uint32_t r, std::uint32_t c) const {
    std::uint64_t h = Hash(GridId(r, c) * 4 + 3);
    return Place(r + (Unit(h, 0) - 0.5) * 0.6, c + (Unit(h, 16) - 0.5) * 0.6);
  }
  LatLon RowHighwayPoint(std::uint32_t k, std::uint32_t c) const {
    return Place(k * kHighwayEvery + 0.35, c);
  }
  LatLon ColumnHighwayPoint(std::uint32_t k, std::uint32_t r) const {
    return Place(r, k * kHighwayEvery + 0.35);
  }

  double SpeedFor(std::uint64_t key, double base, double spread) const {
    return base + Unit(Hash(key * 4 + 1), 8) * spread;
  }

  Weight Cost(LatLon a, LatLon b, double curvature, double speed_kmh) const {
    double meters = HaversineMeters(a, b) * curvature;
    double value = options_.travel_time ? meters / (speed_kmh / 3.6) * 10 : meters;
    return static_cast<Weight>(std::max<long long>(1, std::llround(value)));
  }

  Status Link(Sink& sink, std::uint64_t a, LatLon pa, std::uint64_t b, LatLon pb, double curvature,
              double speed) {
    Weight w = Cost(pa, pb, curvature, speed);
    RR_RETURN_IF_ERROR(sink.Arc(a, b, w));
    return sink.Arc(b, a, w);
  }

  Status Street(Sink& sink, std::uint64_t a, std::uint64_t b, RoadClass cls, std::uint64_t axis) {
    std::uint64_t h = Hash(a * 4 + axis);
    bool local = cls == RoadClass::kLocal;
    if (Unit(h, 0) >= (local ? 0.82 : 0.98)) return Status();
    std::uint32_t shapes = static_cast<std::uint32_t>((h >> 16) % (local ? 4 : 2));
    bool oneway = local && ((h >> 24) & 0xff) < 10;
    bool forward = ((h >> 32) & 1) != 0;
    double curvature = 1.0 + static_cast<double>((h >> 40) & 0xff) / 255.0 * 0.15;
    double speed = local ? 25 + static_cast<double>((h >> 48) & 0xff) / 255.0 * 20
                         : 50 + static_cast<double>((h >> 48) & 0xff) / 255.0 * 20;
    std::uint32_t ra = static_cast<std::uint32_t>(a / side_);
    std::uint32_t ca = static_cast<std::uint32_t>(a % side_);
    std::uint32_t rb = static_cast<std::uint32_t>(b / side_);
    std::uint32_t cb = static_cast<std::uint32_t>(b % side_);
    LatLon pa = GridPoint(ra, ca);
    LatLon pb = GridPoint(rb, cb);
    std::uint64_t prev = a;
    LatLon pprev = pa;
    for (std::uint32_t s = 1; s <= shapes + 1; ++s) {
      std::uint64_t id = b;
      LatLon p = pb;
      if (s <= shapes) {
        double t = static_cast<double>(s) / (shapes + 1);
        double wiggle = (Unit(Hash(h + s), 0) - 0.5) * 0.3;
        double row = ra + (rb - static_cast<double>(ra)) * t + (axis == 0 ? wiggle : 0);
        double column = ca + (cb - static_cast<double>(ca)) * t + (axis == 1 ? wiggle : 0);
        p = Place(row, column);
        id = next_shape_++;
        RR_RETURN_IF_ERROR(sink.Node(id, p));
      }
      Weight w = Cost(pprev, p, curvature, speed);
      if (!oneway || forward) RR_RETURN_IF_ERROR(sink.Arc(prev, id, w));
      if (!oneway || !forward) RR_RETURN_IF_ERROR(sink.Arc(id, prev, w));
      prev = id;
      pprev = p;
    }
    return Status();
  }

  GeneratorOptions options_;
  std::uint32_t side_ = 0;
  std::uint32_t highways_ = 0;
  std::uint64_t grid_nodes_ = 0;
  std::uint64_t highway_nodes_ = 0;
  std::uint64_t next_shape_ = 0;
  double dlat_ = 0;
  double dlon_ = 0;
};

}

Status GenerateRoadNetwork(const GeneratorOptions& options, const std::string& graph_path,
                           const std::string& coord_path, GeneratorSummary* summary) {
  if (options.nodes < 64 || options.nodes > 200000000) {
    return Status::Error("node count must be between 64 and 200000000");
  }
  Network network(options);
  if (!network.Fits()) return Status::Error("the network does not fit around that center");
  Counter counter;
  RR_RETURN_IF_ERROR(network.Emit(counter));
  if (counter.nodes > kMaxNodes || counter.arcs >= 0xffffffffULL) {
    return Status::Error("the network is too large");
  }
  Result<TextOut> graph = TextOut::Create(graph_path);
  if (!graph.ok()) return graph.status();
  Result<TextOut> coords = TextOut::Create(coord_path);
  if (!coords.ok()) return coords.status();
  Writer writer(std::move(*graph), std::move(*coords));
  RR_RETURN_IF_ERROR(writer.Header(counter.nodes, counter.arcs));
  RR_RETURN_IF_ERROR(network.Emit(writer));
  RR_RETURN_IF_ERROR(writer.Finish());
  if (summary != nullptr) {
    summary->nodes = counter.nodes;
    summary->arcs = counter.arcs;
  }
  return Status();
}

}
