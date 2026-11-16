#include "dimacs.h"

#include <algorithm>
#include <string_view>

#include "geo.h"
#include "gzip.h"

namespace rr {

namespace {

void SkipSpaces(std::string_view& s) {
  std::size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
  s.remove_prefix(i);
}

bool ParseUnsigned(std::string_view& s, std::uint64_t* out) {
  SkipSpaces(s);
  std::size_t i = 0;
  std::uint64_t value = 0;
  while (i < s.size() && s[i] >= '0' && s[i] <= '9') {
    if (value > (std::uint64_t{1} << 40)) return false;
    value = value * 10 + static_cast<std::uint64_t>(s[i] - '0');
    ++i;
  }
  if (i == 0) return false;
  s.remove_prefix(i);
  *out = value;
  return true;
}

bool ParseSigned(std::string_view& s, std::int64_t* out) {
  SkipSpaces(s);
  bool negative = !s.empty() && s[0] == '-';
  if (negative || (!s.empty() && s[0] == '+')) s.remove_prefix(1);
  if (s.empty() || s[0] < '0' || s[0] > '9') return false;
  std::uint64_t magnitude = 0;
  if (!ParseUnsigned(s, &magnitude)) return false;
  *out = negative ? -static_cast<std::int64_t>(magnitude) : static_cast<std::int64_t>(magnitude);
  return true;
}

bool ParseWord(std::string_view& s, std::string_view word) {
  SkipSpaces(s);
  if (s.substr(0, word.size()) != word) return false;
  s.remove_prefix(word.size());
  return s.empty() || s[0] == ' ' || s[0] == '\t';
}

bool ParseName(std::string_view& s, std::size_t limit) {
  SkipSpaces(s);
  std::size_t i = 0;
  while (i < s.size() && s[i] != ' ' && s[i] != '\t') ++i;
  if (i == 0 || i > limit) return false;
  s.remove_prefix(i);
  return true;
}

bool OnlySpaces(std::string_view s) {
  SkipSpaces(s);
  return s.empty();
}

Status LineError(const std::string& path, std::uint64_t line, const std::string& what) {
  return Status::Error(path + ":" + std::to_string(line) + ": " + what);
}

enum class Layout { kDimacs, kCompact };

class TextFile {
 public:
  static Result<TextFile> Open(const std::string& path) {
    Result<std::unique_ptr<ByteSource>> source = OpenTextSource(path);
    if (!source.ok()) return source.status();
    return TextFile(path, std::move(*source));
  }

  Result<bool> NextContentLine(std::string_view* line) {
    while (true) {
      Result<bool> more = reader_.Next(line);
      if (!more.ok()) return Status::Error(path_ + ": " + more.status().message());
      if (!*more) return false;
      if (line->empty() || (*line)[0] == 'c') continue;
      return true;
    }
  }

  std::uint64_t line_number() const { return reader_.line_number(); }
  const std::string& path() const { return path_; }

 private:
  TextFile(std::string path, std::unique_ptr<ByteSource> source)
      : path_(std::move(path)), reader_(std::move(source)) {}

  std::string path_;
  LineReader reader_;
};

}

Result<RoadArcs> ReadRoadGraph(const std::string& path) {
  Result<TextFile> file = TextFile::Open(path);
  if (!file.ok()) return file.status();
  TextFile& text = *file;
  RoadArcs graph;
  std::uint64_t expected_arcs = 0;
  Layout layout = Layout::kDimacs;
  std::string_view line;
  Result<bool> first = text.NextContentLine(&line);
  if (!first.ok()) return first.status();
  if (!*first) return Status::Error(path + ": the file has no header line");
  {
    std::uint64_t nodes = 0;
    std::uint64_t line_no = text.line_number();
    if (!line.empty() && line[0] == 'p') {
      line.remove_prefix(1);
      if (!ParseWord(line, "sp") || !ParseUnsigned(line, &nodes) ||
          !ParseUnsigned(line, &expected_arcs) || !OnlySpaces(line)) {
        return LineError(path, line_no, "malformed problem line");
      }
    } else {
      layout = Layout::kCompact;
      if (!ParseName(line, 32) || !ParseName(line, 32) || !ParseUnsigned(line, &nodes) ||
          !ParseUnsigned(line, &expected_arcs) || !OnlySpaces(line)) {
        return LineError(path, line_no, "malformed header line");
      }
    }
    if (nodes == 0 || nodes > kMaxNodes) return LineError(path, line_no, "node count out of range");
    if (expected_arcs >= 0xffffffffULL) return LineError(path, line_no, "too many arcs");
    graph.node_count = static_cast<std::uint32_t>(nodes);
    graph.arcs.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(expected_arcs, 1 << 26)));
  }
  while (true) {
    Result<bool> more = text.NextContentLine(&line);
    if (!more.ok()) return more.status();
    if (!*more) break;
    std::uint64_t line_no = text.line_number();
    if (layout == Layout::kDimacs) {
      if (line[0] != 'a') return LineError(path, line_no, "unexpected line");
      line.remove_prefix(1);
    }
    std::uint64_t u = 0;
    std::uint64_t v = 0;
    std::uint64_t w = 0;
    if (!ParseUnsigned(line, &u) || !ParseUnsigned(line, &v) || !ParseUnsigned(line, &w) ||
        !OnlySpaces(line)) {
      return LineError(path, line_no, "malformed arc line");
    }
    if (layout == Layout::kDimacs) {
      if (u == 0 || v == 0) return LineError(path, line_no, "arc endpoint out of range");
      --u;
      --v;
    }
    if (u >= graph.node_count || v >= graph.node_count) {
      return LineError(path, line_no, "arc endpoint out of range");
    }
    if (w > kMaxArcWeight) return LineError(path, line_no, "arc weight too large");
    if (graph.arcs.size() >= expected_arcs) {
      return LineError(path, line_no, "more arcs than the header declares");
    }
    graph.arcs.push_back({static_cast<NodeId>(u), static_cast<NodeId>(v), static_cast<Weight>(w)});
  }
  if (graph.arcs.size() != expected_arcs) {
    return Status::Error(path + ": expected " + std::to_string(expected_arcs) + " arcs, found " +
                         std::to_string(graph.arcs.size()));
  }
  return graph;
}

Result<std::vector<LatLon>> ReadRoadCoordinates(const std::string& path, std::uint32_t node_count) {
  Result<TextFile> file = TextFile::Open(path);
  if (!file.ok()) return file.status();
  TextFile& text = *file;
  Layout layout = Layout::kDimacs;
  std::string_view line;
  Result<bool> first = text.NextContentLine(&line);
  if (!first.ok()) return first.status();
  if (!*first) return Status::Error(path + ": the file has no header line");
  {
    std::uint64_t nodes = 0;
    std::uint64_t line_no = text.line_number();
    if (!line.empty() && line[0] == 'p') {
      line.remove_prefix(1);
      if (!ParseWord(line, "aux") || !ParseWord(line, "sp") || !ParseWord(line, "co") ||
          !ParseUnsigned(line, &nodes) || !OnlySpaces(line)) {
        return LineError(path, line_no, "malformed problem line");
      }
    } else {
      layout = Layout::kCompact;
      if (!ParseName(line, 32) || !ParseName(line, 32) || !ParseUnsigned(line, &nodes) ||
          !OnlySpaces(line)) {
        return LineError(path, line_no, "malformed header line");
      }
    }
    if (nodes != node_count) {
      return LineError(path, line_no,
                       "the coordinate file has " + std::to_string(nodes) +
                           " nodes but the graph has " + std::to_string(node_count));
    }
  }
  std::vector<LatLon> coords(node_count);
  std::vector<bool> seen(node_count, false);
  std::uint32_t seen_count = 0;
  while (true) {
    Result<bool> more = text.NextContentLine(&line);
    if (!more.ok()) return more.status();
    if (!*more) break;
    std::uint64_t line_no = text.line_number();
    if (layout == Layout::kDimacs) {
      if (line[0] != 'v') return LineError(path, line_no, "unexpected line");
      line.remove_prefix(1);
    }
    std::uint64_t id = 0;
    std::int64_t x = 0;
    std::int64_t y = 0;
    if (!ParseUnsigned(line, &id) || !ParseSigned(line, &x) || !ParseSigned(line, &y) ||
        !OnlySpaces(line)) {
      return LineError(path, line_no, "malformed node line");
    }
    if (layout == Layout::kDimacs) {
      if (id == 0) return LineError(path, line_no, "node id out of range");
      --id;
    }
    if (id >= node_count) return LineError(path, line_no, "node id out of range");
    if (x < -180000000 || x > 180000000 || y < -90000000 || y > 90000000) {
      return LineError(path, line_no, "coordinate out of range");
    }
    if (seen[id]) return LineError(path, line_no, "duplicate node");
    seen[id] = true;
    ++seen_count;
    coords[id] = {static_cast<std::int32_t>(y), static_cast<std::int32_t>(x)};
  }
  if (seen_count != node_count) {
    return Status::Error(path + ": " + std::to_string(node_count - seen_count) +
                         " nodes have no coordinates");
  }
  return coords;
}

}
