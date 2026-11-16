#include "graph.h"

#include <string>
#include <vector>

#include "dimacs.h"
#include "geo.h"
#include "io.h"
#include "support.h"
#include "testing.h"

namespace rr {
namespace {

Status WriteText(const std::string& path, const std::string& text) {
  Result<std::unique_ptr<FileWriter>> writer = FileWriter::Create(path);
  if (!writer.ok()) return writer.status();
  RR_RETURN_IF_ERROR((*writer)->Write(text));
  return (*writer)->Finish();
}

std::vector<LatLon> Line(std::uint32_t count) {
  std::vector<LatLon> coords;
  for (std::uint32_t i = 0; i < count; ++i) {
    coords.push_back(FromDegrees(39.1 + i * 0.001, -84.5));
  }
  return coords;
}

RR_TEST(ParallelArcsCollapseToTheCheapestAndLoopsGo) {
  std::vector<Arc> arcs = {{0, 1, 50}, {0, 1, 20}, {0, 1, 70}, {1, 1, 5}, {1, 0, 30}, {2, 0, 9}};
  Result<Graph> graph = Graph::FromArcs(3, arcs, Line(3));
  RR_CHECK_OK(graph);
  RR_CHECK_EQ(graph->arc_count(), 3u);
  RR_CHECK_EQ(graph->ArcWeight(0, 1), 20u);
  RR_CHECK_EQ(graph->ArcWeight(1, 0), 30u);
  RR_CHECK_EQ(graph->ArcWeight(2, 0), 9u);
  RR_CHECK_EQ(graph->ArcWeight(1, 1), kInf);
  RR_CHECK_EQ(graph->ArcWeight(0, 2), kInf);
  Adjacency view = graph->forward();
  RR_CHECK_EQ(view.first[0], 0u);
  RR_CHECK_EQ(view.first[3], 3u);
}

RR_TEST(BadInputIsRefused) {
  RR_CHECK(!Graph::FromArcs(0, {}, {}).ok());
  RR_CHECK(!Graph::FromArcs(2, {}, Line(3)).ok());
  RR_CHECK(!Graph::FromArcs(2, {{0, 2, 1}}, Line(2)).ok());
  RR_CHECK(!Graph::FromArcs(2, {{0, 1, kMaxArcWeight + 1}}, Line(2)).ok());
  std::vector<LatLon> bad = Line(2);
  bad[1] = {91000000, 0};
  RR_CHECK(!Graph::FromArcs(2, {{0, 1, 1}}, bad).ok());
}

RR_TEST(GraphFilesRoundTripAndCarryAnIdentity) {
  testing::TempDir dir;
  std::string path = dir.File("graph.rrg");
  Result<Graph> graph = Graph::FromArcs(4, {{0, 1, 5}, {1, 2, 6}, {2, 3, 7}, {3, 0, 8}}, Line(4));
  RR_CHECK_OK(graph);
  RR_CHECK_STATUS(graph->Save(path));
  Result<Graph> loaded = Graph::Load(path, true);
  RR_CHECK_OK(loaded);
  RR_CHECK_EQ(loaded->node_count(), graph->node_count());
  RR_CHECK_EQ(loaded->arc_count(), graph->arc_count());
  RR_CHECK_EQ(loaded->identity(), graph->identity());
  RR_CHECK_EQ(loaded->ArcWeight(2, 3), 7u);
  RR_CHECK_EQ(loaded->coord(1).lat, graph->coord(1).lat);
  Result<Graph> other = Graph::FromArcs(4, {{0, 1, 5}, {1, 2, 6}, {2, 3, 70}, {3, 0, 8}}, Line(4));
  RR_CHECK_OK(other);
  RR_CHECK(other->identity() != graph->identity());
}

RR_TEST(TheLargestStrongComponentIsKept) {
  std::vector<Arc> arcs = {{0, 1, 1}, {1, 2, 1}, {2, 0, 1}, {2, 3, 1}, {3, 4, 1}, {4, 3, 1}};
  Result<Graph> graph = Graph::FromArcs(6, arcs, Line(6));
  RR_CHECK_OK(graph);
  ComponentInfo info;
  Result<Graph> strong = LargestStrongComponent(*graph, &info);
  RR_CHECK_OK(strong);
  RR_CHECK_EQ(info.components, 3u);
  RR_CHECK_EQ(info.kept_nodes, 3u);
  RR_CHECK_EQ(strong->node_count(), 3u);
  RR_CHECK_EQ(strong->arc_count(), 3u);
}

RR_TEST(RenumberingMustBeABijection) {
  Result<Graph> graph = Graph::FromArcs(3, {{0, 1, 1}, {1, 2, 1}}, Line(3));
  RR_CHECK_OK(graph);
  std::vector<NodeId> twice = {0, 0, 1};
  RR_CHECK(!Renumber(*graph, twice).ok());
  std::vector<NodeId> out_of_range = {0, 1, 7};
  RR_CHECK(!Renumber(*graph, out_of_range).ok());
  std::vector<NodeId> wrong_size = {0, 1};
  RR_CHECK(!Renumber(*graph, wrong_size).ok());
  std::vector<NodeId> reversed = {2, 1, 0};
  Result<Graph> flipped = Renumber(*graph, reversed);
  RR_CHECK_OK(flipped);
  RR_CHECK_EQ(flipped->ArcWeight(2, 1), 1u);
  RR_CHECK_EQ(flipped->ArcWeight(1, 0), 1u);
}

RR_TEST(ReverseGraphMirrorsEveryArc) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(3000, 5, dir);
  RR_CHECK_OK(graph);
  ReverseGraph reverse(*graph);
  Adjacency fwd = graph->forward();
  Adjacency bwd = reverse.view();
  std::uint32_t seen = 0;
  for (NodeId v = 0; v < graph->node_count(); ++v) {
    for (std::uint32_t a = bwd.first[v]; a < bwd.first[v + 1]; ++a) {
      RR_CHECK_EQ(graph->ArcWeight(bwd.other[a], v), bwd.weight[a]);
      ++seen;
    }
  }
  RR_CHECK_EQ(seen, graph->arc_count());
  RR_CHECK_EQ(bwd.first[graph->node_count()], fwd.first[graph->node_count()]);
}

RR_TEST(HilbertNumberingIsAPermutation) {
  testing::TempDir dir;
  Result<Graph> graph = testing::RandomRoadGraph(2000, 11, dir);
  RR_CHECK_OK(graph);
  std::vector<NodeId> numbering = HilbertNumbering(*graph);
  std::vector<char> seen(numbering.size(), 0);
  for (NodeId id : numbering) {
    RR_CHECK(id < numbering.size());
    RR_CHECK_EQ(seen[id], char{0});
    seen[id] = 1;
  }
  Result<Graph> ordered = Renumber(*graph, numbering);
  RR_CHECK_OK(ordered);
  RR_CHECK_EQ(ordered->arc_count(), graph->arc_count());
}

RR_TEST(TheDimacsLayoutIsParsed) {
  testing::TempDir dir;
  std::string graph_path = dir.File("small.gr");
  std::string coord_path = dir.File("small.co");
  RR_CHECK_STATUS(WriteText(graph_path, "c a comment\np sp 3 3\na 1 2 10\na 2 3 20\na 3 1 30\n"));
  RR_CHECK_STATUS(WriteText(coord_path,
                            "p aux sp co 3\nv 1 -84512000 39103100\nv 2 -84500000 39110000\n"
                            "v 3 -84490000 39120000\n"));
  Result<RoadArcs> graph = ReadRoadGraph(graph_path);
  RR_CHECK_OK(graph);
  RR_CHECK_EQ(graph->node_count, 3u);
  RR_CHECK_EQ(graph->arcs.size(), std::size_t{3});
  RR_CHECK_EQ(graph->arcs[0].tail, 0u);
  RR_CHECK_EQ(graph->arcs[0].head, 1u);
  RR_CHECK_EQ(graph->arcs[0].weight, 10u);
  Result<std::vector<LatLon>> coords = ReadRoadCoordinates(coord_path, 3);
  RR_CHECK_OK(coords);
  RR_CHECK_EQ((*coords)[0].lat, 39103100);
  RR_CHECK_EQ((*coords)[0].lon, -84512000);
}

RR_TEST(BrokenDimacsFilesAreRefused) {
  testing::TempDir dir;
  std::string path = dir.File("broken.gr");
  const char* bad_graphs[] = {
      "a 1 2 3\n",           "p sp 2 1\np sp 2 1\n",
      "p sp 2 2\na 1 2 3\n", "p sp 2 1\na 1 2 3\na 2 1 4\n",
      "p sp 2 1\na 1 3 5\n", "p sp 2 1\na 0 1 5\n",
      "p sp 2 1\na 1 2 x\n", "p sp 2 1\na 1 2 5 6\n",
      "p sp 0 0\n",          "x 1\n",
      "c only a comment\n",  "p sp 2 1\na 1 2 2000000000\n",
  };
  for (const char* text : bad_graphs) {
    RR_CHECK_STATUS(WriteText(path, text));
    Result<RoadArcs> graph = ReadRoadGraph(path);
    if (graph.ok()) {
      testing::Report(__FILE__, __LINE__, std::string("accepted: ") + text);
    }
  }
  std::string coord_path = dir.File("broken.co");
  const char* bad_coords[] = {
      "v 1 0 0\n",
      "p aux sp co 2\nv 1 0 0\n",
      "p aux sp co 3\nv 1 0 0\nv 2 0 0\n",
      "p aux sp co 2\nv 1 0 0\nv 1 0 0\n",
      "p aux sp co 2\nv 1 0 0\nv 3 0 0\n",
      "p aux sp co 2\nv 1 0 0\nv 2 200000000 0\n",
      "p aux sp co 2\nv 1 0 0\nv 2 0 91000000\n",
      "p aux sp co 2\nv 1 0\nv 2 0 0\n",
  };
  for (const char* text : bad_coords) {
    RR_CHECK_STATUS(WriteText(coord_path, text));
    Result<std::vector<LatLon>> coords = ReadRoadCoordinates(coord_path, 2);
    if (coords.ok()) {
      testing::Report(__FILE__, __LINE__, std::string("accepted: ") + text);
    }
  }
}

RR_TEST(TheCompactLayoutIsParsed) {
  testing::TempDir dir;
  std::string graph_path = dir.File("compact.graph");
  std::string coord_path = dir.File("compact.coordinates");
  RR_CHECK_STATUS(WriteText(graph_path, "NW t 3 3\n0 1 10\n1 2 20\n2 0 30\n"));
  RR_CHECK_STATUS(WriteText(coord_path,
                            "NW t 3\n0 -84512000 39103100\n1 -84500000 39110000\n"
                            "2 -84490000 39120000\n"));
  Result<RoadArcs> graph = ReadRoadGraph(graph_path);
  RR_CHECK_OK(graph);
  RR_CHECK_EQ(graph->node_count, 3u);
  RR_CHECK_EQ(graph->arcs.size(), std::size_t{3});
  RR_CHECK_EQ(graph->arcs[0].tail, 0u);
  RR_CHECK_EQ(graph->arcs[0].head, 1u);
  RR_CHECK_EQ(graph->arcs[0].weight, 10u);
  RR_CHECK_EQ(graph->arcs[2].tail, 2u);
  RR_CHECK_EQ(graph->arcs[2].head, 0u);
  Result<std::vector<LatLon>> coords = ReadRoadCoordinates(coord_path, 3);
  RR_CHECK_OK(coords);
  RR_CHECK_EQ((*coords)[0].lat, 39103100);
  RR_CHECK_EQ((*coords)[2].lon, -84490000);
  Result<Graph> built =
      Graph::FromArcs(graph->node_count, std::move(graph->arcs), std::move(*coords));
  RR_CHECK_OK(built);
  RR_CHECK_EQ(built->ArcWeight(1, 2), 20u);
}

RR_TEST(BrokenCompactFilesAreRefused) {
  testing::TempDir dir;
  std::string path = dir.File("compact.graph");
  const char* bad_graphs[] = {
      "NW t 3\n",
      "NW t 3 2\n0 1 10\n",
      "NW t 3 1\n0 1 10\n1 2 20\n",
      "NW t 3 1\n0 3 10\n",
      "NW t 3 1\na 0 1 10\n",
      "NW t 0 0\n",
      "NW t 3 1\n0 1 2000000000\n",
      "NW t 3 1\n0 1\n",
      "NW t 3 1\n0 1 10 11\n",
  };
  for (const char* text : bad_graphs) {
    RR_CHECK_STATUS(WriteText(path, text));
    if (ReadRoadGraph(path).ok()) {
      testing::Report(__FILE__, __LINE__, std::string("accepted: ") + text);
    }
  }
  std::string coord_path = dir.File("compact.coordinates");
  const char* bad_coords[] = {
      "NW t 2\n0 0 0\n",        "NW t 2\n0 0 0\n0 0 0\n",        "NW t 2\n0 0 0\n2 0 0\n",
      "NW t 3\n0 0 0\n1 0 0\n", "NW t 2\n0 0 0\n1 0 91000000\n",
  };
  for (const char* text : bad_coords) {
    RR_CHECK_STATUS(WriteText(coord_path, text));
    if (ReadRoadCoordinates(coord_path, 2).ok()) {
      testing::Report(__FILE__, __LINE__, std::string("accepted: ") + text);
    }
  }
}

RR_TEST(NegativeCoordinatesAndSignsAreAccepted) {
  testing::TempDir dir;
  std::string path = dir.File("signs.co");
  RR_CHECK_STATUS(WriteText(path, "p aux sp co 2\nv 1 -1000 +2000\nv 2 3000 -4000\n"));
  Result<std::vector<LatLon>> coords = ReadRoadCoordinates(path, 2);
  RR_CHECK_OK(coords);
  RR_CHECK_EQ((*coords)[0].lon, -1000);
  RR_CHECK_EQ((*coords)[0].lat, 2000);
  RR_CHECK_EQ((*coords)[1].lon, 3000);
  RR_CHECK_EQ((*coords)[1].lat, -4000);
}

}
}

int main() { return rr::testing::RunAll(); }
