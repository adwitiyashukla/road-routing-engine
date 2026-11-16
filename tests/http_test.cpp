#include "http.h"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "contract.h"
#include "graph.h"
#include "service.h"
#include "support.h"
#include "testing.h"

namespace rr {
namespace {

bool Contains(const std::string& text, const std::string& piece) {
  return text.find(piece) != std::string::npos;
}

RR_TEST(RequestLinesAreParsed) {
  HttpRequest request;
  RR_CHECK(ParseHttpRequest("GET /route?from=1,2&to=3,4 HTTP/1.1\r\nHost: x\r\n", &request));
  RR_CHECK_EQ(request.method, std::string("GET"));
  RR_CHECK_EQ(request.path, std::string("/route"));
  RR_CHECK_EQ(request.query.size(), std::size_t{2});
  RR_CHECK(request.Param("from") != nullptr);
  RR_CHECK_EQ(*request.Param("from"), std::string("1,2"));
  RR_CHECK_EQ(*request.Param("to"), std::string("3,4"));
  RR_CHECK(request.Param("missing") == nullptr);
  RR_CHECK(request.keep_alive);

  RR_CHECK(ParseHttpRequest("GET / HTTP/1.0\r\n", &request));
  RR_CHECK(!request.keep_alive);
  RR_CHECK(ParseHttpRequest("GET / HTTP/1.1\r\nConnection: close\r\n", &request));
  RR_CHECK(!request.keep_alive);
  RR_CHECK(ParseHttpRequest("POST /x HTTP/1.1\r\nContent-Length: 12\r\n", &request));
  RR_CHECK_EQ(request.content_length, std::uint64_t{12});

  const char* bad[] = {
      "GET /x HTTP/2.0\r\n",
      "GET /x\r\n",
      "get /x HTTP/1.1\r\n",
      "GET x HTTP/1.1\r\n",
      " /x HTTP/1.1\r\n",
      "GET /x HTTP/1.1\r\nbroken header\r\n",
      "GET /x HTTP/1.1\r\n: empty\r\n",
      "GET /x HTTP/1.1\r\nContent-Length: abc\r\n",
      "GET /x HTTP/1.1\r\nTransfer-Encoding: chunked\r\n",
      "GET /%zz HTTP/1.1\r\n",
      "GET /%4 HTTP/1.1\r\n",
  };
  for (const char* text : bad) {
    if (ParseHttpRequest(text, &request)) {
      testing::Report(__FILE__, __LINE__, std::string("accepted: ") + text);
    }
  }
}

RR_TEST(QueryValuesAreDecoded) {
  HttpRequest request;
  RR_CHECK(
      ParseHttpRequest("GET /a?name=two+words&town=K%C3%B6ln&bare&eq= HTTP/1.1\r\n", &request));
  RR_CHECK_EQ(*request.Param("name"), std::string("two words"));
  RR_CHECK_EQ(*request.Param("town"), std::string("K\xc3\xb6ln"));
  RR_CHECK_EQ(*request.Param("bare"), std::string(""));
  RR_CHECK_EQ(*request.Param("eq"), std::string(""));
  std::string out;
  RR_CHECK(PercentDecode("a%20b", false, &out));
  RR_CHECK_EQ(out, std::string("a b"));
  RR_CHECK(PercentDecode("a+b", false, &out));
  RR_CHECK_EQ(out, std::string("a+b"));
  RR_CHECK(!PercentDecode("a%2", false, &out));
  RR_CHECK(!PercentDecode("%", false, &out));
  RR_CHECK(PercentDecode("", false, &out));
  RR_CHECK_EQ(out, std::string(""));
}

RR_TEST(JsonStringsAreEscaped) {
  RR_CHECK_EQ(JsonEscape("plain"), std::string("plain"));
  RR_CHECK_EQ(JsonEscape("a\"b\\c"), std::string("a\\\"b\\\\c"));
  RR_CHECK_EQ(JsonEscape("line\ntab\t"), std::string("line\\ntab\\t"));
  RR_CHECK_EQ(JsonEscape(std::string("\x01", 1)), std::string("\\u0001"));
  RR_CHECK_EQ(JsonEscape("K\xc3\xb6ln"), std::string("K\\u00c3\\u00b6ln"));
}

struct Fixture {
  testing::TempDir dir;
  Result<Graph> graph = Status::Error("not built");
  Result<ChGraph> ch = Status::Error("not built");
  std::unique_ptr<Engine> engine;

  bool Build() {
    graph = testing::RandomRoadGraph(2000, 23, dir);
    if (!graph.ok()) return false;
    std::string graph_path = dir.File("graph.rrg");
    if (!graph->Save(graph_path).ok()) return false;
    ContractOptions options;
    options.threads = 2;
    ch = Contract(*graph, options, nullptr);
    if (!ch.ok()) return false;
    std::string ch_path = dir.File("graph.rrch");
    if (!ch->Save(ch_path, *graph).ok()) return false;
    EngineFiles files;
    files.graph = graph_path;
    files.hierarchy = ch_path;
    files.verify = true;
    Result<std::unique_ptr<Engine>> opened = Engine::Open(files);
    if (!opened.ok()) return false;
    engine = std::move(*opened);
    return true;
  }
};

RR_TEST(TheServiceAnswersEveryEndpoint) {
  Fixture fixture;
  RR_CHECK(fixture.Build());
  Service service(*fixture.engine, 2);
  HttpRequest request;
  request.method = "GET";

  request.path = "/health";
  HttpResponse health = service.Handle(request);
  RR_CHECK_EQ(health.status, 200);
  RR_CHECK(Contains(health.body, "\"status\":\"ok\""));
  RR_CHECK(Contains(health.body, "\"hierarchy\":true"));
  RR_CHECK(Contains(health.body, "\"algorithm\":\"ch\""));

  request.path = "/route";
  request.query = {{"from_node", "0"}, {"to_node", "500"}};
  HttpResponse route = service.Handle(request);
  RR_CHECK_EQ(route.status, 200);
  RR_CHECK(Contains(route.body, "\"reachable\":true"));
  RR_CHECK(Contains(route.body, "\"path\":["));
  RR_CHECK(Contains(route.body, "\"distance_meters\""));

  request.query = {{"from_node", "0"}, {"to_node", "500"}, {"path", "polyline"}};
  HttpResponse polyline = service.Handle(request);
  RR_CHECK_EQ(polyline.status, 200);
  RR_CHECK(Contains(polyline.body, "\"polyline\":\""));

  request.query = {{"from_node", "0"}, {"to_node", "500"}, {"path", "none"}};
  HttpResponse bare = service.Handle(request);
  RR_CHECK(!Contains(bare.body, "\"path\""));

  LatLon point = fixture.graph->coord(700);
  std::string text = std::to_string(point.lat / 1e6) + "," + std::to_string(point.lon / 1e6);
  request.query = {{"from", text}, {"to", text}};
  HttpResponse same = service.Handle(request);
  RR_CHECK_EQ(same.status, 200);
  RR_CHECK(Contains(same.body, "\"weight\":0"));

  request.path = "/nearest";
  request.query = {{"point", text}};
  HttpResponse nearest = service.Handle(request);
  RR_CHECK_EQ(nearest.status, 200);
  RR_CHECK(Contains(nearest.body, "\"node\":700"));

  request.path = "/table";
  request.query = {{"source_nodes", "1;2;3"}, {"target_nodes", "4;5"}};
  HttpResponse table = service.Handle(request);
  RR_CHECK_EQ(table.status, 200);
  RR_CHECK(Contains(table.body, "\"sources\":3"));
  RR_CHECK(Contains(table.body, "\"targets\":2"));

  request.path = "/missing";
  request.query = {};
  RR_CHECK_EQ(service.Handle(request).status, 404);

  request.path = "/route";
  const std::vector<std::vector<std::pair<std::string, std::string>>> bad = {
      {},
      {{"from_node", "0"}},
      {{"from_node", "0"}, {"to_node", "999999"}},
      {{"from_node", "0"}, {"to_node", "x"}},
      {{"from_node", "0"}, {"to_node", "1"}, {"algorithm", "magic"}},
      {{"from_node", "0"}, {"to_node", "1"}, {"path", "maybe"}},
      {{"from_node", "0"}, {"to_node", "1"}, {"algorithm", "alt"}},
      {{"from", "200,0"}, {"to", "1,1"}},
      {{"from", "nonsense"}, {"to", "1,1"}},
  };
  for (const auto& query : bad) {
    request.query = query;
    HttpResponse response = service.Handle(request);
    if (response.status != 400) {
      testing::Report(__FILE__, __LINE__,
                      "expected 400 but got " + std::to_string(response.status));
    }
  }
}

RR_TEST(EveryServedAlgorithmAgreesOverHttp) {
  Fixture fixture;
  RR_CHECK(fixture.Build());
  Service service(*fixture.engine, 1);
  HttpServer server;
  RR_CHECK_STATUS(server.Start("127.0.0.1", 0, 2, [&service](const HttpRequest& request) {
    return service.Handle(request);
  }));
  HttpClient client;
  RR_CHECK_STATUS(client.Connect("127.0.0.1", server.port()));
  Result<HttpResponse> health = client.Get("/health");
  RR_CHECK_OK(health);
  RR_CHECK_EQ(health->status, 200);
  RR_CHECK_EQ(health->content_type, std::string("application/json"));

  const char* algorithms[] = {"dijkstra", "bidirectional", "astar", "bidirectional-astar", "ch"};
  Rng rng(31);
  for (int i = 0; i < 20; ++i) {
    NodeId source = rng.Below(fixture.graph->node_count());
    NodeId target = rng.Below(fixture.graph->node_count());
    std::string first;
    for (const char* name : algorithms) {
      std::string target_url = "/route?from_node=" + std::to_string(source) +
                               "&to_node=" + std::to_string(target) + "&algorithm=" + name +
                               "&path=none";
      Result<HttpResponse> response = client.Get(target_url);
      RR_CHECK_OK(response);
      RR_CHECK_EQ(response->status, 200);
      std::size_t at = response->body.find("\"weight\":");
      RR_CHECK(at != std::string::npos);
      std::string weight = response->body.substr(at, response->body.find(',', at) - at);
      if (first.empty()) {
        first = weight;
      } else if (weight != first) {
        testing::Report(__FILE__, __LINE__,
                        std::string(name) + " returned " + weight + " instead of " + first);
        return;
      }
    }
  }
  Result<HttpResponse> missing = client.Get("/nope");
  RR_CHECK_OK(missing);
  RR_CHECK_EQ(missing->status, 404);
  server.Stop();
}

RR_TEST(ManyClientsCanAskAtOnce) {
  Fixture fixture;
  RR_CHECK(fixture.Build());
  Service service(*fixture.engine, 4);
  HttpServer server;
  RR_CHECK_STATUS(server.Start("127.0.0.1", 0, 4, [&service](const HttpRequest& request) {
    return service.Handle(request);
  }));
  std::atomic<int> failures{0};
  std::atomic<int> answered{0};
  std::vector<std::thread> clients;
  for (int c = 0; c < 4; ++c) {
    clients.emplace_back([&, c] {
      HttpClient client;
      if (!client.Connect("127.0.0.1", server.port()).ok()) {
        ++failures;
        return;
      }
      Rng rng(100 + c);
      for (int i = 0; i < 25; ++i) {
        NodeId source = rng.Below(fixture.graph->node_count());
        NodeId target = rng.Below(fixture.graph->node_count());
        Result<HttpResponse> response =
            client.Get("/route?from_node=" + std::to_string(source) +
                       "&to_node=" + std::to_string(target) + "&path=nodes");
        if (!response.ok() || response->status != 200) {
          ++failures;
          return;
        }
        ++answered;
      }
    });
  }
  for (std::thread& client : clients) client.join();
  RR_CHECK_EQ(failures.load(), 0);
  RR_CHECK_EQ(answered.load(), 100);
  server.Stop();
}

RR_TEST(TheServerRefusesWhatItCannotServe) {
  Fixture fixture;
  RR_CHECK(fixture.Build());
  Service service(*fixture.engine, 1);
  HttpServer server;
  RR_CHECK_STATUS(server.Start("127.0.0.1", 0, 1, [&service](const HttpRequest& request) {
    return service.Handle(request);
  }));
  HttpClient client;
  RR_CHECK_STATUS(client.Connect("127.0.0.1", server.port()));
  Result<HttpResponse> bad = client.Get("/route?from_node=0&to_node=abc");
  RR_CHECK_OK(bad);
  RR_CHECK_EQ(bad->status, 400);
  RR_CHECK(Contains(bad->body, "\"error\""));
  Result<HttpResponse> after = client.Get("/health");
  RR_CHECK_OK(after);
  RR_CHECK_EQ(after->status, 200);
  server.Stop();
}

}
}

int main() { return rr::testing::RunAll(); }
