#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base.h"

namespace rr {

struct HttpRequest {
  std::string method;
  std::string path;
  std::vector<std::pair<std::string, std::string>> query;
  bool keep_alive = true;
  std::uint64_t content_length = 0;

  const std::string* Param(std::string_view name) const;
};

struct HttpResponse {
  int status = 200;
  std::string content_type = "application/json";
  std::string body;
};

bool ParseHttpRequest(std::string_view head, HttpRequest* request);
bool PercentDecode(std::string_view text, bool plus_is_space, std::string* out);
std::string JsonEscape(std::string_view text);
const char* StatusText(int status);

class HttpServer {
 public:
  using Handler = std::function<HttpResponse(const HttpRequest&)>;

  HttpServer() = default;
  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;
  ~HttpServer();

  Status Start(const std::string& host, std::uint16_t port, int threads, Handler handler);
  void Stop();
  void Wait();
  std::uint16_t port() const { return port_; }

 private:
  void AcceptLoop();
  void WorkerLoop();
  void Serve(std::intptr_t socket);

  Handler handler_;
  std::string host_;
  std::intptr_t listener_ = -1;
  std::uint16_t port_ = 0;
  std::atomic<bool> stopping_{false};
  std::thread acceptor_;
  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::intptr_t> queue_;
  std::unordered_set<std::intptr_t> active_;
  bool started_ = false;
};

class HttpClient {
 public:
  HttpClient() = default;
  HttpClient(const HttpClient&) = delete;
  HttpClient& operator=(const HttpClient&) = delete;
  ~HttpClient() { Close(); }

  Status Connect(const std::string& host, std::uint16_t port);
  Result<HttpResponse> Get(const std::string& target);
  void Close();

 private:
  std::intptr_t socket_ = -1;
  std::string host_;
  std::string buffer_;
};

}
