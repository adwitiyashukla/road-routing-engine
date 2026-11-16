#include "http.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <exception>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace rr {

namespace {

#if defined(_WIN32)
using SocketHandle = SOCKET;
const SocketHandle kBadSocket = INVALID_SOCKET;
#else
using SocketHandle = int;
const SocketHandle kBadSocket = -1;
#endif

constexpr std::size_t kMaxHead = 16 * 1024;
constexpr std::size_t kMaxQueue = 4096;
constexpr int kReceiveTimeoutMs = 15000;

SocketHandle ToHandle(std::intptr_t s) { return static_cast<SocketHandle>(s); }
std::intptr_t FromHandle(SocketHandle s) { return static_cast<std::intptr_t>(s); }
bool Valid(std::intptr_t s) { return ToHandle(s) != kBadSocket && s != -1; }

bool EnsureNetwork() {
#if defined(_WIN32)
  static const bool ok = [] {
    WSADATA data;
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }();
  return ok;
#else
  return true;
#endif
}

void CloseSocket(std::intptr_t s) {
  if (!Valid(s)) return;
#if defined(_WIN32)
  closesocket(ToHandle(s));
#else
  ::close(ToHandle(s));
#endif
}

void ShutdownSocket(std::intptr_t s) {
#if defined(_WIN32)
  shutdown(ToHandle(s), SD_BOTH);
#else
  shutdown(ToHandle(s), SHUT_RDWR);
#endif
}

void ConfigureSocket(std::intptr_t s) {
  int one = 1;
  setsockopt(ToHandle(s), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one),
             sizeof(one));
#if defined(_WIN32)
  DWORD timeout = kReceiveTimeoutMs;
  setsockopt(ToHandle(s), SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout),
             sizeof(timeout));
#else
  timeval timeout{kReceiveTimeoutMs / 1000, (kReceiveTimeoutMs % 1000) * 1000};
  setsockopt(ToHandle(s), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif
#if defined(__APPLE__)
  setsockopt(ToHandle(s), SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
}

bool SendAll(std::intptr_t s, std::string_view data) {
  while (!data.empty()) {
    int chunk = static_cast<int>(std::min<std::size_t>(data.size(), 1 << 20));
#if defined(_WIN32)
    int sent = send(ToHandle(s), data.data(), chunk, 0);
#elif defined(__APPLE__)
    auto sent = send(ToHandle(s), data.data(), static_cast<std::size_t>(chunk), 0);
#else
    auto sent = send(ToHandle(s), data.data(), static_cast<std::size_t>(chunk), MSG_NOSIGNAL);
#endif
    if (sent <= 0) return false;
    data.remove_prefix(static_cast<std::size_t>(sent));
  }
  return true;
}

long long Receive(std::intptr_t s, char* buffer, std::size_t size) {
#if defined(_WIN32)
  return recv(ToHandle(s), buffer, static_cast<int>(size), 0);
#else
  return static_cast<long long>(recv(ToHandle(s), buffer, size, 0));
#endif
}

bool ReadHead(std::intptr_t s, std::string& buffer, std::string* head) {
  std::size_t scanned = 0;
  while (true) {
    std::size_t end = buffer.find("\r\n\r\n", scanned);
    if (end != std::string::npos) {
      head->assign(buffer, 0, end);
      buffer.erase(0, end + 4);
      return true;
    }
    scanned = buffer.size() >= 3 ? buffer.size() - 3 : 0;
    if (buffer.size() > kMaxHead) return false;
    char chunk[4096];
    long long got = Receive(s, chunk, sizeof(chunk));
    if (got <= 0) return false;
    buffer.append(chunk, static_cast<std::size_t>(got));
  }
}

std::string Lower(std::string_view text) {
  std::string out(text);
  for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

std::string_view Trim(std::string_view text) {
  while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.remove_prefix(1);
  while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) text.remove_suffix(1);
  return text;
}

bool ParseSize(std::string_view text, std::uint64_t* out) {
  if (text.empty() || text.size() > 18) return false;
  std::uint64_t value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') return false;
    value = value * 10 + static_cast<std::uint64_t>(c - '0');
  }
  *out = value;
  return true;
}

int HexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

std::string FormatResponse(const HttpResponse& response, bool keep_alive) {
  std::string out = "HTTP/1.1 " + std::to_string(response.status) + " " +
                    StatusText(response.status) + "\r\nContent-Type: " + response.content_type +
                    "\r\nContent-Length: " + std::to_string(response.body.size()) +
                    "\r\nConnection: " + (keep_alive ? "keep-alive" : "close") + "\r\n\r\n";
  out += response.body;
  return out;
}

HttpResponse ErrorResponse(int status, const std::string& message) {
  HttpResponse response;
  response.status = status;
  response.body = "{\"error\":\"" + JsonEscape(message) + "\"}";
  return response;
}

}

const std::string* HttpRequest::Param(std::string_view name) const {
  for (const auto& [key, value] : query) {
    if (key == name) return &value;
  }
  return nullptr;
}

const char* StatusText(int status) {
  switch (status) {
    case 200:
      return "OK";
    case 400:
      return "Bad Request";
    case 404:
      return "Not Found";
    case 405:
      return "Method Not Allowed";
    case 413:
      return "Payload Too Large";
    case 431:
      return "Request Header Fields Too Large";
    case 503:
      return "Service Unavailable";
    default:
      return "Internal Server Error";
  }
}

bool PercentDecode(std::string_view text, bool plus_is_space, std::string* out) {
  out->clear();
  out->reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ++i) {
    char c = text[i];
    if (c == '%') {
      if (i + 2 >= text.size()) return false;
      int hi = HexValue(text[i + 1]);
      int lo = HexValue(text[i + 2]);
      if (hi < 0 || lo < 0) return false;
      out->push_back(static_cast<char>(hi * 16 + lo));
      i += 2;
    } else if (c == '+' && plus_is_space) {
      out->push_back(' ');
    } else {
      out->push_back(c);
    }
  }
  return true;
}

std::string JsonEscape(std::string_view text) {
  std::string out;
  out.reserve(text.size());
  for (char c : text) {
    unsigned char u = static_cast<unsigned char>(c);
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (u < 0x20 || u >= 0x7f) {
          static const char* digits = "0123456789abcdef";
          out += "\\u00";
          out.push_back(digits[u >> 4]);
          out.push_back(digits[u & 15]);
        } else {
          out.push_back(c);
        }
    }
  }
  return out;
}

bool ParseHttpRequest(std::string_view head, HttpRequest* request) {
  *request = HttpRequest{};
  std::size_t line_end = head.find("\r\n");
  std::string_view line = head.substr(0, line_end);
  std::size_t first_space = line.find(' ');
  if (first_space == std::string_view::npos || first_space == 0) return false;
  std::size_t second_space = line.find(' ', first_space + 1);
  if (second_space == std::string_view::npos) return false;
  std::string_view method = line.substr(0, first_space);
  std::string_view target = line.substr(first_space + 1, second_space - first_space - 1);
  std::string_view version = line.substr(second_space + 1);
  for (char c : method) {
    if (c < 'A' || c > 'Z') return false;
  }
  if (version != "HTTP/1.1" && version != "HTTP/1.0") return false;
  if (target.empty() || target[0] != '/') return false;
  for (char c : target) {
    if (static_cast<unsigned char>(c) <= 0x20 || static_cast<unsigned char>(c) >= 0x7f) {
      return false;
    }
  }
  request->method = std::string(method);
  request->keep_alive = version == "HTTP/1.1";
  std::size_t question = target.find('?');
  if (!PercentDecode(target.substr(0, question), false, &request->path)) return false;
  if (question != std::string_view::npos) {
    std::string_view query = target.substr(question + 1);
    while (!query.empty()) {
      std::size_t amp = query.find('&');
      std::string_view pair = query.substr(0, amp);
      query = amp == std::string_view::npos ? std::string_view() : query.substr(amp + 1);
      if (pair.empty()) continue;
      std::size_t eq = pair.find('=');
      std::string key;
      std::string value;
      if (!PercentDecode(pair.substr(0, eq), true, &key)) return false;
      if (eq != std::string_view::npos && !PercentDecode(pair.substr(eq + 1), true, &value)) {
        return false;
      }
      request->query.emplace_back(std::move(key), std::move(value));
    }
  }
  std::string_view rest =
      line_end == std::string_view::npos ? std::string_view() : head.substr(line_end + 2);
  while (!rest.empty()) {
    std::size_t end = rest.find("\r\n");
    std::string_view header = rest.substr(0, end);
    rest = end == std::string_view::npos ? std::string_view() : rest.substr(end + 2);
    std::size_t colon = header.find(':');
    if (colon == std::string_view::npos || colon == 0) return false;
    std::string name = Lower(header.substr(0, colon));
    std::string_view value = Trim(header.substr(colon + 1));
    if (name == "connection") {
      std::string v = Lower(value);
      if (v == "close") request->keep_alive = false;
      if (v == "keep-alive") request->keep_alive = true;
    } else if (name == "content-length") {
      if (!ParseSize(value, &request->content_length)) return false;
    } else if (name == "transfer-encoding") {
      return false;
    }
  }
  return true;
}

HttpServer::~HttpServer() { Stop(); }

Status HttpServer::Start(const std::string& host, std::uint16_t port, int threads,
                         Handler handler) {
  if (started_) return Status::Error("server already started");
  if (!EnsureNetwork()) return Status::Error("cannot initialize networking");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
    return Status::Error("invalid listen address " + host);
  }
  SocketHandle s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == kBadSocket) return Status::Error("cannot create socket");
#if !defined(_WIN32)
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#endif
  if (bind(s, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 || listen(s, 512) != 0) {
    CloseSocket(FromHandle(s));
    return Status::Error("cannot listen on " + host + ":" + std::to_string(port));
  }
  sockaddr_in bound{};
  socklen_t length = sizeof(bound);
  getsockname(s, reinterpret_cast<sockaddr*>(&bound), &length);
  port_ = ntohs(bound.sin_port);
  host_ = host == "0.0.0.0" ? "127.0.0.1" : host;
  listener_ = FromHandle(s);
  handler_ = std::move(handler);
  stopping_ = false;
  started_ = true;
  for (int i = 0; i < std::max(threads, 1); ++i) workers_.emplace_back([this] { WorkerLoop(); });
  acceptor_ = std::thread([this] { AcceptLoop(); });
  return Status();
}

void HttpServer::Stop() {
  if (!started_) return;
  if (!stopping_.exchange(true)) {
    HttpClient waker;
    waker.Connect(host_, port_);
    waker.Close();
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::intptr_t s : active_) ShutdownSocket(s);
  }
  ready_.notify_all();
  if (acceptor_.joinable()) acceptor_.join();
  for (std::thread& worker : workers_) {
    if (worker.joinable()) worker.join();
  }
  workers_.clear();
  CloseSocket(listener_);
  listener_ = -1;
  for (std::intptr_t s : queue_) CloseSocket(s);
  queue_.clear();
  started_ = false;
}

void HttpServer::Wait() {
  if (acceptor_.joinable()) acceptor_.join();
}

void HttpServer::AcceptLoop() {
  while (!stopping_) {
    SocketHandle client = accept(ToHandle(listener_), nullptr, nullptr);
    if (client == kBadSocket) {
      if (stopping_) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    std::intptr_t handle = FromHandle(client);
    if (stopping_) {
      CloseSocket(handle);
      break;
    }
    ConfigureSocket(handle);
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.size() >= kMaxQueue) {
      lock.unlock();
      SendAll(handle, FormatResponse(ErrorResponse(503, "server is overloaded"), false));
      CloseSocket(handle);
      continue;
    }
    queue_.push_back(handle);
    lock.unlock();
    ready_.notify_one();
  }
  ready_.notify_all();
}

void HttpServer::WorkerLoop() {
  while (true) {
    std::intptr_t s = -1;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (stopping_) return;
      s = queue_.front();
      queue_.pop_front();
      active_.insert(s);
    }
    Serve(s);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_.erase(s);
    }
    CloseSocket(s);
  }
}

void HttpServer::Serve(std::intptr_t s) {
  std::string buffer;
  std::string head;
  while (!stopping_) {
    if (!ReadHead(s, buffer, &head)) {
      if (buffer.size() > kMaxHead) {
        SendAll(s, FormatResponse(ErrorResponse(431, "request head too large"), false));
      }
      return;
    }
    HttpRequest request;
    HttpResponse response;
    bool keep_alive = false;
    if (!ParseHttpRequest(head, &request)) {
      response = ErrorResponse(400, "malformed request");
    } else if (request.content_length > 0) {
      response = ErrorResponse(413, "request bodies are not accepted");
    } else if (request.method != "GET") {
      response = ErrorResponse(405, "only GET is supported");
    } else {
      keep_alive = request.keep_alive;
      try {
        response = handler_(request);
      } catch (const std::exception& e) {
        response = ErrorResponse(500, e.what());
      }
    }
    if (!SendAll(s, FormatResponse(response, keep_alive)) || !keep_alive) return;
  }
}

Status HttpClient::Connect(const std::string& host, std::uint16_t port) {
  Close();
  if (!EnsureNetwork()) return Status::Error("cannot initialize networking");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
    return Status::Error("invalid address " + host);
  }
  SocketHandle s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == kBadSocket) return Status::Error("cannot create socket");
  if (connect(s, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    CloseSocket(FromHandle(s));
    return Status::Error("cannot connect to " + host + ":" + std::to_string(port));
  }
  socket_ = FromHandle(s);
  ConfigureSocket(socket_);
  host_ = host;
  buffer_.clear();
  return Status();
}

void HttpClient::Close() {
  CloseSocket(socket_);
  socket_ = -1;
  buffer_.clear();
}

Result<HttpResponse> HttpClient::Get(const std::string& target) {
  if (!Valid(socket_)) return Status::Error("not connected");
  std::string request = "GET " + target + " HTTP/1.1\r\nHost: " + host_ + "\r\n\r\n";
  if (!SendAll(socket_, request)) return Status::Error("send failed");
  std::string head;
  if (!ReadHead(socket_, buffer_, &head)) return Status::Error("connection closed");
  HttpResponse response;
  if (head.size() < 12 || head.compare(0, 9, "HTTP/1.1 ") != 0) {
    return Status::Error("malformed response");
  }
  response.status = std::atoi(head.substr(9, 3).c_str());
  std::uint64_t length = 0;
  bool have_length = false;
  std::string_view rest(head);
  while (!rest.empty()) {
    std::size_t end = rest.find("\r\n");
    std::string_view line = rest.substr(0, end);
    rest = end == std::string_view::npos ? std::string_view() : rest.substr(end + 2);
    std::size_t colon = line.find(':');
    if (colon == std::string_view::npos) continue;
    std::string name = Lower(line.substr(0, colon));
    std::string_view value = Trim(line.substr(colon + 1));
    if (name == "content-length") have_length = ParseSize(value, &length);
    if (name == "content-type") response.content_type = std::string(value);
  }
  if (!have_length) return Status::Error("response has no length");
  while (buffer_.size() < length) {
    char chunk[65536];
    long long got = Receive(socket_, chunk, sizeof(chunk));
    if (got <= 0) return Status::Error("connection closed");
    buffer_.append(chunk, static_cast<std::size_t>(got));
  }
  response.body = buffer_.substr(0, static_cast<std::size_t>(length));
  buffer_.erase(0, static_cast<std::size_t>(length));
  return response;
}

}
