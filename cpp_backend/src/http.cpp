#include "http.hpp"
#include <chrono>
#include <iostream>
namespace fourwis {
namespace {
bool allowed_origin(const mg_connection *c) {
  auto value = mg_get_header(c, "Origin");
  if (!value)
    return true;
  return std::regex_match(
      value, std::regex("^http://(127\\.0\\.0\\.1|localhost)(:[0-9]+)?$"));
}
void send(mg_connection *c, const Response &r) {
  std::string cors;
  auto origin = mg_get_header(c, "Origin");
  if (origin && allowed_origin(c))
    cors = "Access-Control-Allow-Origin: " + std::string(origin) +
           "\r\nVary: Origin\r\nAccess-Control-Allow-Credentials: true\r\n";
  auto reason = r.status < 300    ? "OK"
                : r.status == 404 ? "Not Found"
                : r.status == 422 ? "Unprocessable Entity"
                                  : "Error";
  mg_printf(c,
            "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: "
            "%zu\r\nCache-Control: no-store\r\nX-Content-Type-Options: "
            "nosniff\r\n%s\r\n",
            r.status, reason, r.content_type.c_str(), r.body.size(),
            cors.c_str());
  mg_write(c, r.body.data(), r.body.size());
}
std::string decode(const std::string &s) {
  std::vector<char> b(s.size() + 1);
  int n = mg_url_decode(s.data(), int(s.size()), b.data(), int(b.size()), 1);
  if (n < 0)
    throw Error(400, "invalid URL encoding");
  return std::string(b.data(), size_t(n));
}
} // namespace
int HttpServer::request(mg_connection *c, void *user) {
  auto *self = static_cast<HttpServer *>(user);
  try {
    auto info = mg_get_request_info(c);
    std::string method = info->request_method,
                path = info->local_uri ? info->local_uri : "/";
    if (!allowed_origin(c))
      throw Error(403, "origin is not permitted for the local simulator");
    if (method == "OPTIONS") {
      mg_printf(c,
                "HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: "
                "%s\r\nAccess-Control-Allow-Methods: GET, POST, PUT, PATCH, "
                "DELETE, OPTIONS\r\nAccess-Control-Allow-Headers: "
                "Content-Type\r\nContent-Length: 0\r\n\r\n",
                mg_get_header(c, "Origin") ? mg_get_header(c, "Origin")
                                           : "http://127.0.0.1");
      return 204;
    }
    if (path.rfind("/api/", 0) != 0 && path != "/health" &&
        path != "/openapi.json") {
      if (method != "GET" && method != "HEAD")
        throw Error(405, "method not allowed");
      auto relative = fs::u8path(path.substr(1));
      for (const auto &part : relative)
        if (part == "..")
          throw Error(404, "file not found");
      auto file = fs::weakly_canonical(self->frontend / relative);
      auto root = fs::weakly_canonical(self->frontend);
      auto contained = file.lexically_relative(root);
      if (contained.empty() || *contained.begin() == "..")
        throw Error(404, "file not found");
      if (fs::is_directory(file))
        file /= "index.html";
      if (!fs::is_regular_file(file))
        throw Error(404, "file not found");
      mg_send_file(c, file.u8string().c_str());
      return 200;
    }
    constexpr size_t limit = 32 * 1024 * 1024;
    if (info->content_length > static_cast<long long>(limit))
      throw Error(413, "request body is too large");
    std::string bytes;
    char buffer[8192];
    int n;
    while ((n = mg_read(c, buffer, sizeof(buffer))) > 0) {
      if (bytes.size() + size_t(n) > limit)
        throw Error(413, "request body is too large");
      bytes.append(buffer, size_t(n));
    }
    if (n < 0)
      throw Error(400, "request body read failed");
    json body;
    if (!bytes.empty())
      body = json::parse(bytes);
    std::map<std::string, std::string> query;
    if (info->query_string)
      for (auto &item : split(info->query_string, '&')) {
        auto at = item.find('=');
        query[decode(item.substr(0, at))] =
            at == item.npos ? "" : decode(item.substr(at + 1));
      }
    auto response = self->service.request(method, path, body, query);
    send(c, response);
    return response.status;
  } catch (const Error &e) {
    send(c, Response(json{{"detail", e.detail}}, e.status));
    return e.status;
  } catch (const json::exception &e) {
    send(c, Response(json{{"detail",
                           std::string("invalid JSON request: ") + e.what()}},
                     422));
    return 422;
  } catch (const std::invalid_argument &e) {
    send(c, Response(json{{"detail", e.what()}}, 422));
    return 422;
  } catch (const std::exception &e) {
    send(c, Response(json{{"detail", e.what()}}, 500));
    return 500;
  }
}
int HttpServer::ws_connect(const mg_connection *c, void *) {
  return allowed_origin(c) ? 0 : 1;
}
void HttpServer::ws_ready(mg_connection *c, void *user) {
  auto *self = static_cast<HttpServer *>(user);
  size_t n;
  {
    std::lock_guard<std::mutex> lock(self->clients_mutex);
    self->clients[c] = uid();
    n = self->clients.size();
  }
  {
    std::lock_guard<std::recursive_mutex> lock(self->service.mutex);
    self->service.subscribers = n;
  }
}
int HttpServer::ws_data(mg_connection *c, int bits, char *bytes, size_t length,
                        void *user) {
  auto *self = static_cast<HttpServer *>(user);
  if (length > 1024 * 1024)
    return 0;
  if ((bits & 15) == 8)
    return 0;
  if ((bits & 15) == 9) {
    mg_websocket_write(c, 10, bytes, length);
    return 1;
  }
  if ((bits & 15) != 1)
    return 1;
  try {
    auto data = json::parse(bytes, bytes + length);
    if (!data.is_object())
      return 1;
    if (str(data, "type") == "ping") {
      auto pong = json{{"type", "pong"}, {"t", data.value("t", json())}}.dump();
      mg_websocket_write(c, 1, pong.data(), pong.size());
      return 1;
    }
    std::string owner;
    {
      std::lock_guard<std::mutex> lock(self->clients_mutex);
      auto it = self->clients.find(c);
      if (it == self->clients.end())
        return 0;
      owner = it->second;
    }
    self->service.ws(data, owner);
  } catch (const std::exception &e) {
    std::cerr << "Ignored invalid WebSocket command: " << e.what() << '\n';
  }
  return 1;
}
void HttpServer::ws_close(const mg_connection *c, void *user) {
  auto *self = static_cast<HttpServer *>(user);
  std::string owner;
  size_t count;
  {
    std::lock_guard<std::mutex> lock(self->clients_mutex);
    auto it = self->clients.find(const_cast<mg_connection *>(c));
    if (it != self->clients.end()) {
      owner = it->second;
      self->clients.erase(it);
    }
    count = self->clients.size();
  }
  self->service.disconnect(owner);
  {
    std::lock_guard<std::recursive_mutex> lock(self->service.mutex);
    self->service.subscribers = count;
  }
}
HttpServer::HttpServer(Service &s, int port) : service(s) {
  if (port < 1 || port > 65535)
    throw Error(422, "invalid HTTP port");
  auto dist = env_path("SIM4WIS_DIST_DIR");
  frontend = dist.value_or(service.store.root / "frontend/dist");
  if (!fs::exists(frontend) && !dist)
    frontend = service.store.root / "app/web";
  std::string binding = "127.0.0.1:" + std::to_string(port);
  const char *options[] = {"listening_ports",
                           binding.c_str(),
                           "num_threads",
                           "12",
                           "request_timeout_ms",
                           "5000",
                           "websocket_timeout_ms",
                           "30000",
                           "enable_websocket_ping_pong",
                           "yes",
                           "enable_directory_listing",
                           "no",
                           nullptr};
  mg_callbacks callbacks{};
  context = mg_start(&callbacks, this, options);
  if (!context)
    throw Error(500, "cannot start C++ HTTP server on " + binding);
  mg_set_websocket_handler(context, "/ws/state", ws_connect, ws_ready, ws_data,
                           ws_close, this);
  mg_set_request_handler(context, "/", request, this);
  active = true;
  ticker = std::thread([this] {
    using Clock = std::chrono::steady_clock;
    auto next = Clock::now();
    size_t ticks = 0;
    while (active) {
      std::string update;
      {
        std::lock_guard<std::recursive_mutex> lock(service.mutex);
        service.live.tick();
        if (++ticks % 3 == 0)
          update = service.live.state().dump(-1, ' ', false,
                                             json::error_handler_t::replace);
      }
      if (!update.empty()) {
        {
          std::lock_guard<std::mutex> lock(snapshot_mutex);
          snapshot = std::move(update);
        }
        changed.notify_one();
      }
      next += std::chrono::milliseconds(5);
      if (next < Clock::now())
        next = Clock::now();
      std::this_thread::sleep_until(next);
    }
  });
  broadcaster = std::thread([this] {
    while (active) {
      std::string message;
      {
        std::unique_lock<std::mutex> lock(snapshot_mutex);
        changed.wait_for(lock, std::chrono::milliseconds(100),
                         [&] { return !snapshot.empty() || !active; });
        message.swap(snapshot);
      }
      if (message.empty())
        continue;
      std::lock_guard<std::mutex> lock(clients_mutex);
      for (auto &client : clients)
        mg_websocket_write(client.first, 1, message.data(), message.size());
    }
  });
}
HttpServer::~HttpServer() {
  active = false;
  changed.notify_all();
  if (ticker.joinable())
    ticker.join();
  if (broadcaster.joinable())
    broadcaster.join();
  if (context)
    mg_stop(context);
}
Response http_call(int port, const std::string &method, const std::string &path,
                   const json &body) {
  char error[256];
  auto *c = mg_connect_client("127.0.0.1", port, 0, error, sizeof(error));
  if (!c)
    throw Error(503, std::string("local backend is unavailable: ") + error);
  std::string text = body.is_null() ? "" : body.dump();
  mg_printf(
      c,
      "%s %s HTTP/1.1\r\nHost: 127.0.0.1:%d\r\nContent-Type: "
      "application/json\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n",
      method.c_str(), path.c_str(), port, text.size());
  if (!text.empty())
    mg_write(c, text.data(), text.size());
  if (mg_get_response(c, error, sizeof(error), 10000) < 0) {
    mg_close_connection(c);
    throw Error(503, std::string("local backend response failed: ") + error);
  }
  auto *info = mg_get_response_info(c);
  int status = info->status_code;
  const char *header = mg_get_header(c, "Content-Type");
  std::string mime = header ? header : "application/json", result;
  char buffer[16384];
  int n;
  while ((n = mg_read(c, buffer, sizeof(buffer))) > 0) {
    result.append(buffer, n);
    if (result.size() > 128 * 1024 * 1024) {
      mg_close_connection(c);
      throw Error(413, "backend response too large for MCP");
    }
  }
  mg_close_connection(c);
  return Response(result, mime, status);
}
} // namespace fourwis
