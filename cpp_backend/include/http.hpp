#pragma once
#include "service.hpp"
#include <civetweb.h>
#include <condition_variable>
namespace fourwis {
class HttpServer {
  Service &service;
  mg_context *context = nullptr;
  std::mutex clients_mutex, snapshot_mutex;
  std::map<mg_connection *, std::string> clients;
  std::string snapshot;
  std::condition_variable changed;
  std::thread ticker, broadcaster;
  std::atomic<bool> active{false};
  fs::path frontend;
  static int request(mg_connection *connection, void *user);
  static int ws_connect(const mg_connection *connection, void *user);
  static void ws_ready(mg_connection *connection, void *user);
  static int ws_data(mg_connection *connection, int bits, char *data,
                     size_t length, void *user);
  static void ws_close(const mg_connection *connection, void *user);

public:
  HttpServer(Service &s, int port);
  ~HttpServer();
  HttpServer(const HttpServer &) = delete;
  HttpServer &operator=(const HttpServer &) = delete;
};
Response http_call(int port, const std::string &method, const std::string &path,
                   const json &body = json());
int mcp_stdio(Service *service, int port, const fs::path &root);
} // namespace fourwis
