#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <chrono>
#include <functional>

namespace asio_tcp
{

class TCPClient
{
public:
  TCPClient();
  ~TCPClient();
  int handle_read(void* data, std::size_t n);
  int connect(in_addr_t ip, int port);
  void write(std::string msg);
  void stop();
  bool is_bound();

private:
  void start_connect();
  void start_read();

  // socket creation
  bool bound = false;
  int socket;

  // internal buffers
  std::string input_buffer_;
  bool stopped_;
  std::chrono::seconds timeout_;
  std::function<void(std::string)> callback_;

  enum err_cd {
      ERR_OK = 0,
      ERR_CONN_FAILED = -1,
      ERR_UNBOUND = -2,
      ERR_TIMEOUT = -3
  };
};
}  // namespace asio_tcp
