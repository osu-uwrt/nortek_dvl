#pragma once

#include <boost/asio.hpp>
#include <string>
#include <chrono>
#include <functional>

using namespace boost::asio;
using ip::tcp;

namespace asio_tcp
{

class TCPClient
{
public:
  TCPClient(std::function<void(std::string)> callback);
  ~TCPClient();
  void connect(std::string ip, int port, std::chrono::seconds timeout);
  void write();

private:
  void start_connect();
  void handle_connect(const boost::system::error_code& ec);
  void start_read();
  void handle_read(const boost::system::error_code& ec, std::size_t n);
  void check_deadline();
  void stop();

  // socket creation
  boost::asio::io_context io_context;
  tcp::socket socket_;
  tcp::endpoint endpoint_;
  steady_timer deadline_;

  // internal buffers
  std::string input_buffer_;
  bool stopped_;
  std::chrono::seconds timeout_;
  std::function<void(std::string)> callback_;
};
}  // namespace asio_tcp
