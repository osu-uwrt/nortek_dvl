#include "async_tcp.h"
#include <iostream>
#include <boost/bind.hpp>

namespace asio_tcp
{

TCPClient::TCPClient(std::function<void(std::string)> callback)
  : io_context(), socket_(io_context), deadline_(io_context), callback_(callback)
{
}

void TCPClient::connect(std::string address, int port, std::chrono::seconds timeout)
{
  endpoint_ = tcp::endpoint(boost::asio::ip::address::from_string(address), port);
  timeout_ = timeout;
  start_connect();
}

void TCPClient::start_connect()
{
  // Set a deadline for the connect operation.
  deadline_.expires_after(timeout_);

  // Start the asynchronous connect operation.
  socket_.async_connect(endpoint_, boost::bind(&TCPClient::handle_connect, this));
}

void TCPClient::handle_connect(const boost::system::error_code& ec)
{
  if (stopped_)
    return;

  // The async_connect() function automatically opens the socket at the start
  // of the asynchronous operation. If the socket is closed at this time then
  // the timeout handler must have run first.
  if (!socket_.is_open() || ec)
  {
    if (ec)
    {
      // Connection establish fail
      std::cout << "Connect error: " << ec.message() << std::endl;

      // We need to close the socket used in the previous connection attempt
      // before starting a new one.
      socket_.close();
    }
    else
    {
      std::cout << "Connect timed out" << std::endl;
    }

    // Try the next available endpoint.
    start_connect();
  }

  // Otherwise we have successfully established a connection.
  else
  {
    std::cout << "Connected to " << endpoint_ << "\n";

    // Start the input actor.
    start_read();
  }
}

void TCPClient::start_read()
{
  // Set a deadline for the read operation.
  deadline_.expires_after(timeout_);

  // Start an asynchronous operation to read a newline-delimited message.
  boost::asio::async_read_until(
      socket_, boost::asio::dynamic_buffer(input_buffer_), '\n',
      boost::bind(&TCPClient::handle_read, this, boost::placeholders::_1, boost::placeholders::_2));
}

void TCPClient::handle_read(const boost::system::error_code& ec, std::size_t n)
{
  if (stopped_)
    return;

  if (!ec)
  {
    // Extract the newline-delimited message from the buffer.
    std::string line(input_buffer_.substr(0, n - 1));
    input_buffer_.erase(0, n);

    // Empty messages are heartbeats and so ignored.
    if (!line.empty())
    {
      callback_(line);
    }

    start_read();
  }
  else
  {
    std::cout << "Error on receive: " << ec.message() << "\n";

    throw boost::system::system_error(ec);
  }
}

void TCPClient::check_deadline()
{
  if (stopped_)
    return;

  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (deadline_.expiry() <= steady_timer::clock_type::now())
  {
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled.
    socket_.close();

    // There is no longer an active deadline. The expiry is set to the
    // maximum time point so that the actor takes no action until a new
    // deadline is set.
    deadline_.expires_at(steady_timer::time_point::max());
  }

  // Put the actor back to sleep.
  deadline_.async_wait(boost::bind(&TCPClient::check_deadline, this));
}

void TCPClient::stop()
{
  stopped_ = true;
  boost::system::error_code ignored_ec;
  socket_.close(ignored_ec);
  deadline_.cancel();
}
}  // namespace asio_tcp