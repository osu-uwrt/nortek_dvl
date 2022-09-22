#include "async_tcp.h"
#include <iostream>

namespace asio_tcp
{

TCPClient::TCPClient()
{
  socket = ::socket(AF_INET, SOCK_STREAM, 0);
}

TCPClient::~TCPClient()
{
  this->stop();
}

void TCPClient::write(std::string msg)
{
  ::send(socket, (void*)msg.c_str(), sizeof(msg), MSG_DONTWAIT);
}

bool TCPClient::is_bound() {
  return bound;
}
/**
 * @brief This is a blocking method that readys a connection at the given
 * address and port. 
 * 
 * @param addr The IP addr to connect to
 * @param port The port at the IP address
 * @return int -1 if the connect call fails, 0 if it completes
 */
int TCPClient::connect(in_addr_t addr, int port)
{
  int cnd = ERR_CONN_FAILED;
  struct sockaddr_in conn_addr;
  conn_addr.sin_family = AF_INET;
  conn_addr.sin_addr.s_addr = addr;
  conn_addr.sin_port = htons(port);
  cnd = ::connect(socket, (struct sockaddr*)&conn_addr, sizeof(conn_addr));
  if(cnd < 0)
    return cnd;
  bound = true;
  return cnd;
}

int TCPClient::handle_read(void* data, std::size_t n)
{
  if(!bound) 
    return ERR_UNBOUND;
  //add code to peek the buffer and ensure that the buffer contains something
  //read until the buffer peaks clear in a seperate method
  ::recv(socket, data, n, 0);
  return ERR_OK;
}

void TCPClient::stop()
{
  bound = false;
  ::shutdown(socket, SHUT_RDWR);
}
}  // namespace asio_tcp