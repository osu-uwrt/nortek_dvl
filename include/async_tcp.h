#pragma once

#include <boost/asio.hpp>
#include <string>
#include <chrono>

using namespace boost::asio;
using ip::tcp;

namespace asio_tcp
{
    class TCPClient{
        public:
        TCPClient();
        ~TCPClient();
        bool connect(std::string ip, int port, std::chrono::duration timeout);

        private:

    };
} // namespace asio_tcp
