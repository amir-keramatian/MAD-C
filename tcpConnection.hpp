#include <iostream>
#include <boost/asio.hpp>

//using namespace boost::asio;
//using ip::tcp;

#ifndef TCPCONNECTION
#define TCPCONNECTION


struct tcpThreadArguments{
  std::string serverAddress; //the address of the server to connect to
  int portNumber;
  std::string message;
} typedef tcpThreadArguments_t;

void* startServer(void *);
void* startClient(void *);

#endif
