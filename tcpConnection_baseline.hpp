#include <iostream>
#include <boost/asio.hpp>

#ifndef TCPCONNECTION
#define TCPCONNECTION


struct tcpThreadArguments{
  std::string serverAddress; //the address of the server to connect to
  int portNumber;
  std::string message;
} typedef tcpThreadArguments_t;

void* startServer_baseline(void *);
void* startClient_baseline(void *);

std::string serializePtCloud(std::vector<idxXYZ> data);
std::vector<idxXYZ> deserializePtCloud(std::string str);

#endif
