#include "madcWrapperEdge.h"
#include "tcpConnection.hpp"
#include <fstream>
#include <omp.h>

//extern envMap* volatile leftChildsMap;
//extern envMap* volatile rightChildsMap;
extern envMap* volatile thisDevicesMap;

extern envMap* volatile childrensMaps[NUMBER_OF_CHILDREN];

extern double tr;

std::string read_(boost::asio::ip::tcp::socket & socket) {
  boost::asio::streambuf buf;
  boost::asio::read_until( socket, buf, "\n" );
  std::string data = boost::asio::buffer_cast<const char*>(buf.data());
  return data;
}
void send_(boost::asio::ip::tcp::socket & socket, const std::string& message) {
  const std::string msg = message + "\n";
  boost::asio::write( socket, boost::asio::buffer(message) );
}


void* startServer(void* inputArg){

  double start_time = omp_get_wtime();
  int* portNumber = &(   ((tcpThreadArguments_t*)inputArg)->portNumber  );

  std::cout << "setting a server at cpu: " << (((*portNumber)-1234+6)%8) << std::endl;
  //  set_cpu(((*portNumber)-1234+6)%8);
  
  /*
  if (*portNumber == 1234){
    set_cpu(6);
  } else if (*portNumber == 1235){
    set_cpu(7);
  } else {
    std::cout << "something's fucked up with start server" << std::endl;
    }
*/
  
  boost::asio::io_service io_service;
  //listen for new connection
  boost::asio::ip::tcp::acceptor
    acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), *portNumber ));

  //socket creation 
  boost::asio::ip::tcp::socket socket_(io_service);
  //waiting for connection
  acceptor_.accept(socket_);
  //read operation
  std::string message = read_(socket_);
  //  std::cout << message << std::endl;
  send_(socket_, "Ack From the Server!");
  
  double stop_time = omp_get_wtime();

  childrensMaps[((*portNumber)-1234)] = new envMap(message);
  std::cout << "deserialized the i-th child's map: " << (*portNumber)-1234;
  /*
  if (*portNumber == 1234){
    leftChildsMap = new envMap(message);
  } else if (*portNumber == 1235){
    rightChildsMap = new envMap(message);
  } else {
    std::cout << "something's fucked up with start server" << std::endl;
    }*/

  char fileName[50];
  sprintf(fileName, "serialized%d.txt", *portNumber);
  std::ofstream serializedMapFile(fileName);
  serializedMapFile  << message;  
  
  return NULL;
}

void* startClient(void* inputArg){
  //  set_cpu(5);
  std::string * serverAddress = &(   ((tcpThreadArguments_t*)inputArg)->serverAddress  );
  int* portNumber = &(   ((tcpThreadArguments_t*)inputArg)->portNumber  );
  
  boost::asio::io_service io_service;
  //socket creation
  boost::asio::ip::tcp::socket socket(io_service);
  //connection
  socket.connect( boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string(*serverAddress), *portNumber));
  // request/message from client

  //###### const std::string msg = "Hello from Client!\n";
  while (thisDevicesMap == NULL)
    ;
  const std::string msg = thisDevicesMap->serializeTheMap();
  //###### std::cout << msg << std::endl;


  
  double start_time = omp_get_wtime();
  boost::system::error_code error;
  boost::asio::write( socket, boost::asio::buffer(msg+"\n"), error );
  if( !error ) {
    //    std::cout << "Client sent hello message!" << std::endl;
  }
  else {
    std::cout << "send failed: " << error.message() << std::endl;
  }
  // getting response from server
  boost::asio::streambuf receive_buffer;
  boost::asio::read(socket, receive_buffer, boost::asio::transfer_all(), error);
  if( error && error != boost::asio::error::eof ) {
    std::cout << "receive failed: " << error.message() << std::endl;
  }
  else {
    const char* data = boost::asio::buffer_cast<const char*>(receive_buffer.data());
    //     std::cout << data << std::endl;
  }
  double stop_time = omp_get_wtime();

  tr = stop_time - start_time;
    

  char fileName[50];
  std::ofstream profileStream;
  sprintf(fileName, "communicationTime%d.txt", *portNumber);
  profileStream.open(fileName, std::ofstream::out | std::ofstream::app);
  profileStream << stop_time - start_time << std::endl;
  
  return NULL;
}

