#include "madcWrapperEdge.h"
#include "tcpConnection_baseline.hpp"
#include <fstream>
#include <omp.h>

extern std::vector<idxXYZ> childrensPtCloudes[NUMBER_OF_CHILDREN+1];

extern double tr;
extern threadArguments_t localClustererData;

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
  set_cpu(((*portNumber)-1234+6)%8);

  
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

  childrensPtCloudes[((*portNumber)-1234)] = deserializePtCloud(message);

  char fileName[50];
  sprintf(fileName, "serialized%d.txt", *portNumber);
  std::ofstream serializedMapFile(fileName);
  serializedMapFile  << message;  
  
  return NULL;
}



void* startClient_baseline(void* inputArg){
  set_cpu(5);
  std::string * serverAddress = &(   ((tcpThreadArguments_t*)inputArg)->serverAddress  );
  int* portNumber = &(   ((tcpThreadArguments_t*)inputArg)->portNumber  );
  
  boost::asio::io_service io_service;
  //socket creation
  boost::asio::ip::tcp::socket socket(io_service);
  //connection
  socket.connect( boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string(*serverAddress), *portNumber));
  // request/message from client

  //###### const std::string msg = "Hello from Client!\n";

  const std::string msg = serializePtCloud(localClustererData.data);
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

std::string serializePtCloud(std::vector<idxXYZ> data){
  std::string result;
  std::ostringstream strs;

  strs << data.size() << ",";
  
  std::vector<idxXYZ>::iterator data_iterator = data.begin();

  while(data_iterator != data.end()){

    strs << data_iterator->x << ","
	 << data_iterator->y << ","
	 << data_iterator->z << ",";
    
    data_iterator++;
  }

  result = strs.str();
  return result;
}


std::vector<idxXYZ> deserializePtCloud(std::string serializedPtCloud){

  std::vector<idxXYZ> result;
  
  char *saveptr;
  char* token;
  token = strtok_r (const_cast<char*>(serializedPtCloud.c_str()), ",", &saveptr);

  int numberOfPoints = atoi(token);

  for (int point = 0; point < numberOfPoints; point++){
    idxXYZ tuple;
    tuple.x = atof(strtok_r (NULL, ",",&saveptr));
    tuple.y = atof(strtok_r (NULL, ",",&saveptr));
    tuple.z = atof(strtok_r (NULL, ",",&saveptr));
    result.push_back(tuple);
  }
  
  return result;
}
