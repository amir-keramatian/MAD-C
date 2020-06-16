#include "madcWrapperEdge.h"
#include "networkConfig.hpp"
#include "tcpConnection.hpp"
#include <omp.h>
#include <time.h>

class envMap;
envMap* volatile leftChildsMap = NULL;
envMap* volatile rightChildsMap = NULL;
envMap* volatile thisDevicesMap = NULL;

int totalNumberOfComparisons = 0;
barrier_t barrier;

pthread_t localClusterer_pid;

threadArguments_t localClustererData;

//double start_omp, stop_omp;

/*Abbreviations. lc: local clustering, co: combine, bw: busy waiting, tr: transmission */
double lc = 0.0; double co = 0.0; double bw = 0.0; double tr = 0.0;


pthread_t *tcpPid;
tcpThreadArguments_t *tcpThreadData;



bool myFunc(idxXYZ a, idxXYZ b){
  return (a.x < b.x);
}

int main(int argc, char* argv[]){
  //  set_cpu(0);
  tcpPid = new pthread_t[3];
  tcpThreadData = new tcpThreadArguments_t[3];
  
  std::cout << "NOISE = " << NOISE << std::endl;
  
  
#ifdef leftChild //serve my left child
  std::cout << "My left child is: " << leftChild << std::endl;
  tcpThreadData[0].message = "Hi my left child";
  tcpThreadData[0].portNumber = 1234;
  pthread_create(&tcpPid[0], NULL, startServer, &tcpThreadData[0]);
#endif

#ifdef rightChild //serve my right child
  std::cout << "My right child is: " << rightChild << std::endl;
  tcpThreadData[1].message = "Hi my right child";
  tcpThreadData[1].portNumber = 1235;
  pthread_create(&tcpPid[1], NULL, startServer, &tcpThreadData[1]);
#endif

  /* Read the input point cloud: */
  std::string basename(argv[1]);
  std::ifstream ptCloudInFile((basename).c_str());

  double x,y,z;
  while ( ptCloudInFile >> x >> y >> z){
    idxXYZ tuple;
    /* REARANGEMENT OF XYZ */
    tuple.x = x; tuple.y = z; tuple.z = y; tuple.clusterIDX =-1; tuple.index_ready = false;/*WARNING!*/ 
    localClustererData.data.push_back(tuple); 
  }
  if (sortInputPointCloud){
    std::sort(localClustererData.data.begin(), localClustererData.data.end(), myFunc);
  }
  
  if (argc == 3){
    int minute = atoi(argv[2]);

    time_t now;
    struct tm *now_tm;
    int minute_now;

    while(true){
      now = time(NULL);
      now_tm = localtime(&now);
      minute_now = now_tm->tm_min;
      //      std::cout << "minute now = " << minute_now << std::endl;
      if (minute == minute_now)
	break;
    }
  }
  //  std::cout << "freeeeeeeeeeeeeeee" << std::endl;
  
  
  /******************************
   * TIME TO GET STARTED
   ******************************/
  barrier_init(&barrier, 1);

  localClustererData.offset = 1000*OFFSET;

#if DBSCAN==1
  pthread_create(&localClusterer_pid, NULL, performDBSCAN_edge, &localClustererData);
#elif EUCLIDEAN==1
  pthread_create(&localClusterer_pid, NULL, performEuclideanClustering_edge, &localClustererData);
#endif
  
  if (PRINT_STD_OUT){
    printFunction("number of points", 20);
    printFunction("number of clusters", 20);
    printFunction("cost_localPCL", 20);
    std::cout << std::endl;

    printFunction(localClustererData.numberOfPoints, 20);
    printFunction(localClustererData.numberOfClusters, 20);
    printFunction(localClustererData.cost_localPCL, 20);
    std::cout << std::endl;
  } //end if (PRINT_STD_OUT)
  
  
#ifdef PARENT //get served by the parent
  std::cout << "My parent is: " << PARENT << std::endl;
  tcpThreadData[2].message = "Hi my server";
  tcpThreadData[2].serverAddress = PARENT;
  tcpThreadData[2].portNumber = PORT_NUMBER_TO_CONNECT_TO;
  pthread_create(&tcpPid[2], NULL, startClient, &tcpThreadData[2]);
#endif

#ifdef leftChild 
  pthread_join(tcpPid[0], NULL);
#endif
  
#ifdef rightChild 
  pthread_join(tcpPid[1], NULL);
#endif

#ifdef PARENT
  pthread_join(tcpPid[2], NULL);
#endif

  pthread_join(localClusterer_pid, NULL);

  std::ofstream profileStream;
  profileStream.open("stats/profile.txt", std::ofstream::out | std::ofstream::app);
  profileStream << lc << "\t\t"
		<< co << "\t\t"
    		<< bw << "\t\t"
		<< tr << std::endl;

  profileStream.close();

  return 0;
}
