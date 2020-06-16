#include "baselineWrapper.h"
#include "networkConfig.hpp"
#include "tcpConnection_baseline.hpp"
#include <omp.h>
#include <time.h>


std::vector<idxXYZ> childrensPtCloudes[NUMBER_OF_CHILDREN+1];

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
  tcpPid = new pthread_t[NUMBER_OF_CHILDREN+1]; //one for every child and one for the parent
  tcpThreadData = new tcpThreadArguments_t[NUMBER_OF_CHILDREN+1]; //one for every child and one for the parent

  std::cout << "NOISE = " << NOISE << std::endl;

  for(int server = 0; server < NUMBER_OF_CHILDREN; server++){
    std::cout << "creating the thread for child " << server << std::endl;
    tcpThreadData[server].message = "Hi my child";
    tcpThreadData[server].portNumber = 1234 + server;
    pthread_create(&tcpPid[server], NULL, startServer_baseline, &tcpThreadData[server]);
  }
  
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
  
  double start_omp = omp_get_wtime();

  for(int server = 0; server < NUMBER_OF_CHILDREN; server++){
    pthread_join(tcpPid[server], NULL);
    
    localClustererData.data.insert(localClustererData.data.end(),
				   (childrensPtCloudes[server]).begin(),
				   (childrensPtCloudes[server]).end()
				   );
  }

  double stop_omp = omp_get_wtime();
  bw = stop_omp - start_omp;

#ifdef PARENT //get served by the parent
  std::cout << "My parent is: " << PARENT << std::endl;
  tcpThreadData[NUMBER_OF_CHILDREN].message = "Hi my server";
  tcpThreadData[NUMBER_OF_CHILDREN].serverAddress = PARENT;
  tcpThreadData[NUMBER_OF_CHILDREN].portNumber = PORT_NUMBER_TO_CONNECT_TO;
  pthread_create(&tcpPid[NUMBER_OF_CHILDREN], NULL, startClient_baseline, &tcpThreadData[NUMBER_OF_CHILDREN]);
#endif
  
  barrier_init(&barrier, 1);
  localClustererData.offset = 1000*OFFSET;

#if DBSCAN==1
  pthread_create(&localClusterer_pid, NULL, performDBSCAN_baseline, &localClustererData);
#elif EUCLIDEAN==1
  pthread_create(&localClusterer_pid, NULL, performEuclideanClustering_baseline, &localClustererData);
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
  
  barrier_init(&barrier, 1);
  localClustererData.offset = 1000*OFFSET;


#ifdef PARENT
  pthread_join(tcpPid[NUMBER_OF_CHILDREN], NULL);
#endif

  pthread_join(localClusterer_pid, NULL);

  std::ofstream profileStream;
  profileStream.open("stats/profile_baseline.txt", std::ofstream::out | std::ofstream::app);
  profileStream << lc << "\t\t"
		<< co << "\t\t"
    		<< bw << "\t\t"
		<< tr << std::endl;

  profileStream.close();

  return 0;
}
