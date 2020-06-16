#include "madcWrapper.h"
#include <omp.h>
std::vector<idxXYZ> data; //concatenated data: used for the baseline method!
int totalNumberOfComparisons = 0;

barrier_t barrier;

pthread_t baseline_pid;
threadArguments_t baselineThreadData;

pthread_t *pid;
threadArguments_t *threadData;

//unsigned long long cost_maxLocal = 0;

unsigned long long cost_sumCombine = 0;
unsigned long long cost_sumLocalPCL = 0;
unsigned long long cost_sumWaiting = 0;

unsigned long long cost_MAD_C_execution = 0; //everything's included!

unsigned long long start;
unsigned long long stop;

double start_omp, stop_omp, cost_madc_omp, cost_baseline_omp;

bool myFunc(idxXYZ a, idxXYZ b){
  return (a.x < b.x);
}

int main(int argc, char* argv[]){

  std::cout << "NOISE = " << NOISE << std::endl;
  
  //  set_cpu(0);
  
  int numberOfThreads = atoi(argv[2]);

  barrier_init(&barrier, numberOfThreads);
  
  pid = new pthread_t[numberOfThreads];
  threadData = new threadArguments_t[numberOfThreads];

  /* Read the input point cloud: */

  std::string basename(argv[1]);
  std::ifstream ptCloudInFile((basename).c_str());

  double x,y,z;
  while ( ptCloudInFile >> x >> y >> z){
    if ( z < -2.1) /* GROUND REMOVAL */
      continue;

    idxXYZ tuple;
    /* REARANGEMENT OF XYZ */
    tuple.x = x; tuple.y = z; tuple.z = y; tuple.clusterIDX =-1; tuple.index_ready = false;/*WARNING!*/ 
    baselineThreadData.data.push_back(tuple); //BASELINE
  }

  if (sortInputPointCloud){
    std::sort(baselineThreadData.data.begin(), baselineThreadData.data.end(), myFunc);
  }
  
  int number_of_samples = baselineThreadData.data.size();
  int number_of_points_in_each_bucket = ((number_of_samples)/(numberOfThreads))+1;
  int thread = 0;
  int size_of_bucket = 0;
  for (int i = 0; i < number_of_samples; i++){
    if (size_of_bucket >= number_of_points_in_each_bucket){
      thread++;
      size_of_bucket = 0;
    }
    threadData[thread].data.push_back(baselineThreadData.data[i]);
    size_of_bucket++;
  }
  int sum_partion_sizes = 0;
  for (int i = 0; i < numberOfThreads; i++){
    sum_partion_sizes+= threadData[i].data.size();
  }
  assert(sum_partion_sizes == number_of_samples);

  /******************************
   * TIME TO GET STARTED
   ******************************/

  //#1:start = rdtsc();
  start_omp = omp_get_wtime();
  for (int i = 0; i < numberOfThreads; i++){
    threadData[i].offset = 1000*(i+1);
    threadData[i].threadID = i;
    threadData[i].K = numberOfThreads - 1;
    threadData[i].mapReady = false;

#if DBSCAN==1
    pthread_create(&pid[i], NULL, performDBSCAN, &threadData[i]);
#elif EUCLIDEAN==1
    pthread_create(&pid[i], NULL, performEuclideanClustering, &threadData[i]);
#endif
  }

  for (int i = 0; i < numberOfThreads; i++){
    pthread_join(pid[i], NULL);

    cost_sumCombine      += threadData[i].cost_localCombine;
    cost_sumLocalPCL     += threadData[i].cost_localPCL;
    cost_sumWaiting      += threadData[i].cost_waitingTime;

    /* max (cost_duration) among the K threads */
    if ( cost_MAD_C_execution < threadData[i].cost_duration){
      cost_MAD_C_execution = threadData[i].cost_duration;
    }
  }
  
  //#1:stop = rdtsc();
  //#1:cost_MAD_C_execution = stop - start;

  if (PRINT_STD_OUT){
    printFunction("number of points", 20);
    printFunction("number of clusters", 20);
    printFunction("cost_localPCL", 20);
    printFunction("cost_localCombine", 20);
    printFunction("cost_waitingTime", 20);
    printFunction("cost_duration", 20);
    std::cout << std::endl;

    for (int i = 0; i < numberOfThreads; i++){
      printFunction(threadData[i].numberOfPoints, 20);
      printFunction(threadData[i].numberOfClusters, 20);
      printFunction(threadData[i].cost_localPCL, 20);
      printFunction(threadData[i].cost_localCombine, 20);
      printFunction(threadData[i].cost_waitingTime, 20);
      printFunction(threadData[i].cost_duration, 20);
      std::cout << std::endl;
    }
    
  } //end if (PRINT_STD_OUT)
  

  /* MADC-ext */

  start = rdtsc();
  for (int i = 0; i < numberOfThreads; i++){
    pthread_create(&pid[i], NULL, lastPhase, &threadData[i]);    
  }
  for (int i = 0; i < numberOfThreads; i++){
    pthread_join(pid[i], NULL);
  }
  stop = rdtsc();
  stop_omp = omp_get_wtime();
  cost_MAD_C_execution += stop - start;
  cost_madc_omp = stop_omp - start_omp;

  /******************************
   * FINISH LINE:
   ******************************/

  std::cout << "cost_MAD_C_execution. 1.RDTSC: " << cost_MAD_C_execution << std::endl
	    << "2.omp: " << cost_madc_omp << std::endl;


  /* MADC-ext results */
  std::ofstream clusteringStream;
  clusteringStream.open("madc_clustering.idx");
  for (int i = 0; i < numberOfThreads; i++){
    std::vector<idxXYZ>::iterator localPointCloudIterator = threadData[i].data.begin();
    while( localPointCloudIterator != threadData[i].data.end()){
      clusteringStream << (*localPointCloudIterator).clusterIDX << std::endl;
      localPointCloudIterator++;
    }
  }
  clusteringStream.close();


  /*  
      std::ofstream stoneStream;
      stoneStream.open("stones.idx");
      for (int i = 0; i < numberOfThreads; i++){
      envMap* i_th_map = &(threadData[i].localMap);
    
      for ( std::vector<Stone*>::iterator mapIterator = i_th_map->stonesInMap.begin(); mapIterator != i_th_map->stonesInMap.end(); mapIterator++){
      stoneStream << (*mapIterator)->getId() << "\t\t" <<  (*mapIterator)->findParent()->getId() << std::endl;
      }
      }
      stoneStream.close();
  */
  
  if (performBaseline){
    barrier_init(&barrier, 1); //basically act as if there was no barrier!
    baselineThreadData.offset = 0;
    
    /******************************
     * TIME TO GET STARTED
     ******************************/
    start_omp = omp_get_wtime();

#if DBSCAN==1
    pthread_create(&baseline_pid, NULL, performDBSCAN, &baselineThreadData);
#elif EUCLIDEAN==1
    pthread_create(&baseline_pid, NULL, performEuclideanClustering, &baselineThreadData);
#endif

    pthread_join(baseline_pid, NULL);

    stop_omp = omp_get_wtime();
    cost_baseline_omp = stop_omp - start_omp;
    /******************************
     * FINISH LINE
     ******************************/

    std::cout << "the baseline:" << std::endl
	      << baselineThreadData.numberOfPoints << "\t\t"
	      << baselineThreadData.numberOfClusters << "\t\t"
	      << baselineThreadData.cost_localPCL << "\t\t"
	      << baselineThreadData.cost_waitingTime <<  std::endl
	      << "OMP: " << cost_baseline_omp << std::endl;

    /*    std::cout << "the baseline:" << std::endl;
	  std::cout << "omp timing: " << stop_omp - start_omp <<baselineThreadData.numberOfPoints << "\t\t" << baselineThreadData.numberOfClusters << "\t\t" << baselineThreadData.cost_localPCL << "\t\t" << baselineThreadData.cost_waitingTime <<  std::endl; */

    clusteringStream.open("baseline_clustering.idx");
      std::vector<idxXYZ>::iterator localPointCloudIterator = baselineThreadData.data.begin();
      while( localPointCloudIterator != baselineThreadData.data.end()){
	clusteringStream << (*localPointCloudIterator).clusterIDX << std::endl;
	localPointCloudIterator++;
      }
    clusteringStream.close();
  } //end if performBaseline

  std::ofstream profileStream;
  profileStream.open("stats/profile.txt", std::ofstream::out | std::ofstream::app);
  profileStream << baselineThreadData.cost_localPCL << "\t\t"
		<< cost_MAD_C_execution << "\t\t"
    		<< cost_sumLocalPCL << "\t\t"
		<< cost_sumCombine << "\t\t"
		<<  cost_sumWaiting << std::endl;
  profileStream.close();

  profileStream.open("stats/profile_omp.txt", std::ofstream::out | std::ofstream::app);
  profileStream << cost_baseline_omp << "\t\t" << cost_madc_omp << std::endl;
  profileStream.close();

  
  profileStream.open("stats/local_pcl.txt", std::ofstream::out | std::ofstream::app);
  for (int i = 0; i < numberOfThreads; i++){
    profileStream << threadData[i].cost_localPCL << "\t\t";
  }
  profileStream << std::endl;
  profileStream.close();

  profileStream.open("stats/local_combine.txt", std::ofstream::out | std::ofstream::app);
  for (int i = 0; i < numberOfThreads; i++){
    profileStream << threadData[i].cost_localCombine << "\t\t";
  }
  profileStream << std::endl;
  profileStream.close();

  profileStream.open("stats/local_waiting.txt", std::ofstream::out | std::ofstream::app);
  for (int i = 0; i < numberOfThreads; i++){
    profileStream << threadData[i].cost_waitingTime << "\t\t";
  }
  profileStream << std::endl;
  profileStream.close();

  profileStream.open("stats/ellipsoid_forest_depths.txt", std::ofstream::out | std::ofstream::app);
  for (int i = 0; i < numberOfThreads; i++){
    std::vector<int>::iterator depth_tree_iter = threadData[i].localMap.depth_tree.begin();
    while(depth_tree_iter != threadData[i].localMap.depth_tree.end()){
      profileStream << *depth_tree_iter << " ";
      depth_tree_iter++;
    }
    profileStream << std::endl;    
  }
  profileStream.close();

  
  return 0;
}
