#include "madcWrapper.h"
extern threadArguments_t *threadData;
extern barrier_t barrier;


void* performEuclideanClustering(void* inputArg){

  barrier_cross(&barrier);
    
  unsigned long long start_duration = rdtsc();
  
  set_cpu(((threadArguments*)inputArg)->threadID+1);
    
  envMap *localMap = &(((threadArguments*)inputArg)->localMap);
  int offset = ((threadArguments*)inputArg)->offset;
  std::vector<idxXYZ>* inputData = &(((threadArguments*)inputArg)->data);
  int localThreadID = ((threadArguments*)inputArg)->threadID;
  int K = ((threadArguments*)inputArg)->K;


  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int numberOfPoints = 0;

  std::vector<idxXYZ>::iterator inputDataIterator;

  for (inputDataIterator = inputData->begin(); inputDataIterator != inputData->end(); ++inputDataIterator){
    cloud->points.push_back(pcl::PointXYZ((*inputDataIterator).x, (*inputDataIterator).y, (*inputDataIterator).z));
    ++numberOfPoints;    
  }
  cloud->width = numberOfPoints; 
  cloud->height = 1;
  cloud->is_dense = false; 

  
  unsigned long long start = rdtsc();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (EPSILON);
  ec.setMinClusterSize (MinPts);
  ec.setMaxClusterSize (750000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  unsigned long long stop = rdtsc();

  int j = 1;
  void* dummy;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    Stone* ellipsoid = new Stone(offset + j);
    ellipsoid->offset_signature = offset;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      (*inputData)[*pit].clusterIDX = offset + j;
      ellipsoid->addMember((*inputData)[*pit]); 
    }
    dummy = (*ellipsoid).getCharacteristicMatrix();
    localMap->insertStone(*ellipsoid);
    j++;
  } //end for it = cluster_indices.begin() ...

  ((threadArguments*)inputArg)->numberOfClusters = j - 1;
  ((threadArguments*)inputArg)->numberOfPoints = inputData->size();
  ((threadArguments*)inputArg)->cost_localPCL = stop - start;


  ((threadArguments*)inputArg)->cost_localCombine = 0;
  
  unsigned long long start_wait_and_combine = rdtsc();

  int whereAreMyChildren[K+1][K+1];
  for (int i = 0; i <= K; i++){
    for (int j = 0; j <= K; j++){
      whereAreMyChildren[i][j] = 0;
    }
  }
  /*
  for (int i = 0; i <= K - 1; i++){
    whereAreMyChildren[K][i] = 1;
    }*/

  
  for (int i = 0; i <= K; i++){
    if( 2*i + 1 <= K){
      whereAreMyChildren[i][2*i+1] = 1;
    }
    if( 2*i + 2 <= K){
      whereAreMyChildren[i][2*i+2] = 1;
    }
  }

  
  bool unprocessedChildExists;
  do{
    unprocessedChildExists = false;
    for(int index = 0; index <= K; index++){
      if (whereAreMyChildren[localThreadID][index] == 1){ //index is an unprocessed child

	if(threadData[index].mapReady == false){
	  unprocessedChildExists = true;
	}

	else{
	  unsigned long long start_combine = rdtsc();
	  localMap->combineV7(threadData[index].localMap);
	  unsigned long long stop_combine = rdtsc();

	  ((threadArguments*)inputArg)->cost_localCombine += stop_combine - start_combine;
	  
	  whereAreMyChildren[localThreadID][index] = 0; //remove index from the unprocessed list
	}
	
      }
    }
  } while(unprocessedChildExists);

  unsigned long long stop_wait_and_combine = rdtsc();

  ((threadArguments*)inputArg)->cost_waitingTime = stop_wait_and_combine - start_wait_and_combine -
    ((threadArguments*)inputArg)->cost_localCombine;

  ((threadArguments*)inputArg)->mapReady = true;

  unsigned long long stop_duration = rdtsc();
  
  ((threadArguments*)inputArg)->cost_duration = stop_duration - start_duration;
  
  pthread_exit(0);
}

void* lastPhase(void * inputArg){
  
  set_cpu(((threadArguments*)inputArg)->threadID+1);
  
  int localThreadID = ((threadArguments*)inputArg)->threadID;
    
  std::vector<idxXYZ>::iterator localPointCloudIterator = threadData[localThreadID].data.begin();

  while( localPointCloudIterator != threadData[localThreadID].data.end()){
    if ( (*localPointCloudIterator).clusterIDX != NOISE){ // if not noise
      (*localPointCloudIterator).clusterIDX = 
	threadData[localThreadID].localMap.getParentID((*localPointCloudIterator).clusterIDX);
    }
    localPointCloudIterator++;
  }
  
  pthread_exit(0);
}

/*
  start = rdtsc();
  for (int i = 0; i < numberOfThreads; i++){
    std::vector<idxXYZ>::iterator localPointCloudIterator = threadData[i].data.begin();
    while( localPointCloudIterator != threadData[i].data.end()){
      if ( (*localPointCloudIterator).clusterIDX != -1){ // if not noise
	(*localPointCloudIterator).clusterIDX = 
	  threadData[i].localMap.getParentID((*localPointCloudIterator).clusterIDX);
	//	threadData[0].localMap.getParentID((*localPointCloudIterator).clusterIDX);
      }
      localPointCloudIterator++;
    }
  }
  stop = rdtsc();
  std::cout << "fucking last phase: " << stop - start << std::endl;
*/

void* performDBSCAN(void* inputArg){

  barrier_cross(&barrier);
    
  unsigned long long start_duration = rdtsc();
  
  set_cpu(((threadArguments*)inputArg)->threadID+1);
    
  envMap *localMap = &(((threadArguments*)inputArg)->localMap);
  int offset = ((threadArguments*)inputArg)->offset;
  std::vector<idxXYZ>* inputData = &(((threadArguments*)inputArg)->data);
  int localThreadID = ((threadArguments*)inputArg)->threadID;
  int K = ((threadArguments*)inputArg)->K;

  std::vector<std::vector<int> > cluster_indices;

  
  unsigned long long start = rdtsc();

  DBSCAN_cluster_extraction_V2(inputData, cluster_indices, EPSILON, MinPts);
  //DBSCAN_cluster_extraction(inputData, cluster_indices, EPSILON, MinPts);
  
  unsigned long long stop = rdtsc();

  int j = 1;
  void* dummy;

  for (std::vector<std::vector<int> >::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    Stone* ellipsoid = new Stone(offset + j);
    ellipsoid->offset_signature = offset;
    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit){
      (*inputData)[*pit].clusterIDX = offset + j;
      ellipsoid->addMember((*inputData)[*pit]); 
    }
    dummy = (*ellipsoid).getCharacteristicMatrix();
    localMap->insertStone(*ellipsoid);
    j++;
  } //end for it = cluster_indices.begin() ...

  ((threadArguments*)inputArg)->numberOfClusters = j - 1;
  ((threadArguments*)inputArg)->numberOfPoints = inputData->size();
  ((threadArguments*)inputArg)->cost_localPCL = stop - start;


  ((threadArguments*)inputArg)->cost_localCombine = 0;
  
  unsigned long long start_wait_and_combine = rdtsc();

  int whereAreMyChildren[K+1][K+1];
  for (int i = 0; i <= K; i++){
    for (int j = 0; j <= K; j++){
      whereAreMyChildren[i][j] = 0;
    }
  }
  /*
  for (int i = 0; i <= K - 1; i++){
    whereAreMyChildren[K][i] = 1;
    }*/

  
  for (int i = 0; i <= K; i++){
    if( 2*i + 1 <= K){
      whereAreMyChildren[i][2*i+1] = 1;
    }
    if( 2*i + 2 <= K){
      whereAreMyChildren[i][2*i+2] = 1;
    }
  }

  
  bool unprocessedChildExists;
  do{
    unprocessedChildExists = false;
    for(int index = 0; index <= K; index++){
      if (whereAreMyChildren[localThreadID][index] == 1){ //index is an unprocessed child

	if(threadData[index].mapReady == false){
	  unprocessedChildExists = true;
	}

	else{
	  unsigned long long start_combine = rdtsc();

	  std::string serialized = threadData[index].localMap.serializeTheMap();//  std::cout << serialized << std::endl;
	  envMap childCopy = envMap(serialized);

	  localMap->combineV7(threadData[index].localMap);
	  //localMap->combineV7(childCopy);


	  
	  unsigned long long stop_combine = rdtsc();

	  ((threadArguments*)inputArg)->cost_localCombine += stop_combine - start_combine;
	  
	  whereAreMyChildren[localThreadID][index] = 0; //remove index from the unprocessed list
	}
	
      }
    }
  } while(unprocessedChildExists);

  unsigned long long stop_wait_and_combine = rdtsc();

  ((threadArguments*)inputArg)->cost_waitingTime = stop_wait_and_combine - start_wait_and_combine -
    ((threadArguments*)inputArg)->cost_localCombine;

  ((threadArguments*)inputArg)->mapReady = true;

  unsigned long long stop_duration = rdtsc();
  
  ((threadArguments*)inputArg)->cost_duration = stop_duration - start_duration;
  
  pthread_exit(0);
}
