#include "madcWrapperEdge.h"
extern threadArguments_t *threadData;

//extern envMap* volatile leftChildsMap;
//extern envMap* volatile rightChildsMap;
extern envMap* volatile thisDevicesMap;

extern envMap* volatile childrensMaps[NUMBER_OF_CHILDREN];

extern double lc; extern double bw; extern double co;
double start_omp, stop_omp;

void* performEuclideanClustering_edge(void* inputArg){
  //  set_cpu(4);

  start_omp = omp_get_wtime();
  
  double start_duration = omp_get_wtime();
  
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

  
  double start = omp_get_wtime();
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
  double stop = omp_get_wtime();

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

  double stop_duration = omp_get_wtime();

  ((threadArguments*)inputArg)->cost_duration = stop_duration - start_duration;

  stop_omp = omp_get_wtime();
  lc = stop_omp - start_omp;
  
  performCombine(localMap);
  
  pthread_exit(0);
}

void* performDBSCAN_edge(void* inputArg){
  //  set_cpu(4);
  
  start_omp = omp_get_wtime();
  
  double start_duration = omp_get_wtime();
  
  envMap *localMap = &(((threadArguments*)inputArg)->localMap);
  int offset = ((threadArguments*)inputArg)->offset;
  std::vector<idxXYZ>* inputData = &(((threadArguments*)inputArg)->data);
  int localThreadID = ((threadArguments*)inputArg)->threadID;
  int K = ((threadArguments*)inputArg)->K;

  std::vector<std::vector<int> > cluster_indices;

  
  double start = omp_get_wtime();

  DBSCAN_cluster_extraction_V2(inputData, cluster_indices, EPSILON, MinPts);
  //DBSCAN_cluster_extraction(inputData, cluster_indices, EPSILON, MinPts);
  
  double stop = omp_get_wtime();

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

  double stop_duration = omp_get_wtime();
  
  ((threadArguments*)inputArg)->cost_duration = stop_duration - start_duration;


  stop_omp = omp_get_wtime();
  lc = stop_omp - start_omp;
  
  performCombine(localMap);
  
  pthread_exit(0);
}

void performCombine(envMap* parentsMap){
  double start_omp_total = omp_get_wtime();
  
  int whereAreMyChildren[NUMBER_OF_CHILDREN];
  for (int i = 0; i < NUMBER_OF_CHILDREN; i++){
    whereAreMyChildren[i] = 1;
  }

  bool unprocessedChildExists;

  do{
    unprocessedChildExists = false;
    
    for(int index = 0; index < NUMBER_OF_CHILDREN; index++){

      if (whereAreMyChildren[index] == 1){ //index is an unprocessed child

	if(childrensMaps[index] == NULL){
	  unprocessedChildExists = true;
	}
	else{
	  double start_omp_combine = omp_get_wtime();
	  parentsMap->combineV6(*childrensMaps[index]);
	  whereAreMyChildren[index] = 0;
	  double stop_omp_combine = omp_get_wtime();
	  co += stop_omp_combine - start_omp_combine;
	}
      }
    }
  } while(unprocessedChildExists);

  double stop_omp_total = omp_get_wtime();
  bw = stop_omp_total - start_omp_total - co;
  
  thisDevicesMap = parentsMap;
  
}

// void performCombine(envMap* parentsMap){

//   double start_duration = omp_get_wtime();  

//   bool wait_for_left_child = false;
//   bool wait_for_right_child = false;

// #ifdef leftChild
//   wait_for_left_child = true;
// #endif

// #ifdef rightChild
//   wait_for_right_child = true;
// #endif

//   while ( (wait_for_left_child == true) || (wait_for_right_child == true) ) { //KOS charkh bezan

//     if (  (wait_for_left_child == true) && (leftChildsMap != NULL)  ){

//       wait_for_left_child = false;

//       start_omp = omp_get_wtime();
//           parentsMap->combineV6(*leftChildsMap);
//       stop_omp = omp_get_wtime();

//       co += stop_omp - start_omp;
//     }

//     if (  (wait_for_right_child == true) && (rightChildsMap != NULL)  ){

//       wait_for_right_child = false;

//       start_omp = omp_get_wtime();
//           parentsMap->combineV6(*rightChildsMap);
//       stop_omp = omp_get_wtime();

//       co += stop_omp - start_omp;
//     }
//   } //END while
  
//   double stop_duration = omp_get_wtime();

//   bw = stop_duration - start_duration - co;

//   thisDevicesMap = parentsMap;
// }


// void performCombine(envMap* parentsMap){
// #ifdef leftChild 
//   start_omp = omp_get_wtime();
//   while (leftChildsMap == NULL)
//     ; //do nothing
//   stop_omp = omp_get_wtime();
//   bw = stop_omp - start_omp;

//   start_omp = omp_get_wtime();
//   parentsMap->combineV6(*leftChildsMap);
//   stop_omp = omp_get_wtime();
//   co = stop_omp - start_omp;
// #endif
  
// #ifdef rightChild
//   start_omp = omp_get_wtime();
//   while (rightChildsMap == NULL)
//     ; //do nothing
//   stop_omp = omp_get_wtime();
//   bw += stop_omp - start_omp;

//   start_omp = omp_get_wtime();
//   parentsMap->combineV6(*rightChildsMap);
//   stop_omp = omp_get_wtime();
//   co += stop_omp - start_omp;
// #endif

//   thisDevicesMap = parentsMap;
// }
