#include "baselineWrapper.h"


extern double lc; extern double bw; extern double co;
double start_omp, stop_omp;

void* performEuclideanClustering_baseline(void* inputArg){
  set_cpu(4);

  start_omp = omp_get_wtime();
  
  double start_duration = omp_get_wtime();
  
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

  double stop_duration = omp_get_wtime();

  ((threadArguments*)inputArg)->cost_duration = stop_duration - start_duration;

  stop_omp = omp_get_wtime();
  lc = stop_omp - start_omp;
  
  pthread_exit(0);
}

void* performDBSCAN_baseline(void* inputArg){
  set_cpu(4);
  
  start_omp = omp_get_wtime();
  
  double start_duration = omp_get_wtime();
  
  int offset = ((threadArguments*)inputArg)->offset;
  std::vector<idxXYZ>* inputData = &(((threadArguments*)inputArg)->data);
  int localThreadID = ((threadArguments*)inputArg)->threadID;
  int K = ((threadArguments*)inputArg)->K;

  std::vector<std::vector<int> > cluster_indices;

  
  double start = omp_get_wtime();

  DBSCAN_cluster_extraction_V2(inputData, cluster_indices, EPSILON, MinPts);
  //DBSCAN_cluster_extraction(inputData, cluster_indices, EPSILON, MinPts);
  
  double stop = omp_get_wtime();

  double stop_duration = omp_get_wtime();
  
  ((threadArguments*)inputArg)->cost_duration = stop_duration - start_duration;


  stop_omp = omp_get_wtime();
  lc = stop_omp - start_omp;
  
  pthread_exit(0);
}
