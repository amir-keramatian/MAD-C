#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <vector>
#include "config.h" //temporarily here. Will be moved
#include "madc_lib/idxXYZ.h"
#include "madc_lib/map.h"
#include "madc_lib/stone.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <fstream>
#include <iomanip>
#include "rdtsc.h"
#include "utils.h"

#include "kd_tree_lib/kdtree2.hpp"
#include "dbscan_lib/dbscan_clusterer.hpp"

#ifndef EUCLIDEANCLUSTERINGWRAPPER
#define EUCLIDEANCLUSTERINGWRAPPER

struct threadArguments{

  //  threadArguments*(

  threadArguments(){
    cost_localPCL = 0;
    cost_localCombine = 0;
    cost_waitingTime = 0;
    cost_duration = 0;
  }

  
  std::vector<idxXYZ> data;
  int offset;
  envMap localMap;

  int numberOfPoints;
  int numberOfClusters;
  
  unsigned long long cost_localPCL;
  unsigned long long cost_localCombine;
  unsigned long long cost_waitingTime;
  unsigned long long cost_duration;
  
  int threadID;
  int K; //max thread id value
  bool mapReady;

  //execution time, memory, ... 1001 koofto zahre mar
} typedef threadArguments_t;


void* performEuclideanClustering(void* );
void* performDBSCAN(void* );
void* lastPhase(void *);



template<typename T> void printFunction(T t, const int& width)
{
  std::cout << std::left << std::setw(width) << std::setfill(' ') << t;
}

#endif

/*
    for (int i = 0; i < numberOfThreads; i++){
      std::cout <<
	threadData[i].numberOfPoints << "\t\t" <<
	threadData[i].numberOfClusters << "\t\t" <<
	threadData[i].cost_localPCL << "\t\t" <<
	threadData[i].cost_localCombine << "\t\t" <<
	threadData[i].cost_waitingTime << "\t\t" <<
	threadData[i].cost_duration <<
	std::endl;
    }


*/
