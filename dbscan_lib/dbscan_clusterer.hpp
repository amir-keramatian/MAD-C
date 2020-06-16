#ifndef DBSCAN_CLUSTERER
#define DBSCAN_CLUSTERER

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../kd_tree_lib/kdtree2.hpp"
#include "../madc_lib/idxXYZ.h"
#include <vector>
#include <iostream>
#include "../config.h" //temporarily here. Will be moved


void DBSCAN_cluster_extraction(std::vector<idxXYZ>*, std::vector<std::vector<int> >&, double, int);
void DBSCAN_cluster_extraction_V2(std::vector<idxXYZ>*, std::vector<std::vector<int> >&, double, int);

#endif
