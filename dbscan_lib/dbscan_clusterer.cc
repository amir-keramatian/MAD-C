#include "dbscan_clusterer.hpp"

void DBSCAN_cluster_extraction(std::vector<idxXYZ>* inputData,
			       std::vector<std::vector<int> > & clustering,
			       double eps, int minPts){

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

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  std::vector<bool> processed(cloud->size(), false);
  std::vector<bool> touched(cloud->size(), false);
  
  pcl::PointCloud<pcl::PointXYZ>::iterator pointCloudIterator;
  int j = 0;

  std::vector<int> neighboursToGetExplored;
  neighboursToGetExplored.reserve(numberOfPoints);
  
  for (pointCloudIterator = cloud->begin(); pointCloudIterator != cloud->end(); ++pointCloudIterator){
    int locationIndex = pointCloudIterator - cloud->begin();
    if ( processed[locationIndex] == false){

      std::vector<float> pointRadiusSquaredDistance;
      neighboursToGetExplored.clear();
      kdtree.radiusSearch(*pointCloudIterator, eps, neighboursToGetExplored, pointRadiusSquaredDistance);
      processed[locationIndex] = true; touched[locationIndex] = true;
      if (neighboursToGetExplored.size() >= minPts)
	{ // core-point: the beginning of a cluster
	  ++j;
	  std::vector<int> thisClusterIndices;
	  
	  (*inputData)[locationIndex].clusterIDX = j; thisClusterIndices.push_back(locationIndex);

	  while (!neighboursToGetExplored.empty()){
	    int index_of_neighbour = neighboursToGetExplored.back();
	    neighboursToGetExplored.pop_back();

	    if (processed[index_of_neighbour] == true)
	      {
		assert(	    (*inputData)[index_of_neighbour].clusterIDX != -1); //make sure it's not uninitialized
		/*point at index_of_neighbour is either noise or edge.
		  In either case no need to explore its neighbours. 
		  If it's edge, it is already been clustered (in another segement) because the processed flag is set to true*/
		/* if previously it was clustered as noise, now its label changes: */

		if ((*inputData)[index_of_neighbour].clusterIDX == NOISE){
		  (*inputData)[index_of_neighbour].clusterIDX = j;
		  thisClusterIndices.push_back(index_of_neighbour);
		}
		continue;
	      }
	    processed[index_of_neighbour] = true;
	    touched[index_of_neighbour] = true;
	    std::vector<int> pointIdxRadiusSearch;
	    kdtree.radiusSearch(*(cloud->begin() + index_of_neighbour), eps, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	    if (pointIdxRadiusSearch.size() >= minPts) // this neighbour is a core point by itself. Therefore, its neighbours have to be explored
	      {
		while(!pointIdxRadiusSearch.empty()){
		  int kirdex_of_neighbour = pointIdxRadiusSearch.back();
		  pointIdxRadiusSearch.pop_back();

		  if (touched[kirdex_of_neighbour] == false){
		    neighboursToGetExplored.push_back(kirdex_of_neighbour);
		    touched[kirdex_of_neighbour] = true;
		  }
		  if ((*inputData)[kirdex_of_neighbour].clusterIDX == NOISE){
		    (*inputData)[kirdex_of_neighbour].clusterIDX = j;
		    thisClusterIndices.push_back(kirdex_of_neighbour);
		  }
		}

		/*
		neighboursToGetExplored.insert(
					       neighboursToGetExplored.end(),
					       pointIdxRadiusSearch.begin(),
					       pointIdxRadiusSearch.end()
					       );*/
	      }
	    (*inputData)[index_of_neighbour].clusterIDX = j;
	    thisClusterIndices.push_back(index_of_neighbour);
	  }
	  clustering.push_back(thisClusterIndices);
	}// end core-point:...
      else //if it's a non-core point
	{
	  (*inputData)[locationIndex].clusterIDX = NOISE; //temporarily indicate it as noise. it might change if the same point is discovered later
	}
    } //end if processed[locationIndex] == false
  }//end for all point in cloud
  //  std::cout << cloud->size() << std::endl;
  return;
}

void DBSCAN_cluster_extraction_V2(std::vector<idxXYZ>* inputData,
			       std::vector<std::vector<int> > & clustering,
			       double eps, int minPts){

  Points* 	m_pts = new Points;
  m_pts->m_i_dims = 3;
  m_pts->m_i_num_points = inputData->size();

  m_pts->m_points.resize(inputData->size());

  for (int ll = 0; ll < inputData->size(); ll++){
    m_pts->m_points[ll].resize(3);

    m_pts->m_points[ll][0] = (*inputData)[ll].x;
    m_pts->m_points[ll][1] = (*inputData)[ll].y;
    m_pts->m_points[ll][2] = (*inputData)[ll].z;
  }

  kdtree2* 	m_kdtree;

  m_kdtree = new kdtree2(m_pts->m_points, false);

  if(m_kdtree == NULL)
    {
      std::cout << "Falied to allocate new kd tree" << std::endl;
      return ;
    }

  std::vector<bool> processed(inputData->size(), false);
  std::vector<bool> touched(inputData->size(), false); //1

  int j = 0;

  kdtree2_result_vector neighboursToGetExplored;
  kdtree2_result_vector ne2;
  neighboursToGetExplored.reserve(inputData->size());
  ne2.reserve(inputData->size());

  std::vector<int>* ind = m_kdtree->getIndex();
  
  int locationIndex;
  for(int bigLoop = 0; bigLoop < m_pts->m_i_num_points; bigLoop++){

    locationIndex = (*ind)[bigLoop];
    
    if ( processed[locationIndex] == false){

      neighboursToGetExplored.clear();
      m_kdtree->r_nearest_around_point(locationIndex, 0, eps*eps, neighboursToGetExplored);
      //      kdtree.radiusSearch(*pointCloudIterator, eps, neighboursToGetExplored, pointRadiusSquaredDistance);
      processed[locationIndex] = true; touched[locationIndex] = true;

      if (neighboursToGetExplored.size() >= minPts)
	{ // core-point: the beginning of a cluster
	  ++j;
	  std::vector<int> thisClusterIndices;
	  
	  (*inputData)[locationIndex].clusterIDX = j; thisClusterIndices.push_back(locationIndex);

	  for (int jj = 0; jj < neighboursToGetExplored.size(); jj++){

	    int index_of_neighbour = neighboursToGetExplored[jj].idx;

	    if (processed[index_of_neighbour] == true)
	      {
		assert(	    (*inputData)[index_of_neighbour].clusterIDX != -1); //make sure it's not uninitialized
		/*point at index_of_neighbour is either noise or edge.
		  In either case no need to explore its neighbours. 
		  If it's edge, it is already been clustered (in another segement) because the processed flag is set to true*/
		/* if previously it was clustered as noise, now its label changes: */

		if ((*inputData)[index_of_neighbour].clusterIDX == NOISE){
		  (*inputData)[index_of_neighbour].clusterIDX = j;
		  thisClusterIndices.push_back(index_of_neighbour);
		}
		continue;
	      }
	    processed[index_of_neighbour] = true;
	    touched[index_of_neighbour] = true;
	    std::vector<int> pointIdxRadiusSearch;
	    //	    kdtree.radiusSearch(*(cloud->begin() + index_of_neighbour), eps, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	    ne2.clear();
	    m_kdtree->r_nearest_around_point(index_of_neighbour, 0, eps*eps, ne2);
	    
	    if (ne2.size() >= minPts) // this neighbour is a core point by itself. Therefore, its neighbours have to be explored
	      {

		for (int jjj = 0; jjj < ne2.size(); jjj++){
		  int kirdex_of_neighbour = ne2[jjj].idx;
		  
		  if (touched[kirdex_of_neighbour] == false){
		    neighboursToGetExplored.push_back(ne2[jjj]);
		    touched[kirdex_of_neighbour] = true;
		  }
		  if ((*inputData)[kirdex_of_neighbour].clusterIDX == NOISE){
		    (*inputData)[kirdex_of_neighbour].clusterIDX = j;
		    thisClusterIndices.push_back(kirdex_of_neighbour);
		  }
		}
		/*neighboursToGetExplored.insert(
					       neighboursToGetExplored.end(),
					       ne2.begin(),
					       ne2.end()
					       );*/
	      }
	    (*inputData)[index_of_neighbour].clusterIDX = j;
	    thisClusterIndices.push_back(index_of_neighbour);
	  }
	  clustering.push_back(thisClusterIndices);
	}// end core-point:...
      else //if it's a non-core point
	{
	  (*inputData)[locationIndex].clusterIDX = NOISE; //temporarily indicate it as noise. it might change if the same point is discovered later
	}
    } //end if processed[locationIndex] == false
  }//end for all point in cloud
  
}
