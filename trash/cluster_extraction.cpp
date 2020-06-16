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

unsigned long long PCLClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr , char*, bool );

#if defined(__i386__)

static __inline__ unsigned long long rdtsc(void)
{
  unsigned long long int x;
  __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
  return x;
}
#elif defined(__x86_64__)


static __inline__ unsigned long long rdtsc(void)
{
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

#elif defined(__powerpc__)


static __inline__ unsigned long long rdtsc(void)
{
  unsigned long long int result=0;
  unsigned long int upper, lower,tmp;
  __asm__ volatile(
		   "0:                  \n"
		   "\tmftbu   %0           \n"
		   "\tmftb    %1           \n"
		   "\tmftbu   %2           \n"
		   "\tcmpw    %2,%0        \n"
		   "\tbne     0b         \n"
		   : "=r"(upper),"=r"(lower),"=r"(tmp)
		   );
  result = upper;
  result = result<<32;
  result = result|lower;

  return(result);
}

#endif

std::vector<int> baseLineClustering;
pcl::PointCloud<pcl::PointXYZ>::Ptr baseLineCloud (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int> initialClustering;
std::vector<int> stoneClustering;

int totalNumberOfStones = 0;

int main (int argc, char** argv)
{

  /* ./cluster_extraction input1 input2 input3 ... inputK output */
  baseLineCloud->height = 1;
  baseLineCloud->is_dense = false;

  std::ofstream runningTimeStream;
  runningTimeStream.open("./stats/pclLocalTime.txt", std::ofstream::out | std::ofstream::app);

  std::ofstream ptCloudSizeStream;
  ptCloudSizeStream.open("./stats/ptCloudSize.txt", std::ofstream::out | std::ofstream::app);


  std::ofstream baselineStream;
  baselineStream.open("./stats/baselineTime.txt", std::ofstream::out | std::ofstream::app);



  for(int inputPtCloud = 1; inputPtCloud < argc-1; inputPtCloud++){

    // Read in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream ptCloudInFile(argv[inputPtCloud]);
    double x, y, z;
    int numberOfPoints = 0;

    while( ptCloudInFile >> x >> y >> z){
      cloud->points.push_back(pcl::PointXYZ(x,y,z));
      //      baseLineCloud->points.push_back(pcl::PointXYZ(x,y,z));

      ++numberOfPoints;
    }
    cloud->width = numberOfPoints; 
    cloud->height = 1;
    cloud->is_dense = false; 
    ptCloudInFile.close();
    //    std::cout << "PointCloud: " << argv[inputPtCloud] << " has: " << cloud->points.size () << " data points." << std::endl;

    ptCloudSizeStream << cloud->points.size() << "\t";

    char bufferLocal[200];
    sprintf(bufferLocal, "%s_local_", argv[inputPtCloud]);
    runningTimeStream << PCLClustering(cloud, bufferLocal, false) << "\t";


  } //end for inputPtCloud...


  runningTimeStream << std::endl;
  runningTimeStream.close();
  
  ptCloudSizeStream << std::endl;
  ptCloudSizeStream.close();



  char bufferLocal[200];
  sprintf(bufferLocal, "baseline/%s", argv[argc-1]);
  baselineStream << PCLClustering(baseLineCloud, bufferLocal, true);


  baselineStream << std::endl;
  baselineStream.close();


  assert(baseLineCloud->points.size()== baseLineClustering.size());
  assert(initialClustering.size()== baseLineClustering.size());


  std::vector<int>::iterator stone_iterator = stoneClustering.begin();

  assert(stoneClustering.size() == totalNumberOfStones);


  std::stringstream ss_stones;
  ss_stones  << "baseline/" << argv[argc-1] << ".stone";

  std::ofstream stoneWriter;
  stoneWriter.open(ss_stones.str().c_str());


  while(stone_iterator != stoneClustering.end()){
    stoneWriter << *stone_iterator << std::endl;

    assert(*stone_iterator != -1);
    stone_iterator++;
  }


  stoneWriter.close();

  return 0;
}


unsigned long long PCLClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char* characteristicName, bool baseline){

  unsigned long long returnValue;
  unsigned long long start = rdtsc();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.35);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (750000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  unsigned long long stop = rdtsc();

  returnValue = (stop-start);

  int j = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredPtCloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

	if(baseline){
	  baseLineClustering[*pit] = j;
	  
	  assert( initialClustering[*pit] -1 < stoneClustering.size() );

	  stoneClustering[initialClustering[*pit]-1] = j; 
	}
	
	else{
	  clusteredPtCloud->points.push_back (cloud->points[*pit]);
	  baseLineCloud->points.push_back(cloud->points[*pit]);

	  indices.push_back(j);

	  initialClustering.push_back(j+totalNumberOfStones);
	  baseLineClustering.push_back(-1);
	}

      }

      if(!baseline){

	clusteredPtCloud->width = clusteredPtCloud->points.size ();
	clusteredPtCloud->height = 1;
	clusteredPtCloud->is_dense = true;

	baseLineCloud->width = baseLineCloud->points.size();

	stoneClustering.push_back(-1);
      }

      j++;
  }//end for it = ...
  
  if(!baseline){
    totalNumberOfStones += cluster_indices.size();
  }


  if(baseline){
    std::vector<int>::iterator check_iter = baseLineClustering.begin();
    while(check_iter != baseLineClustering.end()){
      assert( (*check_iter) != -1 );
      check_iter++;
    }
  }

  std::stringstream ss_points, ss_indices;
  ss_points  << characteristicName << ".ptCloud";
  ss_indices << characteristicName << ".index";

  std::ofstream ptCloudOutFile;
  ptCloudOutFile.open(ss_points.str().c_str());

  std::ofstream indicesOutFile;
  indicesOutFile.open(ss_indices.str().c_str());


  if(baseline){
    {
      pcl::PointCloud<pcl::PointXYZ>::const_iterator it = baseLineCloud->points.begin();
      while(it != baseLineCloud->points.end()){
	ptCloudOutFile <<     (*it).x << " " <<     (*it).y << " " <<     (*it).z << std::endl;
	++it;
      }
      ptCloudOutFile.close();
    }
    {
      std::vector<int>::const_iterator it = baseLineClustering.begin();
      while (it != baseLineClustering.end()){
	indicesOutFile << (*it) << std::endl;
	++it;
      }
      indicesOutFile.close();
    }
  }

  
  else{
    {
      pcl::PointCloud<pcl::PointXYZ>::const_iterator it = clusteredPtCloud->points.begin();
      while(it != clusteredPtCloud->points.end()){
	ptCloudOutFile <<     (*it).x << " " <<     (*it).y << " " <<     (*it).z << std::endl;
	++it;
      }
      ptCloudOutFile.close();
    }
    {
      std::vector<int>::const_iterator it = indices.begin();
      while (it != indices.end()){
	indicesOutFile << (*it) << std::endl;
	++it;
      }
      indicesOutFile.close();
    }
  }

  return returnValue;
}
