cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

#begining of the new stuff:
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
message(STATUS "**********************************Build type: ${CMAKE_BUILD_TYPE}")
#end of the new stuff



find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

#add_executable (cluster_extraction main_better.cc madcWrapper.cc madc_lib/stone.cc madc_lib/stone.cc madc_lib/map.cc kd_tree_lib/kdtree2.cpp dbscan_lib/dbscan_clusterer.cc)
add_executable (cluster_extraction main_edge_device_arbitrary_topology.cc madcWrapperEdge.cc madc_lib/stone.cc madc_lib/stone.cc madc_lib/map.cc kd_tree_lib/kdtree2.cpp tcpConnection.cc dbscan_lib/dbscan_clusterer.cc)
#add_executable (cluster_extraction main_baseline.cc baselineWrapper.cc madc_lib/stone.cc kd_tree_lib/kdtree2.cpp tcpConnection_baseline.cc dbscan_lib/dbscan_clusterer.cc)

target_link_libraries (cluster_extraction ${PCL_LIBRARIES} gsl gslcblas gomp)
