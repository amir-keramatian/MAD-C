#ifndef CONFIG_FILE_H_
#define CONFIG_FILE_H_

#define EPSILON 0.35
#define MinPts 10

#define performBaseline 1

#define PRINT_STD_OUT 0

#define DBSCAN 0
#define EUCLIDEAN (1-DBSCAN)



#define sortInputPointCloud 0








#if DBSCAN==1
#define NOISE -10
#elif EUCLIDEAN==1
#define NOISE -1
#endif

#endif
