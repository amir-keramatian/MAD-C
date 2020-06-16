#ifndef MAP_H
#define MAP_H
#include <vector>
#include <iostream>
#include <stdio.h>

class Stone;
typedef struct{
  std::vector<Stone*> ellipsoids;

  double x_lower_bound;
  double x_upper_bound;

  double z_lower_bound;
  double z_upper_bound;

} object;

typedef Stone* object_t;


class envMap{

 public:

  envMap(std::string);
  envMap();
  
  std::vector<Stone*> stonesInMap;
  
  //std::vector<Stone> getStones();

  void insertStone(Stone&);

  std::vector<int> depth_tree;

  /*
  void combine(envMap&);
  void combineV2(envMap&);
  void combineV3(envMap&);
  void combineV4(envMap&);
  void combineV5(envMap&);
  */
  void combineV6(envMap&);
  void combineV7(envMap&);

  void printMap();

  void doNop(envMap&);

  int getParentID(int );
  
  std::vector<object> getObjects();
  std::vector<Stone*> getObjectRepresentatives();

  /********************************************************************************************************/
  
  void resetTheFlags();
  object_t getNextObject_t();
  std::vector<object_t> getObjects_t();
  std::vector<object_t>::iterator object_t_iterator;// = stonesInMap.begin();  
  std::string serializeTheMap();

};

#endif
