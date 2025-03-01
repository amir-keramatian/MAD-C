#include "map.h"
#include "stone.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "idxXYZ.h"

extern std::vector<idxXYZ> data;
extern int totalNumberOfComparisons;

void envMap::insertStone(Stone& toBeInserted){
  stonesInMap.push_back(&toBeInserted);
  if (toBeInserted.clean_mu_cov == false){
    std::cout << "Ellipsoid not finalized" << std::endl;
  }
}

void envMap::doNop(envMap& otherMap){

  return ;
}


void envMap::printMap(){
  //TODO
  /* get objects and display them by ellipsoids */
}

std::vector<object> envMap::getObjects(){
  int sumOfDepths, depth;
  std::vector<object> result;
  
  std::vector<Stone*>::iterator outerIterator = stonesInMap.begin();

  while(outerIterator != stonesInMap.end()){
    (*outerIterator)->processed = false;
    outerIterator++;
  }
  outerIterator = stonesInMap.begin();


  while(outerIterator != stonesInMap.end()){

    std::vector<Stone*>::iterator innerIterator = outerIterator;
    bool needsNewLine = false;
    object obj;
    obj.x_lower_bound = INT_MAX; obj.x_upper_bound = -INT_MAX;
    obj.z_lower_bound = INT_MAX; obj.z_upper_bound = -INT_MAX;
    
    //?    int id = (*outerIterator)->findRoot(depth)->getId();
    //?    depth_tree.push_back(depth);
    
    while(innerIterator != stonesInMap.end()){

      if (
	  ( (*innerIterator)->processed == 0 )
	  &&
	  (Stone::areTwoStonesLinked( (*innerIterator), (*outerIterator), &depth_tree ) == true)
	  ){
	//	std::cout << (*innerIterator)->getId() << " ";
	(*innerIterator)->processed = 1;
	needsNewLine = true;
	obj.ellipsoids.push_back(*innerIterator);

	obj.x_lower_bound = my_MIN(obj.x_lower_bound, (*innerIterator)->x_lower_bound);
	obj.x_upper_bound = my_MAX(obj.x_upper_bound, (*innerIterator)->x_upper_bound);

	obj.z_lower_bound = my_MIN(obj.z_lower_bound, (*innerIterator)->z_lower_bound);
	obj.z_upper_bound = my_MAX(obj.z_upper_bound, (*innerIterator)->z_upper_bound);

      }
      innerIterator++;
    }
    
    if (needsNewLine) {

      /*obj.x_lower_bound -= epsilon/2;
      obj.x_upper_bound += epsilon/2;
      
      obj.z_lower_bound -= epsilon/2;
      obj.z_upper_bound += epsilon/2;*/

      result.push_back(obj);
    }
    outerIterator++;
  }

  return result;
}

int envMap::getParentID(int id){
  int depth;
  int parentID = -1;
  std::vector<Stone*>::iterator currentMapIterator;

  for( currentMapIterator = stonesInMap.begin(); currentMapIterator != stonesInMap.end(); currentMapIterator++){
    
    if ( (*currentMapIterator)->getId() == id){
      parentID = (*currentMapIterator)->findRoot(depth)->getId();
    }
  }

  if (parentID == -1) {
    std::cout << "error at getParentID. Something's fucked up!" << std::endl;
  }

  return parentID;

}

void envMap::resetTheFlags(){
  std::vector<Stone*>::iterator ellipsoidIterator = stonesInMap.begin();

  while(ellipsoidIterator != stonesInMap.end()){
    (*ellipsoidIterator)->processed = false;
    ellipsoidIterator++;
  }
  object_t_iterator = stonesInMap.begin();
}

std::vector<object_t> envMap::getObjects_t(){
  resetTheFlags();
  std::vector<object_t> result;

  object_t nextObject = getNextObject_t();
  while(nextObject != NULL){
    result.push_back(nextObject);
    nextObject = getNextObject_t();
  }
  
  return result;
}


object_t envMap::getNextObject_t(){
  int depth;
  object_t nextObject;
  if (object_t_iterator == stonesInMap.end()){
    return NULL;
  }

  depth = 0;
  nextObject = (*object_t_iterator)->findRoot(depth);
  //##depth_tree.push_back(depth);
  
  if (nextObject->processed == false){
    nextObject->processed = true;
    object_t_iterator++;
    return nextObject;
  }

  else{
    object_t_iterator++;
    return getNextObject_t();
  }
  
}


std::vector<Stone*> envMap::getObjectRepresentatives(){
  int depth;
  std::vector<Stone*> result;
  resetTheFlags();
  
  std::vector<Stone*>::iterator outerIterator = stonesInMap.begin();


  while(outerIterator != stonesInMap.end()){
    depth = 0;
    object_t nextObject = (*outerIterator)->findRoot(depth);
    //##depth_tree.push_back(depth);
    
    if (  nextObject->processed == false     ){
      nextObject->processed = true;
      result.push_back(nextObject);
    }
    outerIterator++;
  }
  return result;
}

void envMap::combineV6(envMap& otherMap){
  int sumOfDepths;
  std::vector<object> thisMapObjects = getObjects();
  std::vector<object> otherMapObjects = otherMap.getObjects();
  
  
  std::vector<object>::iterator currentMapObjectIterator = thisMapObjects.begin();

  while(currentMapObjectIterator != thisMapObjects.end()){ //iterater over objects in the current map


    std::vector<object>::iterator otherMapObjectIterator = otherMapObjects.begin();

    while(otherMapObjectIterator != otherMapObjects.end()){ //iterate over objects in the otherMap

      object thisObject = *currentMapObjectIterator;
      object thatObject = *otherMapObjectIterator;


      if ( (thisObject.x_upper_bound < thatObject.x_lower_bound) ||
	   (thatObject.x_upper_bound < thisObject.x_lower_bound) ){
	; //do nothing
      }
      
      else if ( (thisObject.z_upper_bound < thatObject.z_lower_bound) ||
		(thatObject.z_upper_bound < thisObject.z_lower_bound) ){
	; //do nothing
      }

      else{
	std::vector<Stone*>::iterator thisObjectsEllipsoids = thisObject.ellipsoids.begin();

	while(thisObjectsEllipsoids != thisObject.ellipsoids.end()){
	  std::vector<Stone*>::iterator thatObjectsEllipsoids = thatObject.ellipsoids.begin();
	  while(thatObjectsEllipsoids != thatObject.ellipsoids.end()){

	    Stone* stoneInCurrentMap = *thisObjectsEllipsoids;
	    Stone* stoneInOtherMap   = *thatObjectsEllipsoids;

	    totalNumberOfComparisons++;
	  
	    if (Stone::compareTwoStonesV2(*stoneInCurrentMap, *stoneInOtherMap)) {

	      Stone::linkTwoStones(stoneInCurrentMap, stoneInOtherMap, sumOfDepths);
	      //##depth_tree.push_back(sumOfDepths);
	      
	      thisObjectsEllipsoids = thisObject.ellipsoids.end() - 1;
	      thatObjectsEllipsoids = thatObject.ellipsoids.end() - 1;
	    }
	    thatObjectsEllipsoids++;
	  }
	  thisObjectsEllipsoids++;
	}
      } //END else
      
      otherMapObjectIterator++;
    } //END iterate over objects in the otherMap
    
    currentMapObjectIterator++;
  }
  
  //now append theOtherMap to the working map (this map)
  stonesInMap.insert(stonesInMap.end(), otherMap.stonesInMap.begin(), otherMap.stonesInMap.end());
}

void envMap::combineV7(envMap& otherMap){
  int sumOfDepths;
  std::vector<object_t> thisMapObjects = getObjects_t();
  std::vector<object_t>::iterator thisMapObjectIter = thisMapObjects.begin();

  std::vector<object_t> thatMapObjects = otherMap.getObjects_t();
  
  while(thisMapObjectIter != thisMapObjects.end()){

    std::vector<object_t>::iterator thatMapObjectIter = thatMapObjects.begin();

    while(thatMapObjectIter != thatMapObjects.end()){
      bool areTheObjectsLinked = Stone::areTwoStonesLinked( (*thisMapObjectIter), (*thatMapObjectIter), sumOfDepths);
      //##depth_tree.push_back(sumOfDepths);
      
      if( areTheObjectsLinked == true ) {
	; // do nothing
      }
      else if (Stone::compareTwoObjectRepresentatives((*thisMapObjectIter), (*thatMapObjectIter))) {
	/* 1. set the new parent pointer */
	/* 2. expand the bounding box for the new parent */
	/* 3. make a singular linked list */
	Stone::linkTwoStones((*thisMapObjectIter), (*thatMapObjectIter), sumOfDepths);
	//##depth_tree.push_back(sumOfDepths);
      }
      thatMapObjectIter++;
    }
    thisMapObjectIter++;
  }
  stonesInMap.insert(stonesInMap.end(), otherMap.stonesInMap.begin(), otherMap.stonesInMap.end());
}

std::string envMap::serializeTheMap(){
  std::string result;
  
  std::vector<object> thisMapObjects = getObjects();
  std::vector<object>::iterator currentMapObjectIterator = thisMapObjects.begin();

  std::ostringstream strs;

  strs << thisMapObjects.size() << ",";
  
  while(currentMapObjectIterator != thisMapObjects.end()){ //iterater over objects in the current map

    object thisObject = *currentMapObjectIterator;

    strs << "obj" << ",";

    strs << thisObject.ellipsoids.size() << ",";
    
    std::vector<Stone*>::iterator thisObjectsEllipsoids = thisObject.ellipsoids.begin();

    while(thisObjectsEllipsoids != thisObject.ellipsoids.end()){

      Stone* stoneInCurrentMap = *thisObjectsEllipsoids;
      strs << stoneInCurrentMap->serializeTheStone();
      
      thisObjectsEllipsoids++;
    }
    
    currentMapObjectIterator++;
  }

  result = strs.str();
  return result;
}

envMap::envMap(){}

envMap::envMap(std::string serializedMap){

  char *saveptr;
  char* token;

  token = strtok_r (const_cast<char*>(serializedMap.c_str()), ",", &saveptr);
  
  int numberOfObjects = atoi(token);
  //  std::cout << "Number of Objects: " << numberOfObjects << std::endl;

  
  for (int obj = 0; obj < numberOfObjects; obj++){

    Stone* parent_ellipsoid;
    
    token = strtok_r (NULL, ",",&saveptr);
    if (strcmp(token, "obj") != 0 ){
      std::cout << "assertion failed" << std::endl;
      exit(EXIT_FAILURE);
    }

    int numberOfEllipsoids = atoi(strtok_r (NULL, ",",&saveptr));

    // std::cout << "x_lower_bound = " << x_lower_bound << std::endl;
    // std::cout << "x_upper_bound = " << x_upper_bound << std::endl;
    // std::cout << "z_lower_bound = " << z_lower_bound << std::endl;
    // std::cout << "z_upper_bound = " << z_upper_bound << std::endl;

    // std::cout << "number of ellipsoids = " << numberOfEllipsoids << std::endl;
    
    
    for (int ellips = 0; ellips < numberOfEllipsoids; ellips++){

      int offset_signature = atoi(strtok_r (NULL, ",",&saveptr));
      int id = atoi(strtok_r (NULL, ",",&saveptr));

      //std::cout << "\t offset_signature= " << offset_signature << std::endl;
      //std::cout << "\t id = " << id << std::endl;
      
      gsl_matrix* characteristic_matrix = gsl_matrix_alloc(4,4);
      for (int i = 0; i < 4; ++i)
	{
	  for (int j = 0; j < 4; ++j)
	    {
	      gsl_matrix_set(characteristic_matrix, i, j, atof(strtok_r (NULL, ",",&saveptr)));
	    }
	}


      gsl_matrix* characteristic_matrix_inverse = gsl_matrix_alloc(4, 4);
      for (int i = 0; i < 4; ++i)
	{
	  for (int j = 0; j < 4; ++j)
	    {
	      gsl_matrix_set(characteristic_matrix_inverse, i, j, atof(strtok_r (NULL, ",",&saveptr)));
	    }
	}

      double x_lower_bound = atof(strtok_r (NULL, ",",&saveptr));
      double x_upper_bound = atof(strtok_r (NULL, ",",&saveptr));
      double z_lower_bound = atof(strtok_r (NULL, ",",&saveptr));
      double z_upper_bound = atof(strtok_r (NULL, ",",&saveptr));

      Stone* ellipsoid = new Stone(id);
      ellipsoid->offset_signature = offset_signature;
      ellipsoid->x_lower_bound = x_lower_bound;
      ellipsoid->x_upper_bound = x_upper_bound;
      ellipsoid->z_lower_bound = z_lower_bound;
      ellipsoid->z_upper_bound = z_upper_bound;

      if(ellips == 0){
	parent_ellipsoid = ellipsoid;
	//	parent_ellipsoid
      }

      ellipsoid->setParent(parent_ellipsoid);
      ellipsoid->characteristic_matrix = characteristic_matrix;
      ellipsoid->characteristic_matrix_inverse = characteristic_matrix_inverse;
      ellipsoid->clean_mu_cov = true;

      this->insertStone(*ellipsoid);
    }
  }



  
  // while(std::getline(iss, token, ',')) {
  //   std::cout << token << '\n';
  // } 
  
  //std::cout << std::endl;
}
