void envMap::combineV3(envMap& otherMap){ 
  std::vector<Stone*> thisMapObjectRepresentatives = getObjectRepresentatives();
  std::vector<Stone*>::iterator currentMapObjRepIter = thisMapObjectRepresentatives.begin();



  while(currentMapObjRepIter != thisMapObjectRepresentatives.end()){

    std::vector<Stone*> thatMapObjectRepresentatives = otherMap.getObjectRepresentatives(); //CONSIDER moving this line out of the inner loop

    std::vector<Stone*>::iterator otherMapObjRepIter   = thatMapObjectRepresentatives.begin();
    while(otherMapObjRepIter != thatMapObjectRepresentatives.end()){

      if ( (*currentMapObjRepIter)->findRoot() == (*otherMapObjRepIter)->findRoot()){
	;// do nothing
      }
      
      else if (Stone::compareTwoObjectRepresentatives(*currentMapObjRepIter, *otherMapObjRepIter)) {
	// 1. set the new parent pointer /
	// 2. expand the bounding box for the new parent /
	// 3. make a singular linked list /


	//	((*currentMapObjRepIter)->getId() < (*otherMapObjRepIter)->getId())?
	//	  (*otherMapObjRepIter)->setParent((*currentMapObjRepIter)):
	//	  ((*currentMapObjRepIter))->setParent((*otherMapObjRepIter)) ;

	// 1.
	//#@!	((*otherMapObjRepIter)->findRoot())->setParent((*currentMapObjRepIter)->findRoot()) ;
	((*currentMapObjRepIter)->findRoot())->setParent((*otherMapObjRepIter)->findRoot()) ;

	// 2.
	(*otherMapObjRepIter)->x_lower_bound =
	  my_MIN((*currentMapObjRepIter)->x_lower_bound, (*otherMapObjRepIter)->x_lower_bound);
	(*otherMapObjRepIter)->x_upper_bound =
	  my_MAX((*currentMapObjRepIter)->x_upper_bound, (*otherMapObjRepIter)->x_upper_bound);

	(*otherMapObjRepIter)->z_lower_bound =
	  my_MIN((*currentMapObjRepIter)->z_lower_bound, (*otherMapObjRepIter)->z_lower_bound);
	(*otherMapObjRepIter)->z_upper_bound =
	  my_MAX((*currentMapObjRepIter)->z_upper_bound, (*otherMapObjRepIter)->z_upper_bound);
	
	//3
	//swapping the next pointers
	Stone* temp = (*currentMapObjRepIter)->next;
	(*currentMapObjRepIter)->next = (*otherMapObjRepIter)->next;
	(*otherMapObjRepIter)->next = temp;
      }
      otherMapObjRepIter++;
    }
    currentMapObjRepIter++;
  }
  stonesInMap.insert(stonesInMap.end(), otherMap.stonesInMap.begin(), otherMap.stonesInMap.end());
}

void envMap::combine(envMap& otherMap){ 
    
  std::vector<Stone*>::iterator currentMapIterator = stonesInMap.begin();

  while(currentMapIterator != stonesInMap.end()){ //iterater over the current map
      
    std::vector<Stone*>::iterator otherMapIterator = otherMap.stonesInMap.begin();

    while(otherMapIterator != otherMap.stonesInMap.end()){ //iterate over otherMap

      Stone* stoneInCurrentMap = *currentMapIterator;
      Stone* stoneInOtherMap   = *otherMapIterator;

      Stone* lastParentOfStoneInCurrentMap = stoneInCurrentMap->findRoot();
      Stone* lastParentOfStoneInOtherMap   = stoneInOtherMap->findRoot();	

      
      if (lastParentOfStoneInCurrentMap == lastParentOfStoneInOtherMap){
	otherMapIterator++;
	continue;
      }

      totalNumberOfComparisons++;
      
      if (Stone::compareTwoStonesV2(*stoneInCurrentMap, *stoneInOtherMap)) {

	(lastParentOfStoneInCurrentMap->getId() < lastParentOfStoneInOtherMap->getId()) ?
	  lastParentOfStoneInOtherMap->setParent(lastParentOfStoneInCurrentMap) :
	  lastParentOfStoneInCurrentMap->setParent(lastParentOfStoneInOtherMap);
      }

      otherMapIterator++;
    }
    
    currentMapIterator++;
  }
  
  //now append theOtherMap to the working map (this map)
  stonesInMap.insert(stonesInMap.end(), otherMap.stonesInMap.begin(), otherMap.stonesInMap.end());
  // printMap();
}


void envMap::combineV2(envMap& otherMap){ 

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

	    Stone* lastParentOfStoneInCurrentMap = stoneInCurrentMap->findRoot();
	    Stone* lastParentOfStoneInOtherMap   = stoneInOtherMap->findRoot();	

	    totalNumberOfComparisons++;
	  
	    if (Stone::compareTwoStonesV2(*stoneInCurrentMap, *stoneInOtherMap)) {

	      (lastParentOfStoneInCurrentMap->getId() < lastParentOfStoneInOtherMap->getId()) ?
		lastParentOfStoneInOtherMap->setParent(lastParentOfStoneInCurrentMap) :
		lastParentOfStoneInCurrentMap->setParent(lastParentOfStoneInOtherMap);

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

void envMap::combineV4(envMap& otherMap){
  object_t thisObject, thatObject;
  resetTheFlags();


  thisObject = getNextObject_t();

  while(thisObject != NULL){

    otherMap.resetTheFlags();
    thatObject = otherMap.getNextObject_t();

    while(thatObject != NULL){
      
      if( thisObject->findRoot() == thatObject->findRoot()){
	; //do nothing
      }
      else if (Stone::compareTwoObjectRepresentatives(thisObject, thatObject)) {
	/* 1. set the new parent pointer */
	/* 2. expand the bounding box for the new parent */
	/* 3. make a singular linked list */

	/* 1. */
	(thisObject->findRoot())->setParent(thatObject->findRoot()) ;

	/* 2. */
	thatObject->findRoot()->x_lower_bound =
	  my_MIN(thisObject->findRoot()->x_lower_bound, thatObject->findRoot()->x_lower_bound);
	thatObject->x_upper_bound =
	  my_MAX(thisObject->findRoot()->x_upper_bound, thatObject->findRoot()->x_upper_bound);

	thatObject->z_lower_bound =
	  my_MIN(thisObject->findRoot()->z_lower_bound, thatObject->findRoot()->z_lower_bound);
	thatObject->z_upper_bound =
	  my_MAX(thisObject->findRoot()->z_upper_bound, thatObject->findRoot()->z_upper_bound);

	
	/* 3. */
	/*swapping the next pointers*/
	Stone* temp = (thisObject)->next;
	thisObject->next = thatObject->next;
	thatObject->next = temp;
      }

      thatObject = otherMap.getNextObject_t();
    }
    
    thisObject = getNextObject_t();
  }

  stonesInMap.insert(stonesInMap.end(), otherMap.stonesInMap.begin(), otherMap.stonesInMap.end());
}

void envMap::combineV5(envMap& otherMap){
  std::vector<object_t> thisMapObjects = getObjects_t();
  std::vector<object_t>::iterator thisMapObjectIter = thisMapObjects.begin();


  while(thisMapObjectIter != thisMapObjects.end()){

    std::vector<object_t> thatMapObjects = otherMap.getObjects_t();
    std::vector<object_t>::iterator thatMapObjectIter = thatMapObjects.begin();

    while(thatMapObjectIter != thatMapObjects.end()){
      if( (*thisMapObjectIter)->findRoot() == (*thatMapObjectIter)->findRoot()   ){
	; // do nothing
      }
      else if (Stone::compareTwoObjectRepresentatives((*thisMapObjectIter), (*thatMapObjectIter))) {
	/* 1. set the new parent pointer */
	/* 2. expand the bounding box for the new parent */
	/* 3. make a singular linked list */

	/* 1. */
	((*thisMapObjectIter)->findRoot())->setParent((*thatMapObjectIter)->findRoot()) ;

	/* 2. */ /* CURRENTLY MISSING */
	 
	
	/* 3. */
	/*swapping the next pointers*/
	Stone* temp = (*thisMapObjectIter)->next;
	(*thisMapObjectIter)->next = (*thatMapObjectIter)->next;
	(*thatMapObjectIter)->next = temp;
      }
      thatMapObjectIter++;
    }
    thisMapObjectIter++;
  }
  stonesInMap.insert(stonesInMap.end(), otherMap.stonesInMap.begin(), otherMap.stonesInMap.end());
}
