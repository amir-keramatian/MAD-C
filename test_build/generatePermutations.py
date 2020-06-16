
from itertools import permutations
import sys
import random


LIDAR=int(sys.argv[1])
#print LIDAR

MyFile=open('permutations.txt','w')

for element in samples:
    print >>MyFile, element[LIDAR-1]

    
MyFile.close()



#python -c 'from generatePermutations import generatePermutations; generatePermutations(LIDAR)'
