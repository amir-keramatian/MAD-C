
from itertools import permutations
import sys
import random


LIDAR=int(sys.argv[1])
#print LIDAR

perm = list(permutations([0,1,2,3,4,5,6],5) )

number_of_samples = min(len(perm), 30);

random.seed(1)
samples = random.sample(perm, number_of_samples);

MyFile=open('permutations.txt','w')

for element in samples:
    print >>MyFile, element[LIDAR-1]

    
MyFile.close()



#python -c 'from generatePermutations import generatePermutations; generatePermutations(LIDAR)'
