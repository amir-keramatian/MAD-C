import sys
from array import *

T=[
[3,4,12,9,13,7,5,11,16,15,2,19,18,14,8,17,6,20,10,1],
[14,7,6,11,16,20,1,18,15,4,13,19,8,5,12,3,2,9,17,10],
[18,14,7,16,10,8,2,15,6,12,1,3,4,5,19,20,11,9,17,13],
[2,11,15,10,8,4,9,18,3,20,6,19,14,13,17,5,16,1,12,7],
[7,1,20,15,3,4,2,5,18,12,14,17,19,6,11,10,16,13,8,9],
[9,10,5,1,11,19,6,17,7,15,16,4,14,12,20,13,18,8,2,3],
[5,1,11,20,2,17,9,10,8,12,13,19,3,16,7,4,15,18,14,6],
[13,19,11,6,12,14,9,7,16,18,15,8,3,5,4,10,2,20,17,1],
[19,16,3,11,10,4,7,17,12,13,5,14,2,15,6,8,9,18,20,1],
[16,20,9,4,14,13,5,2,10,8,7,3,11,12,1,19,17,18,6,15]
]

LIDAR=int(sys.argv[1])
#print LIDAR

MyFile=open('permutations.txt','w')


for element in T:
    print >>MyFile, element[LIDAR-1]

    
MyFile.close()
