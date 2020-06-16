#! /bin/bash

for K in 2 5 10 15 20 30 36 40 50 60 70
do
#    for scan in {1000..1001} #{1000..1043}
 #   do
        scan=1010
	command="./cluster_extraction"



	item=" ../../kitti/scan"${scan}" "${K}
	command=$command$item


	$command
	octave-cli computeAccuracy.m
	rm baseline_clustering.idx
	rm madc_clustering.idx
	

  #  done
    
done
