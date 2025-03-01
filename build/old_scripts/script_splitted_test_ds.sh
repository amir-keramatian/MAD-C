#! /bin/bash

for K in 2 4 5 8 10 12 16 18 22 28 32 34 36 38 40 42 44 46 48 50 52 54 56 58 60 62 64 66 68 70 72
do
    for lidar in {3..3}
    do
	for iter in {0..0}
	do

	    command="./cluster_extraction"

	    for (( i=1; i<=$K; i++))
	    do
		item=" ../splited_dataset/load_0.5_lidar_"${lidar}"_iter_"${iter}"_seg_"${i}"_of_"${K}
		command=$command$item
	    done

	    $command
	    #octave-cli computeAccuracy.m
	    rm baseline_clustering.idx
	    rm madc_clustering.idx
	    
	done
    done
    mv stats/ellipsoid_forest_depths.txt stats/ellipsoid_forest_depths_${K}.txt
    
done
