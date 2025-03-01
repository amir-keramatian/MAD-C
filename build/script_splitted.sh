#! /bin/bash

#for K in 2 4 5 8 10 12 16 18 22 28 32 34 36 38 40 42 44 46 48 50 52 54 56 58 60 62 64 66 68 70 72
for K in 2 5 10 15 20 30 36 40 50 60 70
do
    for lidar in {0..6}
    do
	for iter in {0..9}
	do

	    command="./cluster_extraction"

	    for (( i=1; i<=$K; i++))
	    do
		
		item=" ../../synthetic_data/syn_10/load_0.1_lidar_"${lidar}"_iter_"${iter}"_seg_"${i}"_of_"${K}
		#item=" ../../synthetic_data/syn_50/load_0.5_lidar_"${lidar}"_iter_"${iter}"_seg_"${i}"_of_"${K}
		#item=" ../../synthetic_data/syn_100/load_1_lidar_"${lidar}"_iter_"${iter}"_seg_"${i}"_of_"${K}
		command=$command$item
	    done

	    $command
	    octave-cli computeAccuracy.m
	    rm baseline_clustering.idx
	    rm madc_clustering.idx
	    
	done
    done
    
done


#		printf "%d " $i
#	    printf "\n"
