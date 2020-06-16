#!/bin/bash

LIDAR=1
MINUTE=39

for LOAD in 0.1 0.5 1
do
    command="python generatePermutations.py "$((LIDAR))
    echo $command
    $command

    input="permutations.txt"
    while IFS= read -r line
    do
	number=("$line")
	
	for ((iter=0;iter<10;iter++)); do

	    command="./cluster_extraction ../../ptClouds/load_"${LOAD}"_lidar_"${number}"_iter_"${iter}" "${MINUTE}	

	    echo $command
	    $command

	    MINUTE=$((MINUTE+1))

	    if [ $MINUTE -eq 60 ]; then
		MINUTE=0
	    fi
	    
	    sleep .1
	done
    done < "$input"
    rm permutations.txt
done
