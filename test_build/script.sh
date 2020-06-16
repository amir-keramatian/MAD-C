#!/bin/bash

var=$(hostname)
LIDAR=${var:3}

MINUTE=$(date +"%M")
MINUTE=$((MINUTE+2))
MINUTE=$((MINUTE%60))

#numberOfNodes=7
#topology="binary"
#command="../generateConfigFile "${numberOfNodes}" "${topology}
#echo $command
#$command


for scene in 1 2 3 4 5 6 7 8 9 10 11
do
    command="python generatePermutations.py "$((LIDAR))
    echo $command
    $command

    input="permutations.txt"
    while IFS= read -r line
    do
	number=("$line")

	command="./cluster_extraction ../../ptClouds/load_"${LOAD}"_lidar_"${number}"_iter_"${iter}" "${MINUTE}	

	echo $command
	$command

	MINUTE=$((MINUTE+1))

	MINUTE=$((MINUTE%60))
	
	sleep .1
	
    done < "$input"

done
