#!/bin/bash

var=$(hostname)

LIDAR=${var:3}

MINUTE=$(date +"%M")
MINUTE=$((MINUTE+2))
MINUTE=$((MINUTE%60))


for scene in 1 2 3 4 5 6 7 8 9 10 11
do
    command="python createPermutations.py "$((LIDAR))
    echo $command
    $command

    input="permutations.txt"
    while IFS= read -r line
    do
	number=("$line")

	command="./cluster_extraction /home/amir/ford_scenes/scene_"${scene}"_lidar_"${number}" "${MINUTE}

	echo $command
	$command

	MINUTE=$((MINUTE+1))

	MINUTE=$((MINUTE%60))
	
	sleep .1
	
    done < "$input"

done
