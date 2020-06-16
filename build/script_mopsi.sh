#! /bin/bash

for K in 2 5 10 15 20 30 36 40 50 60 70
do

    command="./cluster_extraction"


    item=" ../../mopsi/duplicate_removed_mopsi_data_3D.txt "${K}
    command=$command$item


    $command
    #	octave-cli computeAccuracy.m
    #	rm baseline_clustering.idx
    mv madc_clustering.idx parma_mopsi_clustering.idx${K}
    


    
done

