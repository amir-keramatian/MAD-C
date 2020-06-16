lidar=1
iter=2
for K in 32 
do

    command="./cluster_extraction"

    for (( i=1; i<=$K; i++))
    do
	item=" ../splited_dataset/load_0.5_lidar_"${lidar}"_iter_"${iter}"_seg_"${i}"_of_"${K}
	command=$command$item
    done

    $command
    octave-cli computeAccuracy.m
#    rm baseline_clustering.idx
#    rm madc_clustering.idx
    
done
