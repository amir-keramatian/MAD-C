

index_madc = dlmread('madc_clustering.idx');

index_baseline = dlmread('baseline_clustering.idx');

accuracy = rand_index(index_madc, index_baseline);


dlmwrite('stats/accuracy.txt', accuracy, '-append')

%!rm madc_clustering.idx
%!rm baseline_clustering.idx
