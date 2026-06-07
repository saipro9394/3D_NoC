#!/bin/bash

multimedia_benchmark=("MWD" "VOPD" "DVOPD")
netrace_benchmark=( "bodytrack" )
random_benchmark=("graph3" )
# "blackscholes" "bodytrack" "canneal" "dedup" "ferret" "fluidanimate" "swaptions" "vips"
dx=8
dy=8
dz=4
NOC="${dx}x${dy}x${dz}"

# for data in "${random_benchmark[@]}"; do
#     cd HotSpot-7.0/standard
#     ./surya_run.sh ../applications/${data}.xml
#     echo "surya_run completed"
#     cd shreya_algorithm
#     # ./new.sh ../applications/${data}.xml
#     echo "shreya_run completed"
#     cd ../../../PAT-Noxim/PAT_Noxim/bin

#     ./surya_run.sh ../../../surya_results/random_benchmark/${NOC} ${dx} ${dy} ${dz}
#     # ./shreya_run.sh ../../../shreya_results/random_benchmark/${NOC} ${dx} ${dy} ${dz}
#     cd ../../..
# done

# Uncomment this section when you want to run the multimedia benchmark

# for data in "${multimedia_benchmark[@]}"; do
#     cd HotSpot-7.0/standard
#     ./surya_run.sh ../applications/multimedia_benchmark/${data}_task_graph.xml
#     cd shreya_algorithm
#     # ./new.sh ../applications/multimedia_benchmark/${data}_task_graph.xml
#     echo "surya_run completed"
#     cd ../../../PAT-Noxim/PAT_Noxim/bin

#     ./surya_run.sh ../../../surya_results/multimedia_benchmark/${data} ${dx} ${dy} ${dz}
#     # ./shreya_run.sh ../../../shreya_results/multimedia_benchmark/${data} ${dx} ${dy} ${dz}
#     cd ../../..
# done


# Uncomment this sectino when you want to run the netrace benchmark

for data in "${netrace_benchmark[@]}"; do
    cd HotSpot-7.0/standard
    ./surya_run.sh ../applications/netrace_benchmark/${data}.xml
    cd shreya_algorithm
    # ./new.sh ../applications/netrace_benchmark/${data}.xml
    cd ../../../PAT-Noxim/PAT_Noxim/bin

    ./surya_run.sh ../../../surya_results/netrace_benchmark/${data} ${dx} ${dy} ${dz}
    # ./shreya_run.sh ../../../shreya_results/netrace_benchmark/${data} ${dx} ${dy} ${dz}
    cd ../../..
done


