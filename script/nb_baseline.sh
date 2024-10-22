#!/bin/bash

# Check if the number of processes is passed as an argument
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <NUM_CORES>"
  exit 1
fi

# Get the number of cores from the input argument
NUM_CORES=$1

# Remove and recreate the output directory
rm -r output/nb_baseline
mkdir -p output/nb_baseline

# Create a temporary file to store the list of commands
cmd_file="output/commands.txt"
rm -f $cmd_file
touch $cmd_file

# Define arrays for all parameters
agent_num=(500 350 350 600 900 750)
map_name=("empty-32-32" "random-32-32-20" "warehouse-10-20-10-2-1" "ost003d" "den520d" "Paris_1_256")
scene_num=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25)
uniform_neighbor_neighborSize=("1 4" "0 4" "0 8" "0 16" "0 32" "3 4")

# Loop through all combinations of parameters and add commands to the list
for map_idx in {1..6}; do
    for scene in "${scene_num[@]}"; do
        for neigh_pair in "${uniform_neighbor_neighborSize[@]}"; do
            neighbor=$(echo $neigh_pair | cut -d' ' -f1)
            size=$(echo $neigh_pair | cut -d' ' -f2)
            agent_count=${agent_num[$map_idx]}
            one_map_name=${map_name[$map_idx]}
            map_path="data/map/${one_map_name}.map"
            state="data/lns2_init_states/map-${one_map_name}-scene-${scene}-agent-${agent_count}.json"

            # Create log file name based on parameters
            state_basename=$(basename "$state" .json)
            log_file="output/nb_baseline/${state_basename}_NBopt_${neighbor}_NBsize_${size}.log"

            # Construct the command string and append it to the cmd_file
            echo "./rule-based-lns --destroyStrategy RandomWalkProb \
                --uniform_neighbor $neighbor --neighborSize $size \
                --map ${map_path} \
                --state $state \
                --agentNum $agent_count --maxIterations 100 \
                --cutoffTime 300 > $log_file 2>&1" >> $cmd_file
        done
    done
done


echo "commands are generated in $cmd_file"
# Run all the commands in parallel using GNU parallel
parallel -j $NUM_CORES < $cmd_file

# Clean up the command file if needed
rm -f $cmd_file
