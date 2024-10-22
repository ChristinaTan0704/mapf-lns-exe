#!/bin/bash

# Create output directory if it doesn't exist
rm -r output/nb_test
mkdir -p output/nb_test

# File containing the list of states
state_file="script/den520_700.txt"

# Loop through each state in the file
while IFS= read -r state; do
    # Loop through the different neighbor sizes
    for neighbor_size in 4 8 16 32; do
        state_basename=$(basename "$state" .json)
        
        # Extract the second last folder name from the state path
        second_last_folder=$(basename "$(dirname "$(dirname "$state")")")

        # Create the log file name based on the second last folder, state, and neighbor size
        log_file="output/nb_test/${second_last_folder}_${state_basename}_neighbor_${neighbor_size}.log"
        
        exe/lns-rule-based/rule-based-lns --destroyStrategy RandomWalkProb \
            --uniform_neighbor 0 --neighborSize $neighbor_size \
            --map data/map/den520d.map \
            --state $state \
            --agentNum 700 --maxIterations 100 \
            --cutoffTime 300 > $log_file 2>&1
        
        # Output the command to be run
        echo "exe/lns-rule-based/rule-based-lns --destroyStrategy RandomWalkProb \
            --uniform_neighbor 0 --neighborSize $neighbor_size \
            --map data/map/den520d.map \
            --state $state \
            --agentNum 700 --maxIterations 100 \
            --cutoffTime 300 > $log_file 2>&1"
    done
done < "$state_file"
