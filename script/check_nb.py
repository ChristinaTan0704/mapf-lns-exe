import os
import re
from collections import defaultdict


folder_path = "/local-scratchb/jiaqit/exp/mapf-lns-benchmark/output/nb_adp"


tag_nb_size = defaultdict()

for one_file in os.listdir(folder_path):
    # data_map-den520d-scene-4-agent-900_neighbor_8.log
    scene_name = re.findall("scene-(\d+)", one_file)[0]
    nb_size = re.findall("neighbor_(\d+)", one_file)[0]
    file_path = os.path.join(folder_path, one_file)
    tag = one_file.split("_")[0] + "_" + scene_name
    
    # LNS(EECBS;PP): Iterations = 2913, solution cost = 152372, initial solution cost = 182265, lns_runtime = 300.094, group size = 15.7933, clipped_time = 0, failed iterations = 1874, sum_of_delay = 1950, num_agents  = 900, num_no_delay = 505
    with open(file_path, "r") as f:
        lines = f.readlines()
        result = lines[-1]
        delay = int(re.findall("sum_of_delay = (\d+)", result)[0])
    if tag not in tag_nb_size:
        tag_nb_size[tag] = dict()
    tag_nb_size[tag][nb_size] = delay
    

for tag, nb_size in tag_nb_size.items():
    nb_size = sorted(nb_size.items(), key=lambda x: int(x[0]))
    print("##########", tag)
    min_delay = 100000
    best_nb = None
    for nb, delay in nb_size:
        print(nb, delay)
        if delay < min_delay:
            min_delay = delay
            best_nb = nb
    print("best nb size: ", best_nb)