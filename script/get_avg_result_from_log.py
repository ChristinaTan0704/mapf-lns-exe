import argparse
import re
import os

# time_threshold = 300
# log_folder = "output/nb_baseline"
# log_key_word = ["empty-32-32", "agent-500", "NBopt_1_NBsize_4"]

# python script/get_avg_result_from_log.py --log_folder output/nb_baseline --log_key_word empty-32-32 agent-500 NBopt_1_NBsize_4 --time_threshold 300

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_folder", type=str, default="output/nb_baseline")
    parser.add_argument("--log_key_word", type=str, nargs="+", default=["den520d", "agent-900", "NBopt_1_NBsize_4"])
    parser.add_argument("--time_threshold", type=int, default=300)
    args = parser.parse_args()

    log_folder = args.log_folder
    log_key_word = args.log_key_word
    time_threshold = args.time_threshold

    log_files = os.listdir(log_folder)
    log_files = [f for f in log_files if all([k in f for k in log_key_word])]

    avg_results = []
    for log_file in log_files:

        with open(os.path.join(log_folder, log_file), "r") as f:
            lines = f.readlines()
            if "lns_runtime =" in lines[-1]:
                # LNS(EECBS;PP): Iterations = 3106, solution cost = 156817, initial solution cost = 186498, lns_runtime = 300, group size = 14.4267, clipped_time = 0, failed iterations = 2052, sum_of_delay = 1416, num_agents  = 900, num_no_delay = 552
                sum_of_delay = int(re.findall("sum_of_delay = (\d+)", lines[-1])[0])
                avg_results.append(sum_of_delay)

            # for line in lines:
            #     import pdb; pdb.set_trace()

            #     if "lns_runtime :" not in line or "sum_of_delay" not in line:
            #         continue
            #     lns_runtime_pattern = r"lns_runtime\s*:\s*([\d.]+)"
            #     sum_of_delay_pattern = r"sum_of_delay\s*=\s*(\d+)"
            #     # Search for lns_runtime
            #     try:
            #         lns_runtime_match = re.search(lns_runtime_pattern, line)
            #         lns_runtime = lns_runtime_match.group(1) 
            #         lns_runtime = float(lns_runtime)
            #     except:
            #         import pdb; pdb.set_trace()
            #     # lns_runtime_match = re.search(lns_runtime_pattern, line)
            #     # lns_runtime = lns_runtime_match.group(1) 
            #     # lns_runtime = float(lns_runtime)

            #     # # Search for sum_of_delay (it might not be present)
            #     print("lns_runtime: ", lns_runtime)
            #     if abs(lns_runtime - args.time_threshold) < 0.1:
                    
            #         sum_of_delay_match = re.search(sum_of_delay_pattern, line)
            #         sum_of_delay = sum_of_delay_match.group(1) 
            #         sum_of_delay = float(sum_of_delay)
            #         avg_results.append(sum_of_delay)
            #         break
    print("Results: ", avg_results) 
    print("Average result: ", sum(avg_results) / len(avg_results))
    print("Number of results: ", len(avg_results))