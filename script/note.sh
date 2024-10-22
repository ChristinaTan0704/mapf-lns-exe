./rule-based-lns --destroyStrategy  RandomWalkProb  \
--uniform_neighbor 0 --neighborSize 32 \
--map random-32-32-20.map \
--agents random-32-32-20-random-1.scen \
--state map-random-32-32-20-scene-1-agent-150.json \
--agentNum 150 \
--cutoffTime 300


exe/lns-rule-based/rule-based-lns --destroyStrategy  RandomWalkProb  \
--uniform_neighbor 0 --neighborSize 32 \
--map data/map/den520d.map \
--agents random-32-32-20-random-1.scen \
--state map-random-32-32-20-scene-1-agent-150.json \
--agentNum 150 \
--cutoffTime 300

# map-den520d-scene-20-agent-900_NBopt_0_NBsize_16.log
# map-den520d-scene-20-agent-900_NBopt_0_NBsize_32.log
# map-den520d-scene-20-agent-900_NBopt_0_NBsize_4.log
# map-den520d-scene-20-agent-900_NBopt_0_NBsize_8.log
# map-den520d-scene-20-agent-900_NBopt_1_NBsize_4.log

python data/generate_init_state.py \
--map_path data/map/den520d.map \
--scene_folder data/scene \
--scene_start_idx 0 \
--scene_end_idx 10 \
--agent_num 900 

python script/get_avg_result_from_log.py \
--log_folder output/nb_baseline \
--log_key_word den520d agent-900 NBopt_1_NBsize_4 \
--time_threshold 300

Average result:  1718.36
Number of results:  25

python script/get_avg_result_from_log.py \
--log_folder output/nb_baseline \
--log_key_word den520d agent-900 NBopt_0_NBsize_16 \
--time_threshold 300
Average result:  1484.96
Number of results:  25

python script/get_avg_result_from_log.py \
--log_folder output/nb_baseline \
--log_key_word den520d agent-900 NBopt_0_NBsize_32 \
--time_threshold 300
Average result:  2302.36
Number of results:  25

python script/get_avg_result_from_log.py \
--log_folder output/nb_baseline \
--log_key_word den520d agent-900 NBopt_0_NBsize_4 \
--time_threshold 300

Average result:  2901.24
Number of results:  25

python script/get_avg_result_from_log.py \
--log_folder output/nb_baseline \
--log_key_word den520d agent-900 NBopt_0_NBsize_8 \
--time_threshold 300

Average result:  1622.32
Number of results:  25

python script/get_avg_result_from_log.py \
--log_folder output/nb_adp \
--log_key_word den520d agent-900  NBopt_3 \
--time_threshold 300

Average result:  7213.68
Number of results:  25

python script/get_avg_result_from_log.py \
--log_folder output/nb_adp_no_time \
--log_key_word den520d agent-900  NBopt_3 \
--time_threshold 300

Average result:  6445.88
Number of results:  25


python script/get_avg_result_from_log.py \
--log_folder output/nb_adp_random \
--log_key_word den520d agent-900  NBopt_3 \
--time_threshold 300



python script/get_avg_result_from_log.py \
--log_folder output/nb_adp_random \
--log_key_word den520d agent-900  NBopt_3 \
--time_threshold 300
Results:  [8987, 8285, 8906, 10017, 5232, 4215, 11276, 7649, 4951, 8568, 5704, 6816, 6270, 9728, 6468, 5673, 9230, 10071, 7202, 7731, 9818, 10047, 7694, 8994, 10264]
Average result:  7991.84
Number of results:  25


python script/get_avg_result_from_log.py \
--log_folder output/nb_adp_16_debug \
--log_key_word den520d agent-900  NBopt_3 \
--time_threshold 300

Average result:  6418.24
Number of results:  25


python script/get_avg_result_from_log.py \
--log_folder output/nb_adp \
--log_key_word den520d agent-900  NBopt_3 \
--time_threshold 300


Average result:  6901.08
Number of results:  25


python script/get_avg_result_from_log.py \
--log_folder output/nb_adp \
--log_key_word den520d agent-900  NBopt_1 \
--time_threshold 300


Average result:  8085.28
Number of results:  25


python script/get_avg_result_from_log.py \
--log_folder output/nb_adp \
--log_key_word den520d agent-900  NBopt_0 \
--time_threshold 300

Average result:  6731.28
Number of results:  25