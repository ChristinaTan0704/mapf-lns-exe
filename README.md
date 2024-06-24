# MAPF-LNS 

This repository is directly modified based on [MAPF-LNS](https://github.com/Jiaoyang-Li/MAPF-LNS) , with the addition of a RandomWalkProb heuristic. The input to this executable is a JSON file where the key is the agent ID and the value is a list of agent locations in 2D x, y coordinates (see [map-random-32-32-20-scene-1-agent-150.json](map-random-32-32-20-scene-1-agent-150.json) as an example). The delay and runtime information will be logged in the terminal. 


## Installation 
The code requires external libraries 
BOOST (https://www.boost.org/) and Eigen (https://eigen.tuxfamily.org/). 
An easy way to install the required libraries on Ubuntu:    
```shell script
sudo apt update
```
- Install the Eigen library (used for linear algebra computing)
 ```shell script
    sudo apt install libeigen3-dev
 ```
- Install the boost library 
 ```shell script
    sudo apt install libboost-all-dev
 ```
    
After you installed both libraries and downloaded the source code, 
go into the directory of the source code and compile it with CMake: 

```
cmake .
make 
```
## Usage


```python
./lns-removal-replan --destoryStrategy [Intersection / RandomWalk / Random / Adaptive / RandomWalkProb ] \
--uniform_neighbor 0 --neighborSize 32 \
--map random-32-32-20.map \


```

<!-- pre_work/baseline/MAPF-LNS/lns --destoryStrategy Random --uniform_neighbor 0 --neighborSize 32 -m pre_work/baseline/MAPF-LNS/map/den520d.map -a pre_work/baseline/MAPF-LNS/scene/den520d-random-9.scen -k 900 -t 1510 --initAlgo PP --maxIterations=2000 --state data/initial_state_json_10s/LNS2/map-den520d-scene-9-agent-900.json --log_step 50 -->


## References
[1] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig.
Anytime Multi-Agent Path Finding via Large Neighborhood Search.
In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 4127-4135, 2021.         

 

 
