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
./lns-removal-replan --destroyStrategy [Intersection / RandomWalk / Random / Adaptive / RandomWalkProb ] \
--uniform_neighbor 0 --neighborSize 32 \
--map random-32-32-20.map \
--agents random-32-32-20-random-1.scen \
--state map-random-32-32-20-scene-1-agent-150.json \
--agentNum 150 \
--maxIterations 10000 \
--cutoffTime 300
```

<!-- ./lns-removal-replan --destroyStrategy RandomWalk \
--uniform_neighbor 0 --neighborSize 32 \
--map random-32-32-20.map \
--agents random-32-32-20-random-1.scen \
--state map-random-32-32-20-scene-1-agent-150.json \
--agentNum 150 \
--cutoffTime 300 -->


## References
[1] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig.
Anytime Multi-Agent Path Finding via Large Neighborhood Search.
In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 4127-4135, 2021.         

 

 
