# LNS REMOVAL & REPLAN

The removal and replan functions are from [MAPF-LNS](https://github.com/Jiaoyang-Li/MAPF-LNS). This repository is directly modified based on [MAPF-LNS](https://github.com/Jiaoyang-Li/MAPF-LNS) and enables users to call the removal and replan functions multiple times without changing the current state.


## Installation 
The code requires the external libraries 
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
## Example Usage


## References
[1] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig.
Anytime Multi-Agent Path Finding via Large Neighborhood Search.
In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 4127-4135, 2021.         

 

 
