#pragma once
#include "ECBS.h"
#include "SpaceTimeAStar.h"
#include <chrono>
#include <utility>

//pibt related
#include "simplegrid.h"
#include "pibt_agent.h"
#include "problem.h"
#include "mapf.h"
#include "pibt.h"
#include "pps.h"
#include "winpibt.h"
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;
enum destroy_heuristic { RANDOMWALK, INTERSECTION, RANDOMAGENTS, RANDOMWALKLARGE, RANDOMWALKPROB }; //

struct Agent
{
    int id;
    SpaceTimeAStar path_planner; // start, goal, and heuristics are stored in the path planner
    Path path;

    Agent(const Instance& instance, int id) : id(id), path_planner(instance, id) {}

    int getNumOfDelays() const { return (int) path.size() - 1 - path_planner.my_heuristic[path_planner.start_location]; }

};


struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;
    vector<Path> old_paths;
};


class LNS
{
public:
    vector<Agent> agents;
    vector<int> delayed_agents;
    vector<int> delay_list;
    list<IterationStats> iteration_stats; //stats about each iteration
    double tabu_discount = 0.5; // delayed agent selected probability discount if it's in the tabu_list
    int uniform_neighbor;
    double preprocessing_time = 0;
    double initial_solution_runtime = 0;
    double runtime = 0;
    int initial_sum_of_costs = -1;
    int sum_of_costs = -1;
    int sum_of_costs_lowerbound = -1;
    int sum_of_distances = -1;
    double average_group_size = -1;
    int num_of_failures = 0; // #replanning that fails to find any solutions
    int num_of_low_level = 0;
    int pprun = 1;
    string state_json = "";
    double replan_time_limit = 600;
    double replan = true;
    LNS(const Instance& instance, double time_limit,
        string init_algo_name, string replan_algo_name, string destory_name,
        int neighbor_size, int num_of_subset, int screen, PIBTPPS_option pipp_option);

    bool getInitialSolution();
    bool run();
    void validateSolution() const;
    void writeIterStatsToFile(string file_name) const;
    void writeResultToFile(string file_name) const;
    void writePathsToFile(string file_name) const;
    string getSolverName() const { return "LNS(" + init_algo_name + ";" + replan_algo_name + ")"; }
    vector<double> destroy_weights = {1, 1, 0};
    unordered_set<int> tabu_list; // used by randomwalk strategy
    list<int> intersections;

private:
    int num_neighbor_sizes = 1; //4; // so the neighbor size could be 2, 4, 8, 16

    // input params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    string init_algo_name;
    string replan_algo_name;
    int screen;
    destroy_heuristic destroy_strategy = RANDOMWALK;
    int neighbor_size;
    int num_of_subset;

    high_resolution_clock::time_point start_time;


    PathTable path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.

    Neighbor neighbor;


    // adaptive LNS
    bool ALNS = false;
    double decay_factor = 0.01;
    double reaction_factor = 0.01;
    int selected_neighbor;

    bool runEECBS();
    bool runCBS();
    bool runPP();
    bool runPIBT();
    bool runPPS();
    bool runWinPIBT();

    PIBTPPS_option pipp_option;

    MAPF preparePIBTProblem(vector<int> shuffled_agents);
    void updatePIBTResult(const PIBT_Agents& A,vector<int> shuffled_agents);

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByRandomWalk();
    bool generateNeighborByRandomWalkProbSelect();
    bool generateNeighborByRandomWalkLarge();
    bool generateNeighborByIntersection(bool temporal = true);

    int findMostDelayedAgent();
    int findAgentBasedOnDelay();
    int findRandomAgent() const;
    void randomWalk(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);
    void randomWalkLarge(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);
};
