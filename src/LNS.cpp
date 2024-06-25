#include "LNS.h"
#include <queue>
#include <random>
#include <array>
#include <nlohmann/json.hpp>

int getRandomFromSetExp() {
    std::array<int, 5> values = {2, 4, 8, 16, 32};
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, values.size() - 1);

    // Generate random index and return the value at that index
    int randomIndex = distrib(gen);
    return values[randomIndex];
}

int getRandomFromRange() {
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(5, 16); // Adjusted to cover the range [0, 31]

    // Generate and return a random number from the range [0, 31]
    return distrib(gen);
}

// --map pre_work/baseline/MAPF-LNS/map/Paris_1_256.map --agentNum 650 --state data/initial_state_json_10s/LNS2/map-Paris_1_256-scene-1-agent-650.json --adaptive_weight 1 1 --pprun 1 --num_subset 20 --uniform_neighbor 2 --replanTime 0.6
LNS::LNS(const Instance& instance, double time_limit, string init_algo_name, string replan_algo_name, string destory_name,
         int neighbor_size, int num_of_subset, int screen, PIBTPPS_option pipp_option) :
         instance(instance), time_limit(time_limit), init_algo_name(std::move(init_algo_name)),
         replan_algo_name(replan_algo_name), neighbor_size(neighbor_size), num_of_subset(num_of_subset),
         screen(screen), path_table(instance.map_size),pipp_option(pipp_option)
{
    start_time = Time::now();
    if (destory_name == "Adaptive")
    {
        ALNS = true;
        destroy_weights.assign(3, 1);
    }
    else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else if (destory_name == "RandomWalkLarge")
        destroy_strategy = RANDOMWALKLARGE;
    else if (destory_name == "RandomWalkProb")
        destroy_strategy = RANDOMWALKPROB;
    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    for (int i = 0; i < N; i++){
        agents.emplace_back(instance, i);
    }

    preprocessing_time = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 2)
        cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
    int debug;
}

bool LNS::run()
{

    // only for statistic analysis, and thus is not included in runtime
    initial_solution_runtime = 0;
    bool succ = true;
    int count = 0;
    // load initial state from json file
    using json = nlohmann::json;
    std::ifstream f(state_json);
    json data = json::parse(f);
    neighbor.sum_of_costs = 0;
    for (auto & [key, value] : data.items()){
        Path path;
        int id = std::stoi(key);
        for (const auto& pair : value) {
            int row = static_cast<int>(pair[0]);
            int col = static_cast<int>(pair[1]);
            auto loc = instance.num_of_cols * row + col;
            path.emplace_back(PathEntry(loc));
        }
        agents[id].path = path;
        neighbor.sum_of_costs += path.size() - 1;
        path_table.insertPath(agents[id].id, agents[id].path);
    }
    initial_sum_of_costs = neighbor.sum_of_costs;
    sum_of_costs = neighbor.sum_of_costs;
    cout << "load initial state from " << state_json << endl;
    cout << "Initial solution cost = " << initial_sum_of_costs << endl;
    neighbor_size = replan_agents.size();
    // set the replanend agents
    neighbor.agents.resize(neighbor_size);
    for (int i = 0; i < neighbor_size; i++){
        neighbor.agents[i] = agents[replan_agents[i]].id;
    }

    succ = true;
    // print the replanned agents 
    cout << "replaned agents : ";
    for (auto id : neighbor.agents)
        cout << agents[id].id << " ";
    cout << endl;

    neighbor.old_paths.resize(neighbor.agents.size());
    neighbor.old_sum_of_costs = 0;
    for (int i = 0; i < (int)neighbor.agents.size(); i++)
    {
        if (replan_algo_name == "PP")
            neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
        path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
        neighbor.old_sum_of_costs += agents[neighbor.agents[i]].path.size() - 1;
    }
    cout << "one start end" << endl;

    runPP();

    cout << "one replan end" << endl;
    path_table.reset();


    return true;
}

bool LNS::generateNeighborByRandomWalkProbSelect()
{
    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    // clear and init delayed_agents and delay_list before each lns iteration
    delayed_agents.clear();
    delay_list.clear();
    int max_delay = 0;
    for (int i = 0; i < agents.size(); i++)
    {
        int agent_delay = agents[i].getNumOfDelays();
        if (agent_delay > max_delay) max_delay = agent_delay;
        if (agent_delay > 0){
            delayed_agents.push_back(i);
            // if the agent is selected in previous rounds, discount it by tabu_discount
            if (tabu_list.find(i) != tabu_list.end()){
                delay_list.push_back(agent_delay*tabu_discount);
            }else{
                delay_list.push_back(agent_delay);
            }
        }
    }


    set<int> neighbors_set;
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10)
    {
        int a = findAgentBasedOnDelay();
        if (a < 0)
            return false;

        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;
//        cout << "## randomwalk_iter : " << count << " removal_set_size : " << neighbors_set.size() << " agent : " << a << " delay : " << agents[a].getNumOfDelays() << " (" << max_delay << ")" << endl;

    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());


    return true;
}




int LNS::findAgentBasedOnDelay() {
    // Ensure both vectors are of the same size
    if (delayed_agents.size() != delay_list.size()) {
        throw std::invalid_argument("Vectors must be of the same size.");
    }

    // Compute the cumulative sum of delay_list
    std::vector<int> cumulative_weights(delay_list.size());
    std::partial_sum(delay_list.begin(), delay_list.end(), cumulative_weights.begin());

    // Generate a random number in the range [0, total weight)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, cumulative_weights.back() - 1);

    int random_number = distrib(gen);

    // Find the index where random_number fits in cumulative_weights
    auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), random_number);

    // Calculate the index based on the iterator
    int index = std::distance(cumulative_weights.begin(), it);

    // Return the corresponding element from delayed_agents
    tabu_list.insert(delayed_agents[index]);
    delay_list[index] *= tabu_discount; // if selected in the current LNS round, discount it again
    return delayed_agents[index];
}




bool LNS::runPP()
{   
    double pp_sum_of_costs = 0; // sum of costs of the new paths of all pp runs
    double pp_sum_of_time = 0; // sum of time of all pp runs
    auto shuffled_agents = neighbor.agents;
    int replan_iter = 0;
    bool first_iter = true;


    while (replan_iter < pprun){
        replan_iter += 1;
        auto replan_start_time = Time::now();
        std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
        int remaining_agents = (int)shuffled_agents.size();
        auto p = shuffled_agents.begin();
        neighbor.sum_of_costs = 0;

        while (p != shuffled_agents.end() && ((fsec)(Time::now() - replan_start_time)).count() < replan_time_limit)
        {
            int id = *p;
            agents[id].path = agents[id].path_planner.findOptimalPath(path_table, replan_time_limit);
            if (agents[id].path.empty())
            {
                break;
            }
            neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
            path_table.insertPath(agents[id].id, agents[id].path);
            remaining_agents--;
            ++p;


        }
        pp_sum_of_time +=  ((fsec)(Time::now() - replan_start_time)).count() ;
        if (remaining_agents == 0){
            pp_sum_of_costs += neighbor.sum_of_costs;
        }
        else{
            pp_sum_of_costs += neighbor.old_sum_of_costs;
        }

        if (first_iter == true){
            if (neighbor.sum_of_costs < neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "Improved : " << neighbor.old_sum_of_costs - neighbor.sum_of_costs << endl;
                cout << "new paths start " << endl;
                for (auto id : shuffled_agents) {
                    cout << "agent " << agents[id].id << " ";
                    int t = 0;
                    for (auto loc: agents[id].path) {
                        // cout << "(" << t << "," << loc.location << ")" << " --> ";
                        auto coord = instance.getCoordinate(loc.location);
                        cout  << "(" << coord.first << "," << coord.second << ") --> ";
                        t += 1;
                    }
                    cout << endl;
                }
                cout << "new paths end " << endl;

            }
            else if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "No improvement" << endl;
            }
            else if (remaining_agents!=0){
                cout << "No path" << endl;
            }
        }
        else{
            if (neighbor.sum_of_costs < neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "Improved : " << neighbor.old_sum_of_costs - neighbor.sum_of_costs << endl;
            }else if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "No improvement" << endl;
            }
            else if (remaining_agents!=0){
                cout << "No path" << endl;
            }

        }
        first_iter = false; 



        // set back to old paths
        auto p2 = shuffled_agents.begin();
        while (p2 != p) // remove the planned paths
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        neighbor.sum_of_costs = neighbor.old_sum_of_costs;
    }

    for (int i = 0; i < (int)neighbor.agents.size(); i++)
    {
        auto a = neighbor.agents[i];
        agents[a].path = neighbor.old_paths[i];
        path_table.insertPath(agents[a].id, agents[a].path);

    }
    neighbor.sum_of_costs = neighbor.old_sum_of_costs;

    cout << "average_replan_time: " << pp_sum_of_time / pprun << endl;
    cout << "average_improvement: " << (neighbor.old_sum_of_costs * pprun - pp_sum_of_costs) / pprun << endl;



    return true;
}


void LNS::chooseDestroyHeuristicbyALNS()
{
    double sum = 0;
    for (const auto& h : destroy_weights)
        sum += h;
    if (screen >= 2)
    {
        cout << "destroy weights = ";
        for (const auto& h : destroy_weights)
            cout << h / sum << ",";
    }
    double r = (double) rand() / RAND_MAX;
    double threshold = destroy_weights[0];
    selected_neighbor = 0;
    while (threshold < r * sum)
    {
        selected_neighbor++;
        threshold += destroy_weights[selected_neighbor];
    }
    switch (selected_neighbor / num_neighbor_sizes)
    {
        case 0 : destroy_strategy = RANDOMWALK; break;
        case 1 : destroy_strategy = INTERSECTION; break;
        case 2 : destroy_strategy = RANDOMAGENTS; break;
        default : cerr << "ERROR" << endl; exit(-1);
    }
    // neighbor_size = (int) pow(2, selected_neighbor % num_neighbor_sizes + 1);
}

bool LNS::generateNeighborByIntersection(bool temporal)
{
    if (intersections.empty())
    {
        for (int i = 0; i < instance.map_size; i++)
        {
            if (!instance.isObstacle(i) && instance.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    auto pt = intersections.begin();
    std::advance(pt, rand() % intersections.size());
    int location = *pt;
    path_table.get_agents(neighbors_set, neighbor_size, location);
    if (neighbors_set.size() < neighbor_size)
    {
        set<int> closed;
        closed.insert(location);
        std::queue<int> open;
        open.push(location);
        while (!open.empty() && (int) neighbors_set.size() < neighbor_size)
        {
            int curr = open.front();
            open.pop();
            for (auto next : instance.getNeighbors(curr))
            {
                if (closed.count(next) > 0)
                    continue;
                open.push(next);
                closed.insert(next);
                if (instance.getDegree(next) >= 3)
                {
                    path_table.get_agents(neighbors_set, neighbor_size, next);
                    if ((int) neighbors_set.size() == neighbor_size)
                        break;
                }
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbor.agents.size() > neighbor_size)
    {
        std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
        neighbor.agents.resize(neighbor_size);
    }
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by intersection " << location << endl;
    return true;
}


int LNS::findMostDelayedAgent()
{
    int a = -1;
    int max_delays = -1;
    for (int i = 0; i < agents.size(); i++)
    {
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        int delays = agents[i].getNumOfDelays();
        if (max_delays < delays)
        {
            a = i;
            max_delays = delays;
            max_delays = delays;
        }
    }
    if (max_delays == 0 or a == -1)
    {
        tabu_list.clear();
        return -1;
    }
    tabu_list.insert(a);
//    cout << "insert " << a <<  " to tabu list " << endl;
    if (tabu_list.size() == agents.size())
        tabu_list.clear();
    return a;
}


bool LNS::generateNeighborByRandomWalk()
{



    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }
    cout << " findMostDelayedAgent " << endl; // TODO
    int a = findMostDelayedAgent();
    if (a < 0)
        return false;

    set<int> neighbors_set;
    neighbors_set.insert(a);
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10)
    {
        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set)
        {
            if (i == idx)
            {
                a = n; 
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
             << "(" << agents[a].path_planner.my_heuristic[agents[a].path_planner.start_location]
             << "->" << agents[a].path.size() - 1 << ")" << endl;

    return true;
}



bool LNS::generateNeighborByRandomWalkLarge()
{



    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    int a = findMostDelayedAgent();
    if (a < 0)
        return false;

    set<int> neighbors_set;
    neighbors_set.insert(a);
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 100)
    {
        int t = rand() % agents[a].path.size();
        randomWalkLarge(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set)
        {
            if (i == idx)
            {
                a = n;
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
             << "(" << agents[a].path_planner.my_heuristic[agents[a].path_planner.start_location]
             << "->" << agents[a].path.size() - 1 << ")" << endl;

 if (neighbors_set.size() != neighbor_size){
        return false;
    }

    return true;
}


int LNS::findRandomAgent() const
{
    int a = 0;
    int pt = rand() % (sum_of_costs - sum_of_distances) + 1;
    int sum = 0;
    for (; a < (int) agents.size(); a++)
    {
        sum += agents[a].getNumOfDelays();
        if (sum >= pt)
            break;
    }
    assert(sum >= pt);
    assert(sum >= pt);
    return a;
}

// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
void LNS::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner.my_heuristic[*it];
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
}



// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
void LNS::randomWalkLarge(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            auto temp = conflicting_agents;
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner.my_heuristic[*it];
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
            if (conflicting_agents.size() > neighbor_size){
                set <int> new_add;
                std::set_difference(conflicting_agents.begin(), conflicting_agents.end(), temp.begin(), temp.end(), std::inserter(new_add, new_add.begin()));
                int redundant_agents = conflicting_agents.size() - neighbor_size;
                int remove_num = 0;
                for (auto one_redundant_agent : new_add){
                    conflicting_agents.erase(one_redundant_agent);
                    remove_num += 1;
                    if (remove_num >= redundant_agents ){
                        break;
                    }
                }
                break;
            }
            else if(conflicting_agents.size() == neighbor_size){
                break;
            }

        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
}




void LNS::validateSolution() const
{
    int sum = 0;
    for (const auto& a1_ : agents)
    {
        if (a1_.path.empty())
        {
            cerr << "No solution for agent " << a1_.id << endl;
            exit(-1);
        }
        else if (a1_.path_planner.start_location != a1_.path.front().location)
        {
            cerr << "The path of agent " << a1_.id << " starts from location " << a1_.path.front().location
                << ", which is different from its start location " << a1_.path_planner.start_location << endl;
            exit(-1);
        }
        else if (a1_.path_planner.goal_location != a1_.path.back().location)
        {
            cerr << "The path of agent " << a1_.id << " ends at location " << a1_.path.back().location
                 << ", which is different from its goal location " << a1_.path_planner.goal_location << endl;
            exit(-1);
        }
        for (int t = 1; t < (int) a1_.path.size(); t++ )
        {
            if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
            {
                cerr << "The path of agent " << a1_.id << " jump from "
                     << a1_.path[t - 1].location << " to " << a1_.path[t].location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                exit(-1);
            }
        }
        sum += (int) a1_.path.size() - 1;
        for (const auto& a2_: agents)
        {
            if (a1_.id >= a2_.id || a1_.path.empty())
                continue;
            const auto a1 = a1_.path.size() <= a2_.path.size()? a1_ : a2_;
            const auto a2 = a1_.path.size() <= a2_.path.size()? a2_ : a1_;
            int t = 1;
            for (; t < (int) a1.path.size(); t++)
            {
                if (a1.path[t].location == a2.path[t].location) // vertex conflict
                {
                    cerr << "Find a vertex conflict between agents " << a1.id << " and " << a2.id <<
                            " at location " << a1.path[t].location << " at timestep " << t << endl;
                    exit(-1);
                }
                else if (a1.path[t].location == a2.path[t - 1].location &&
                        a1.path[t - 1].location == a2.path[t].location) // edge conflict
                {
                    cerr << "Find an edge conflict between agents " << a1.id << " and " << a2.id <<
                         " at edge (" << a1.path[t - 1].location << "," << a1.path[t].location <<
                         ") at timestep " << t << endl;
                    exit(-1);
                }
            }
            int target = a1.path.back().location;
            for (; t < (int) a2.path.size(); t++)
            {
                if (a2.path[t].location == target)  // target conflict
                {
                    cerr << "Find a target conflict where agent " << a2.id << " traverses agent " << a1.id <<
                         "'s target location " << target << " at timestep " << t << endl;
                    exit(-1);
                }
            }
        }
    }
    if (sum_of_costs != sum)
    {
        cerr << "The computed sum of costs " << sum_of_costs <<
             " is different from the sum of the paths in the solution " << sum << endl;
        exit(-1);
    }
}

void LNS::writeIterStatsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << "num of agents," <<
           "sum of costs," <<
           "runtime," <<
           "cost lowerbound," <<
           "sum of distances," <<
           "MAPF algorithm" << endl;

    for (const auto &data : iteration_stats)
    {
        output << data.num_of_agents << "," <<
               data.sum_of_costs << "," <<
               data.runtime << "," <<
               max(sum_of_costs_lowerbound, sum_of_distances) << "," <<
               sum_of_distances << "," <<
               data.algorithm << endl;
    }
    output.close();
}

void LNS::writeResultToFile(string file_name) const
{
    std::ifstream infile(file_name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(file_name);
        addHeads << "runtime,solution cost,initial solution cost,min f value,root g value," <<
                 "iterations," <<
                 "group size," <<
                 "runtime of initial solution,area under curve," <<
                 "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
    ofstream stats(file_name, std::ios::app);
    double auc = 0;
    if (!iteration_stats.empty())
    {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit)
        {
            auc += (prev->sum_of_costs - sum_of_distances) * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += (prev->sum_of_costs - sum_of_distances) * (time_limit - prev->runtime);
    }
    stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs << "," <<
            max(sum_of_distances, sum_of_costs_lowerbound) << "," << sum_of_distances << "," <<
            iteration_stats.size() << "," << average_group_size << "," <<
            initial_solution_runtime << "," << auc << "," <<
            preprocessing_time << "," << getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}

void LNS::writePathsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << agents.size() << endl;

    for (const auto &agent : agents)
    {
        for (const auto &state : agent.path)
            output << state.location << ",";
        output << endl;
    }
    output.close();
}
/*
bool LNS::generateNeighborByStart()
{
    if (start_locations.empty())
    {
        for (int i = 0; i < (int)al.agents_all.size(); i++)
        {
            auto start = ml.linearize_coordinate(al.agents_all[i].initial_location);
            start_locations[start].push_back(i);
        }
        auto it = start_locations.begin();
        while(it != start_locations.end()) // delete start locations that have only one agent
        {
            if (it->second.size() == 1)
                it = start_locations.erase(it);
            else
                ++it;
        }
    }
    if (start_locations.empty())
        return false;
    auto step = rand() % start_locations.size();
    auto it = start_locations.begin();
    advance(it, step);
    neighbors.assign(it->second.begin(), it->second.end());
    if (neighbors.size() > max_group_size ||
        (replan_strategy == 0 && neighbors.size() > group_size)) // resize the group for CBS
    {
        sortNeighborsRandomly();
        if (replan_strategy == 0)
            neighbors.resize(group_size);
        else
            neighbors.resize(max_group_size);
    }
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by start location " << it->first << endl;
    return true;
}

void LNS::sortNeighborsByRegrets()
{
    quickSort(neighbors, 0, neighbors.size() - 1, true);
    if (options1.debug) {
        for (auto agent : neighbors) {
            cout << agent << "(" << al.agents_all[agent].distance_to_goal << "->" << al.paths_all[agent].size() - 1
                 << "), ";
        }
        cout << endl;
    }
}

void LNS::sortNeighborsByStrategy()
{
    if (agent_priority_strategy == 5)
    {
        // decide the agent priority for agents at the same start location
        start_locations.clear(); // map the agents to their start locations
        for (auto i : neighbors)
            start_locations[ml.linearize_coordinate(al.agents_all[i].initial_location)].push_back(i);
        for (auto& agents : start_locations)
        {
            vector<int> agents_vec(agents.second.begin(), agents.second.end());
            quickSort(agents_vec, 0, agents_vec.size() - 1, false);
            for (int i = 0; i < (int)agents.second.size(); i++)
            {
                al.agents_all[agents_vec[i]].priority = i;
            }
        }
    }

    // sort the agents
    if (agent_priority_strategy != 0)
        quickSort(neighbors, 0, (int)neighbors.size() - 1, false);
}


void LNS::quickSort(vector<int>& agent_order, int low, int high, bool regret)
{
    if (low >= high)
        return;
    int pivot = agent_order[high];    // pivot
    int i = low;  // Index of smaller element
    for (int j = low; j <= high - 1; j++)
    {
        // If current element is smaller than or equal to pivot
        if ((regret && compareByRegrets(agent_order[j], pivot)) ||
            al.compareAgent(al.agents_all[agent_order[j]], al.agents_all[pivot], agent_priority_strategy))
        {
            std::swap(agent_order[i], agent_order[j]);
            i++;    // increment index of smaller element
        }
    }
    std::swap(agent_order[i], agent_order[high]);

    quickSort(agent_order, low, i - 1, regret);  // Before i
    quickSort(agent_order, i + 1, high, regret); // After i
}



*/