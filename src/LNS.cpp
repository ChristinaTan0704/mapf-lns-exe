#include "LNS.h"


bool LNS::run()
{
    start_time = Time::now();
    if (!getInitialSolution())
        return false;

    runtime = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 2)
        cout << "Initial solution cost = " << initial_sum_of_costs << ", "
             << "runtime = " << runtime << endl;
    iteration_stats.emplace_back(neighbor.agents.size(), initial_sum_of_costs, runtime);



    bool succ;
    while (runtime < time_limit && iteration_stats.size() < num_of_iterations)
    {
        runtime =((fsec)(Time::now() - start_time)).count();
        if (ALNS)
            updateDestroyHeuristicbyALNS();

        switch (destroy_strategy)
        {
            case RANDOMWALK:
                generateNeighborByRandomWalk(tabu_list);
                break;
            case INTERSECTION:
                succ = generateNeighborByTemporalIntersection();
                if(!succ) // the selected intersection has fewer than 2 agents
                    continue;
                break;
            default:
                cerr << "Wrong neighbor generation strategy" << endl;
                exit(-1);
        }

        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;
        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            if (replan_algo_name == "PP")
                neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
            path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
            neighbor.old_sum_of_costs += neighbor.old_paths[i].size() - 1;
        }

        if (replan_algo_name == "EECBS")
            succ = runEECBS();
        else if (replan_algo_name == "PP")
            succ = runPP();
        else
        {
            cerr << "Wrong replanning strategy" << endl;
            exit(-1);
        }

        if (ALNS) // update destroy heuristics
        {
            if (neighbor.old_sum_of_costs > neighbor.sum_of_costs )
                destroy_weights[destroy_strategy] = reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs)
                                                       + (1 - reaction_factor) * destroy_weights[(int)destroy_strategy];
            else
                destroy_weights[destroy_strategy] = (1 - decay_factor) * destroy_weights[(int)destroy_strategy];
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        if (screen >= 2)
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "remaining time = " << time_limit - runtime << endl;
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime);
    }
    return true;
}


bool LNS::getInitialSolution()
{
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++)
        neighbor.agents[i] = i;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    bool succ = false;
    if (init_algo_name == "EECBS")
        succ = runEECBS();
    else if (init_algo_name == "PP")
        succ = runPP();
    else
    {
        cerr <<  "Initial MAPF solver " << init_algo_name << " does not exist!" << endl;
        exit(-1);
    }
    if (succ)
    {
        initial_sum_of_costs = neighbor.sum_of_costs;
        sum_of_costs = neighbor.sum_of_costs;
        return true;
    }
    else
        return false;

}

bool LNS::runEECBS()
{
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents)
    {
        search_engines.push_back(&agents[i].path_planner);
    }

    ECBS ecbs(search_engines, path_table, screen - 1);
    ecbs.setPrioritizeConflicts(true);
    ecbs.setDisjointSplitting(false);
    ecbs.setBypass(true);
    ecbs.setRectangleReasoning(true);
    ecbs.setCorridorReasoning(true);
    ecbs.setHeuristicType(heuristics_type::WDG, heuristics_type::GLOBAL);
    ecbs.setTargetReasoning(true);
    ecbs.setMutexReasoning(false);
    ecbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    ecbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    ecbs.setSavingStats(false);
    ecbs.setHighLevelSolver(high_level_solver_type::EES, 2);
    runtime = ((fsec)(Time::now() - start_time)).count();
    bool succ = ecbs.solve(time_limit - runtime, 0);
    if (succ && ecbs.solution_cost < neighbor.old_sum_of_costs) // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++)
        {
            agents[*id].path = *ecbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = ecbs.solution_cost;
    }
    else // stick to old paths
    {
        for (int id : neighbor.agents)
        {
            path_table.insertPath(agents[id].id, agents[id].path);
        }
        neighbor.sum_of_costs = neighbor.old_sum_of_costs;
    }
    return succ;
}


bool LNS::runPP()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 3) {
        for (auto id : shuffled_agents)
            cout << id << "(" << agents[id].path_planner.my_heuristic[agents[id].path_planner.start_location] <<
                "->" << agents[id].path.size() - 1 << "), ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    while (p != shuffled_agents.end())
    {
        int id = *p;
        if (screen >= 3)
            cout << "Remaining agents = " << remaining_agents <<
                 ", remaining time = " << time_limit - runtime << " seconds. " << endl
                 << "Agent " << agents[id].id << endl;
        agents[id].path = agents[id].path_planner.findOptimalPath(path_table);
        if (agents[id].path.empty())
            break;
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs)
            break;
        path_table.insertPath(agents[id].id, agents[id].path);
        remaining_agents--;
        ++p;
    }
    if (p == shuffled_agents.end() && neighbor.sum_of_costs < neighbor.old_sum_of_costs) // accept new paths
    {
        return true;
    }
    else // stick to old paths
    {
        auto p2 = shuffled_agents.begin();
        while (p2 != p)
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        p2 = neighbor.agents.begin();
        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            int a = *p2;
            agents[a].path = neighbor.old_paths[i];
            path_table.insertPath(agents[a].id, agents[a].path);
            ++p2;
        }
        neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        return false;
    }
}

void LNS::updateDestroyHeuristicbyALNS()
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
    int i = 0;
    while (threshold < r * sum)
    {
        i++;
        threshold += destroy_weights[i];
    }
    switch (i)
    {
        case 0 : destroy_strategy = RANDOMWALK; break;
        case 1 : destroy_strategy = INTERSECTION; break;
        default : cerr << "ERROR" << endl; exit(-1);
    }
}

bool LNS::generateNeighborByTemporalIntersection()
{
    /*if (intersections.empty())
    {
        for (int i = 0; i < ml.map_size(); i++)
        {
            if (ml.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    int location = intersections[rand() % intersections.size()];
    al.constraintTable.get_agents(neighbors_set, group_size, location);
    if (neighbors_set.size() <= 1)
        return false;
    neighbors.assign(neighbors_set.begin(), neighbors_set.end());
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by intersection " << location << endl;
    return true;*/
    return false;
}

bool LNS::generateNeighborByIntersection()
{
    /*if (intersections.empty())
    {
        for (int i = 0; i < ml.map_size(); i++)
        {
            if (ml.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    int location = intersections[rand() % intersections.size()];
    al.constraintTable.get_agents(neighbors_set, location);
    if (neighbors_set.size() <= 1)
        return false;
    neighbors.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbors.size() > max_group_size || (replan_strategy == 0 && neighbors.size() > group_size)) // resize the group for CBS
    {
        sortNeighborsRandomly();
        if (replan_strategy == 0)
            neighbors.resize(group_size);
        else
            neighbors.resize(max_group_size);
    }
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by intersection " << location << endl;
    return true;*/
    return false;
}

void LNS::generateNeighborByRandomWalk(boost::unordered_set<int>& tabu_list)
{
    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return;
    }

    // find the agent with max regret
    int a = -1;
    int max_delays = -1;
    for (int i = 0; i < agents.size(); i++)
    {
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        int delays = agents[i].path.size() - 1
                - agents[i].path_planner.my_heuristic[agents[i].path_planner.start_location];
        if (max_delays < delays)
        {
            a = i;
            max_delays = delays;
        }
    }
    if (tabu_list.size() > agents.size() / 2)
        tabu_list.clear();
    else
        tabu_list.insert(a);

    set<int> neighbors_set;
    neighbors_set.insert(a);
    int T = agents[a].path.size();
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10 && T > 0)
    {
        int t = rand() % T;
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        T = t;
        count++;
    }

    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
             << "(" << agents[a].path_planner.my_heuristic[agents[a].path_planner.start_location]
             << "->" << agents[a].path.size() - 1 << ")" << endl;
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



void LNS::replanByPP()
{
    updateNeighborPaths();
    deleteNeighborPaths();
    al.num_of_agents = 1;
    al.agents.resize(1);
    list<Path> new_paths;
    int sum_of_costs = 0;
    int sum_of_showup_time = 0;
    int makespan = 0;
    for (const auto& agent : neighbors)
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        if (runtime >= soft_time_limit)
        { // change back to the original paths
            auto path = neighbor_paths.begin();
            for (const auto& agent : neighbors)
            {
                al.paths_all[agent] = *path;
                ++path;
            }
            return;
        }
        al.agents[0] = &al.agents_all[agent];
//        MultiMapICBSSearch<FlatlandLoader> icbs(&ml, &al, f_w, c, 0, options1.debug? 3 : 0, options1);
//        icbs.runICBSSearch();
//        updateCBSResults(icbs);
//        addAgentPath(agent, *icbs.paths[0]);
        SinglePlanning planner(ml,al,f_w,0,options1);
        planner.search();
        updateCBSResults(planner);
        addAgentPath(agent, planner.path);
        assert(planner.path.back().location == al.paths_all[agent].back().location);

        if (planner.path.empty())
        {
            sum_of_costs += max_timestep;
            makespan = max_timestep;
        }
        else
        {
            sum_of_costs += (int)planner.path.size() - 1;
            makespan = max(makespan, (int)planner.path.size() - 1);
            for (int t  = 0; t < (int)planner.path.size(); t++)
            {
                if (planner.path.at(t).location >= 0)
                {
                    sum_of_showup_time += t;
                    break;
                }
            }
        }
    }
    if (sum_of_costs < neighbor_sum_of_costs ||
        (sum_of_costs == neighbor_sum_of_costs && sum_of_showup_time > neighbor_sum_of_showup_time) ||
        (sum_of_costs == neighbor_sum_of_costs && sum_of_showup_time == neighbor_sum_of_showup_time && makespan < neighbor_makespan))
    {
        delta_costs = sum_of_costs - neighbor_sum_of_costs;
    }
    else
    { // change back to the original paths
        deleteNeighborPaths();
        auto path = neighbor_paths.begin();
        for (auto agent : neighbors)
        {
            addAgentPath(agent, *path);
            ++path;
        }
        delta_costs = 0;
    }
}

bool LNS::replanByCBS()
{
    updateNeighborPathsCosts();
    deleteNeighborPaths();
    al.num_of_agents = (int)neighbors.size();
    al.agents.clear();
    for (auto i : neighbors)
    {
        al.agents.push_back(&al.agents_all[i]);
    }
    runtime = ((fsec)(Time::now() - start_time)).count();
    double cbs_time_limit = min((double)soft_time_limit - runtime, 10.0);
    MultiMapICBSSearch<FlatlandLoader> icbs(&ml, &al, f_w, c, cbs_time_limit, options1.debug? 3 : 0, options1);
    icbs.trainCorridor1 = trainCorridor1;
    icbs.corridor2 = corridor2;
    icbs.chasing_reasoning = chasing;
    if (options1.debug)
        cout << "start search engine" << endl;
    bool res = icbs.runICBSSearch();
    updateCBSResults(icbs);
    // assert(!res || icbs.solution_cost <= neighbor_sum_of_costs);
    if (res && icbs.solution_cost <= neighbor_sum_of_costs)
    {
        int i = 0;
        for (auto n : neighbors)
        {
            assert(icbs.paths[i]->back().location == al.paths_all[n].back().location);
            addAgentPath(n, *icbs.paths[i]);
            i++;
        }
        delta_costs = icbs.solution_cost - neighbor_sum_of_costs;
    }
    else
    {
        for (auto i : neighbors)
        {
            if(!al.constraintTable.insert_path(i, al.paths_all[i]))
                exit(13);
        }
        delta_costs = 0;
    }
    return res;
}


void LNS::updateNeighborPaths()
{
    if (options1.debug)
        cout << "Agents ids: ";
    neighbor_sum_of_costs = 0;
    neighbor_sum_of_showup_time = 0;
    neighbor_makespan = 0;
    neighbor_paths.clear();
    for (auto i : neighbors)
    {
        if (options1.debug)
            cout << i << ",";
        neighbor_paths.emplace_back(al.paths_all[i]);
        if (al.paths_all[i].empty())
        {
            neighbor_sum_of_costs += max_timestep;
            neighbor_makespan = max_timestep;
        }
        else
        {
            neighbor_sum_of_costs += (int)al.paths_all[i].size() - 1;
            neighbor_makespan = max(neighbor_makespan, (int)al.paths_all[i].size() - 1);
            for (int t  = 0; t < (int)al.paths_all[i].size(); t++)
            {
                if (al.paths_all[i][t].location >= 0)
                {
                    neighbor_sum_of_showup_time += t;
                    break;
                }
            }
        }
    }
    if (options1.debug)
        cout << endl;
}

void LNS::updateNeighborPathsCosts()
{
    if (options1.debug)
        cout << "Agents ids: ";
    neighbor_sum_of_costs = 0;
    neighbor_makespan = 0;
    neighbor_paths.clear();
    for (auto i : neighbors)
    {
        if (options1.debug)
            cout << i << ",";
        neighbor_sum_of_costs += (int)al.paths_all[i].size() - 1;
        neighbor_makespan = max(neighbor_makespan, (int)al.paths_all[i].size() - 1);
    }
    if (options1.debug)
        cout << endl;
}

void LNS::addAgentPath(int agent, const Path& path)
{
    assert(agent == al.agents_all[agent].agent_id);
    if(!al.constraintTable.insert_path(agent, path))
        exit(10);
    al.paths_all[agent] = path;
}

void LNS::deleteNeighborPaths()
{
    for (auto i : neighbors)
    {
        assert(i == al.agents_all[i].agent_id);
        al.constraintTable.delete_path(i, al.paths_all[i]);
    }
}

void LNS::sortNeighborsRandomly()
{
    std::random_shuffle(neighbors.begin(), neighbors.end());
    if (options1.debug) {
        for (auto agent : neighbors) {
            cout << agent << "(" << al.agents_all[agent].distance_to_goal << "->" << al.paths_all[agent].size() - 1
                 << "), ";
        }
        cout << endl;
    }
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